# fsai-open: conformance tests, one per clause of ADS-DV Software Interface Specification v4.0
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Read these tests next to the spec: each test name starts with the clause it checks.

Two kinds of test, both needed:
  * "bridge obeys": our bridge never sends a frame that breaks the rule (checked by an independent
    observer in the bench, not by the bridge itself).
  * "emulator enforces": a deliberately bad AI breaks the rule, and the emulated VCU reacts as the
    spec says. This tests the test bench, so a passing bridge result means something.
"""
import random

import pytest

from fsai import MASTER_SEED
from fsai.bridge.core import DriveCommand
from fsai.params import Params, load
from fsai.vcu.emulator import AS_DRIVING, AS_EBRAKE, AS_FINISHED, AS_OFF, AS_READY
from fsai.vcu.lockstep import Bench

P = load()


def driving_bench(params=None, **kw) -> Bench:
    b = Bench(params or P, **kw)
    assert b.arm(1), b.vcu.events
    assert b.go(), b.vcu.events
    return b


def drive(b: Bench, seconds: float, speed=8.0, steer=0.0):
    for _ in range(int(seconds / 0.01)):
        b.bridge.set_command(DriveCommand(speed_mps=speed, steer_deg=steer), b.t)
        b.step()


def tamper(field_name, value, from_cycle, fid):
    def override(cycle, frame_id, vals):
        if frame_id == fid and cycle >= from_cycle:
            vals = dict(vals)
            vals[field_name] = value
            return vals
        return None
    return override


# ---------------------------------------------------------------- 2.3 reserved IDs
def test_2_3_bridge_never_transmits_on_reserved_ids():
    from fsai.can.signals import is_reserved
    b = driving_bench()
    drive(b, 3)
    assert b.sent_ids == {0x510, 0x511, 0x512, 0x513, 0x514}
    assert not any(is_reserved(i) for i in b.sent_ids)


def test_2_3_transmit_guard_refuses_reserved_ids():
    from fsai.bridge.core import TransmitGuardError, guard_tx
    for fid in (0x000, 0x080, 0x124, 0x184, 0x600, 0x640, 0x705):
        with pytest.raises(TransmitGuardError):
            guard_tx(fid)
    guard_tx(0x510)


# ---------------------------------------------------------------- 2.4 handshake and comms loss
def test_2_4_handshake_survives_60s_with_jitter_below_the_limit():
    rng = random.Random(MASTER_SEED)
    stalls = set()
    c = 800
    lim = P.vcu.comms_timeout_cycles
    while c < 6000:                                     # random stalls of 1..limit-1 cycles
        n = rng.randint(1, lim - 1)
        stalls.update(range(c, c + n))
        c += n + rng.randint(5, 60)
    b = driving_bench(ai_stall=lambda cyc: cyc in stalls)
    drive(b, 50)
    assert b.vcu.state == AS_DRIVING, b.vcu.events
    assert not b.vcu.faults.ai_comms_lost


@pytest.mark.parametrize("limit", [5, 10])
@pytest.mark.parametrize("below, lost", [(2, False), (1, False), (0, True), (-15, True)])
def test_2_4_ai_comms_lost_after_the_timeout(limit, below, lost):
    """FS-AI 2026 rules T1.1.8: the handshake must be answered within 50 ms (5 cycles). Spec v4.0 2.4
    says 10 consecutive missed messages (100 ms). Both settings are tested: limit-2 and limit-1 missed
    cycles are tolerated, limit missed cycles raise AI_COMMS_LOST."""
    stall_cycles = limit - below
    b = driving_bench(params=load(vcu__comms_timeout_cycles=limit))
    start = b.cycle + 1
    b.ai_stall = lambda cyc: start <= cyc < start + stall_cycles
    drive(b, 1)
    assert b.vcu.faults.ai_comms_lost is lost
    assert (b.vcu.state == AS_EBRAKE) is lost
    if lost:
        assert b.bridge.last_status["shutdown_cause"] == "AI_COMMS_FAULT"
        assert "AI_COMMS_LOST" in b.bridge.last_status["faults"]


@pytest.mark.parametrize("fid", [0x510, 0x511, 0x512, 0x513, 0x514])
def test_2_4_any_single_ai_frame_missing_triggers_comms_lost(fid):
    b = driving_bench()
    start = b.cycle + 1
    b.drop = lambda cyc, f: f == fid and cyc >= start
    drive(b, 0.5)
    assert b.vcu.state == AS_EBRAKE and b.vcu.faults.ai_comms_lost


def test_2_4_handshake_not_echoed_triggers_comms_lost():
    b = driving_bench()
    b.override = tamper("HANDSHAKE", 0, b.cycle + 1, 0x510)   # a stuck handshake bit
    drive(b, 0.5)
    assert b.vcu.state == AS_EBRAKE and b.vcu.faults.ai_comms_lost


# ---------------------------------------------------------------- 3.1 AS_OFF -> AS_READY
@pytest.mark.parametrize("missing", ["tsms", "asms", "ami", "ebs_armed"])
def test_3_1_ready_needs_all_four_conditions(missing):
    b = Bench(P)
    i = b.vcu.inputs
    i.tsms, i.asms, i.ami, i.ebs_armed = True, True, 1, True
    setattr(i, missing, 0 if missing == "ami" else False)
    b.run(3)
    assert b.vcu.state == AS_OFF


def test_3_1_bridge_confirms_mission_selected():
    b = Bench(P)
    assert b.arm(3)
    b.run(0.1)
    assert b.bridge.last_status["mission_status"] == "SELECTED"
    assert b.bridge.last_status["ami_state"] == "AUTOCROSS"


def test_3_1_strict_ack_variant_reaches_ready_with_bridge():
    b = Bench(load(vcu__require_selected_ack=True))
    assert b.arm(1)


# ---------------------------------------------------------------- 3.2 AS_READY -> AS_DRIVING
def test_3_2_bridge_sends_only_zero_requests_and_neutral_in_ready():
    b = Bench(P)
    assert b.arm(1)
    for _ in range(300):
        b.bridge.set_command(DriveCommand(speed_mps=10, steer_deg=15), b.t)   # team software too eager
        b.step()
    assert b.vcu.state == AS_READY
    assert not b.violations
    assert b.bridge.last_status["direction"] == "NEUTRAL"


def test_3_2_go_before_5s_is_ignored_and_must_be_cycled():
    b = Bench(P)
    assert b.arm(1)
    b.run(1.0)
    b.vcu.inputs.res_go = True                    # too early
    b.run(6.0)
    assert b.vcu.state == AS_READY                # rising edge was spent before the conditions held
    b.vcu.inputs.res_go = False
    b.run(0.1)
    b.vcu.inputs.res_go = True
    assert b.run(0.5, until=lambda x: x.vcu.state == AS_DRIVING)


def test_3_2_go_already_on_when_entering_ready_must_be_cycled():
    b = Bench(P)
    b.vcu.inputs.res_go = True
    assert b.arm(1)
    b.run(7)
    assert b.vcu.state == AS_READY
    b.vcu.inputs.res_go = False
    b.run(0.1)
    b.vcu.inputs.res_go = True
    assert b.run(0.5, until=lambda x: x.vcu.state == AS_DRIVING)


@pytest.mark.parametrize("field_name, value, fid", [
    ("STEER_REQUEST", 3.0, 0x513), ("REAR_AXLE_TRQ_REQUEST", 10.0, 0x512),
    ("FRONT_AXLE_TRQ_REQUEST", 10.0, 0x511), ("DIRECTION_REQUEST", 1, 0x510)])
def test_3_2_emulator_blocks_driving_unless_requests_are_zero(field_name, value, fid):
    b = Bench(P)
    assert b.arm(1)
    b.override = tamper(field_name, value, 0, fid)
    assert not b.go()
    assert b.vcu.state == AS_READY


def test_3_2_emulator_blocks_driving_if_wheels_steered_over_5_degrees():
    b = Bench(P)
    assert b.arm(1)
    b.vcu.steer = 6.0
    assert not b.go()
    b.vcu.steer = 4.9
    b.vcu.inputs.res_go = False
    b.run(0.05)
    b.vcu.inputs.res_go = True
    assert b.run(0.5, until=lambda x: x.vcu.state == AS_DRIVING)


def test_3_2_go_signal_reported_to_ai():
    b = driving_bench()
    b.run(0.05)
    assert b.bridge.last_status["go_signal"] == 1
    assert b.bridge.last_status["direction"] == "FORWARD"
    assert b.bridge.last_status["mission_status"] == "RUNNING"


# ---------------------------------------------------------------- 3.3 AS_DRIVING -> AS_FINISHED
def test_3_3_bridge_brakes_to_standstill_then_reports_finished():
    b = driving_bench()
    drive(b, 5, speed=10)
    assert b.vcu.v > 5
    b.bridge.request_finish()
    assert b.run(15, until=lambda x: x.vcu.state == AS_FINISHED), b.vcu.events
    assert b.vcu.wheel_rpm <= P.vcu.standstill_rpm and not b.violations   # spec 4: <= 10 rpm counts as stopped
    assert not b.vcu.faults.any()


def test_3_3_finished_state_ignores_drive_and_steer():
    b = driving_bench()
    drive(b, 1)
    b.bridge.request_finish()
    assert b.run(15, until=lambda x: x.vcu.state == AS_FINISHED)
    b.override = lambda cyc, fid, vals: {**vals, "REAR_AXLE_TRQ_REQUEST": 100.0,
                                         "REAR_MOTOR_SPEED_MAX": 3000} if fid == 0x512 else None
    b.run(2)
    assert b.vcu.v == 0


# ---------------------------------------------------------------- 3.4 / 3.6 ASMS off
def test_3_4_finished_to_off_when_asms_off():
    b = driving_bench()
    b.bridge.request_finish()
    assert b.run(10, until=lambda x: x.vcu.state == AS_FINISHED)
    b.vcu.inputs.asms = False
    b.run(0.05)
    assert b.vcu.state == AS_OFF


def test_3_6_ready_to_off_when_asms_off():
    b = Bench(P)
    assert b.arm(1)
    b.vcu.inputs.asms = False
    b.run(0.05)
    assert b.vcu.state == AS_OFF


# ---------------------------------------------------------------- 3.5 / 3.7 / 3.8 emergency brake
@pytest.mark.parametrize("phase", ["ready", "driving", "finished"])
def test_3_5_3_7_3_8_shutdown_circuit_open_triggers_ebrake(phase):
    b = Bench(P)
    assert b.arm(1)
    if phase != "ready":
        assert b.go()
        drive(b, 1)
    if phase == "finished":
        b.bridge.request_finish()
        assert b.run(10, until=lambda x: x.vcu.state == AS_FINISHED)
    b.vcu.inputs.sdc_closed = False
    b.run(0.05)
    assert b.vcu.state == AS_EBRAKE


@pytest.mark.parametrize("switch", ["res_go", "asms"])
def test_3_7_go_or_asms_off_while_driving_triggers_ebrake(switch):
    b = driving_bench()
    drive(b, 1)
    setattr(b.vcu.inputs, switch, False)
    b.run(0.05)
    assert b.vcu.state == AS_EBRAKE


def test_3_7_ai_estop_request_end_to_end():
    b = driving_bench()
    drive(b, 2, speed=8)
    b.bridge.request_estop()
    b.run(0.05)
    assert b.vcu.state == AS_EBRAKE
    b.run(0.05)
    assert b.bridge.last_status["shutdown_cause"] == "AI_COMPUTER_REQUEST"
    assert "AI_ESTOP_REQUEST" in b.bridge.last_status["faults"]
    assert b.run(5, until=lambda x: x.vcu.v == 0)


# ---------------------------------------------------------------- 3.9 EMERGENCY_BRAKE -> AS_OFF
def test_3_9_ebrake_holds_15s_then_off_and_needs_power_cycle():
    b = driving_bench()
    b.vcu.inputs.sdc_closed = False
    b.run(0.05)
    assert b.vcu.state == AS_EBRAKE
    b.vcu.inputs.asms = False
    b.run(14.5)
    assert b.vcu.state == AS_EBRAKE                 # 15 s timer not elapsed
    b.run(1.0)
    assert b.vcu.state == AS_OFF
    b.vcu.inputs.sdc_closed = True
    b.vcu.inputs.asms = True
    b.run(2)
    assert b.vcu.state == AS_OFF                    # no re-arm without a power cycle
    b.vcu.power_cycle()
    b.vcu.inputs.res_go = False
    assert b.arm(1)


# ---------------------------------------------------------------- 4 fault conditions in AS_DRIVING
def test_4_emulator_mission_status_fault_finished_while_moving():
    b = driving_bench()
    drive(b, 2)
    b.override = tamper("MISSION_STATUS", 3, b.cycle + 1, 0x510)
    b.run(0.05)
    assert b.vcu.state == AS_EBRAKE and b.vcu.faults.mission_status
    b.run(0.02)
    assert b.bridge.last_status["shutdown_cause"] == "MISSION_STATUS_FAULT"


def test_4_emulator_autonomous_braking_fault_neutral_while_moving():
    b = driving_bench()
    drive(b, 2)
    b.override = tamper("DIRECTION_REQUEST", 0, b.cycle + 1, 0x510)
    b.run(0.05)
    assert b.vcu.state == AS_EBRAKE and b.vcu.faults.autonomous_braking


def test_4_emulator_brake_plausibility_fault():
    b = driving_bench()
    drive(b, 1)
    start = b.cycle + 1
    b.override = lambda cyc, fid, vals: ({**vals, "HYD_PRESS_F_REQ_pct": 10.0} if fid == 0x514 and cyc >= start
                                          else None)
    drive(b, 0.1)
    assert b.vcu.state == AS_EBRAKE and b.vcu.faults.brake_plausibility


def test_4_switching_brake_to_torque_passes_through_one_empty_cycle():
    """Frames of one cycle reach the VCU one by one, drive (0x511/0x512) before brake (0x514). A direct
    brake -> torque switch would let the VCU read new torque beside the old brake. Found in a real-time run."""
    b = driving_bench()
    drive(b, 2)
    sent = []
    for k in range(60):
        cmd = DriveCommand(torque_nm=40.0, motor_speed_max_rpm=1000) if (k // 10) % 2 else DriveCommand(brake_pct=20.0)
        b.bridge.set_command(cmd, b.t)
        b.step()
        st = b.bridge.last_status
        sent.append((st["torque_cmd_nm"], st["brake_cmd_pct"]))
    for (t0, b0), (t1, b1) in zip(sent, sent[1:]):
        assert not (t1 > 0 and b0 > 0) and not (b1 > 0 and t0 > 0), sent
    assert any(t > 0 for t, _ in sent) and any(x > 0 for _, x in sent)
    assert b.vcu.state == AS_DRIVING and not b.vcu.faults.brake_plausibility


def test_4_bridge_never_breaks_a_fault_rule_under_10000_random_commands():
    """Fuzz: random speed / raw torque / raw brake / NaN commands and finish requests for 100 s."""
    rng = random.Random(MASTER_SEED)
    b = driving_bench()
    for i in range(10_000):
        kind = rng.random()
        if kind < 0.4:
            cmd = DriveCommand(speed_mps=rng.uniform(-5, 30), steer_deg=rng.uniform(-40, 40),
                               accel_mps2=rng.uniform(-2, 8))
        elif kind < 0.8:
            cmd = DriveCommand(steer_deg=rng.uniform(-40, 40), torque_nm=rng.uniform(-50, 300),
                               motor_speed_max_rpm=rng.uniform(-100, 5000), brake_pct=rng.choice([0, 0, rng.uniform(-10, 120)]))
        elif kind < 0.9:
            cmd = DriveCommand(speed_mps=float("nan"), steer_deg=float("inf"))
        else:
            cmd = None
        if cmd is not None:
            b.bridge.set_command(cmd, b.t)
        if i == 9_000:
            b.bridge.request_finish()
        b.step()
        assert b.vcu.state in (AS_DRIVING, AS_FINISHED), (i, b.vcu.events[-3:])
    assert not b.violations
    assert b.bridge.rejected_commands > 0
    assert b.vcu.state == AS_FINISHED
