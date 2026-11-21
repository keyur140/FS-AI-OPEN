# fsai-open: regression tests for every known pitfall of the 2021 FS-AI_API and the drifting specs
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Each test first reproduces the pitfall (using a faithful port of the 2021 library, or the real
official DBC), then shows fsai-open handles it. Reproducing the bug first is the "known-answer gate":
it proves the test can see the problem, so its pass for fsai-open means something."""
import hashlib
import math
from pathlib import Path

import pytest

from fsai.bridge.core import DERATE_TABLE, BridgeCore, DriveCommand
from fsai.can import codec, signals as S
from fsai.params import load
from fsai.reference.fsai_api_2021 import Library2021
from fsai.vcu.emulator import AS_DRIVING, AS_EBRAKE, AS_FINISHED
from fsai.vcu.lockstep import Bench

from ..conftest import ROOT, dbc_2026

P = load()


def vcu_frames(angle_max=24.0, trq_max=195.0, rpm=0, as_state=3, **status):
    st = {"AS_STATE": as_state, "AMI_STATE": 1, "HANDSHAKE": 1, **status}
    return [
        (0x520, codec.encode(S.VCU2AI_STATUS, st)),
        (0x521, codec.encode(S.VCU2AI_DRIVE_F, {"FRONT_AXLE_TRQ_MAX": trq_max})),
        (0x522, codec.encode(S.VCU2AI_DRIVE_R, {"REAR_AXLE_TRQ_MAX": trq_max})),
        (0x523, codec.encode(S.VCU2AI_STEER, {"ANGLE_MAX": angle_max})),
        (0x525, codec.encode(S.VCU2AI_SPEEDS, {k: rpm for k in ("FL_WHEEL_SPEED", "FR_WHEEL_SPEED",
                                                                 "RL_WHEEL_SPEED", "RR_WHEEL_SPEED")})),
    ]


def feed(core, frames, now=0.0):
    for fid, data in frames:
        core.on_frame(fid, data, now)


def sent(frames):
    return {fid: codec.decode(S.BY_ID[fid], d) for fid, d in frames}


# ---------------------------------------------------------------- P1 set-before-get silently zeroes commands
def test_p1_2021_library_silently_zeroes_commands_before_first_get():
    lib = Library2021()
    out = lib.set_data(steer_deg=10, axle_speed_rpm=500, axle_torque_nm=100, brake_pct=0)
    assert out["steer"] == 0 and out["rear_trq"] == 0          # reproduced: "car won't steer or drive"


def test_p1_fsai_open_never_zeroes_silently():
    core = BridgeCore(P)
    feed(core, [(0x520, codec.encode(S.VCU2AI_STATUS, {"AS_STATE": 3, "AMI_STATE": 1}))])
    core.set_command(DriveCommand(steer_deg=10, torque_nm=100, motor_speed_max_rpm=1000), 0.0)
    core.step(0.0)
    notes = " ".join(core.last_status["notes"])
    assert "steer limit not yet received" in notes and "torque limit not yet received" in notes
    assert core.last_status["angle_max_deg"] is None and core.last_status["torque_cap_nm"] is None
    feed(core, vcu_frames())                                    # limits arrive -> commands flow
    core.set_command(DriveCommand(steer_deg=10, torque_nm=100, motor_speed_max_rpm=1000), 0.01)
    f = sent(core.step(0.01))
    assert f[0x513]["STEER_REQUEST"] == 10.0 and f[0x512]["REAR_AXLE_TRQ_REQUEST"] == 100.0


# ---------------------------------------------------------------- P2 signed 1-bit flags in the 2026 DBC
def test_p2_official_2026_dbc_decodes_set_flags_as_minus_one():
    db = dbc_2026()
    data = codec.encode(S.VCU2AI_STATUS, {"HANDSHAKE": 1, "GO_SIGNAL": 1, "AI_COMMS_LOST": 1, "AS_STATE": 3})
    theirs = db.get_message_by_frame_id(0x520).decode(data, decode_choices=False)
    assert theirs["HANDSHAKE"] == -1 and theirs["GO_SIGNAL"] == -1 and theirs["AI_COMMS_LOST"] == -1
    ours = codec.decode(S.VCU2AI_STATUS, data)
    assert ours["HANDSHAKE"] == 1 and ours["GO_SIGNAL"] == 1 and ours["AI_COMMS_LOST"] == 1


# ---------------------------------------------------------------- P3 steering limit differs in every document
@pytest.mark.parametrize("angle_max", [21.0, 24.0, 27.2, 30.0])
def test_p3_steer_limit_is_taken_live_from_vcu(angle_max):
    core = BridgeCore(P)
    feed(core, vcu_frames(angle_max=angle_max))
    for want in (35.0, -35.0):
        core.set_command(DriveCommand(steer_deg=want, torque_nm=0), 0.0)
        f = sent(core.step(0.0))
        assert f[0x513]["STEER_REQUEST"] == math.copysign(angle_max, want)
    b = Bench(load(vcu__angle_max_deg=angle_max))
    assert b.arm(1) and b.go()
    for _ in range(300):
        b.bridge.set_command(DriveCommand(speed_mps=3, steer_deg=40), b.t)
        b.step()
    assert abs(b.vcu.steer - angle_max) < 0.11 and b.vcu.state == AS_DRIVING


# ---------------------------------------------------------------- P4 hidden torque derate
@pytest.mark.parametrize("rpm, cap", [(0, 195.0), (300, 195.0), (301, 150.0), (401, 120.0), (501, 100.0),
                                      (601, 85.0), (701, 50.0), (1000, 50.0)])
def test_p4_torque_derate_is_explicit_and_matches_2021_table(rpm, cap):
    lib = Library2021()
    lib.get_data(195, 195, 24, (rpm, 0, 0, 0))
    assert lib.set_data(0, 100, 195, 0)["rear_trq"] == cap
    core = BridgeCore(P)
    feed(core, vcu_frames(rpm=rpm))
    core.set_command(DriveCommand(torque_nm=195, motor_speed_max_rpm=4000), 0.0)
    f = sent(core.step(0.0))
    assert f[0x512]["REAR_AXLE_TRQ_REQUEST"] == cap
    if cap < 195:
        assert any("derated" in n for n in core.last_status["notes"])
    assert DERATE_TABLE[0] == (700, 50.0)


def test_p4_derate_can_be_switched_off_by_the_team():
    core = BridgeCore(load(bridge__torque_power_derate=False))
    feed(core, vcu_frames(rpm=800))
    core.set_command(DriveCommand(torque_nm=195, motor_speed_max_rpm=4000), 0.0)
    assert sent(core.step(0.0))[0x512]["REAR_AXLE_TRQ_REQUEST"] == 195.0


# ---------------------------------------------------------------- P5 one value for both axles, brake zeroes torque silently
def test_p5_2021_library_forces_identical_axles_and_hides_the_brake_override():
    lib = Library2021()
    lib.get_data(195, 195, 24)
    out = lib.set_data(0, 100, 50, brake_pct=5)
    assert out["front_brake"] == out["rear_brake"] and out["rear_trq"] == 0   # merged, no message


def test_p5_fsai_open_reports_interlock_and_respects_driven_axles():
    core = BridgeCore(P)                                         # 2026 spec: rear-wheel drive
    feed(core, vcu_frames())
    core.set_command(DriveCommand(torque_nm=50, motor_speed_max_rpm=1000, brake_pct=5), 0.0)
    f = sent(core.step(0.0))
    assert f[0x512]["REAR_AXLE_TRQ_REQUEST"] == 0 and f[0x514]["HYD_PRESS_R_REQ_pct"] == 5.0
    assert any("brake plausibility" in n for n in core.last_status["notes"])
    core.set_command(DriveCommand(torque_nm=50, motor_speed_max_rpm=1000), 0.01)
    f = sent(core.step(0.01))                                    # frame-order guard: one empty cycle
    assert f[0x512]["REAR_AXLE_TRQ_REQUEST"] == 0 and f[0x514]["HYD_PRESS_R_REQ_pct"] == 0
    assert any("frame-order guard" in n for n in core.last_status["notes"])
    core.set_command(DriveCommand(torque_nm=50, motor_speed_max_rpm=1000), 0.02)
    f = sent(core.step(0.02))
    assert f[0x511]["FRONT_AXLE_TRQ_REQUEST"] == 0 and f[0x512]["REAR_AXLE_TRQ_REQUEST"] == 50.0
    both = BridgeCore(load(vehicle__driven_axles=("front", "rear")))
    feed(both, vcu_frames())
    both.set_command(DriveCommand(torque_nm=50, motor_speed_max_rpm=1000), 0.0)
    f = sent(both.step(0.0))
    assert f[0x511]["FRONT_AXLE_TRQ_REQUEST"] == 50.0 == f[0x512]["REAR_AXLE_TRQ_REQUEST"]


# ---------------------------------------------------------------- P6 diagnostics thrown away
@pytest.mark.parametrize("flag, cause, cause_name", [
    ("HVIL_OPEN_FAULT", 2, "HVIL_OPEN_FAULT"), ("EBS_FAULT", 4, "EBS_FAULT"),
    ("AI_COMMS_LOST", 6, "AI_COMMS_FAULT"), ("BRAKE_PLAUSIBILITY_FAULT", 11, "BRAKE_PLAUSIBILITY_FAULT"),
    ("BMS_FAULT", 10, "BMS_FAULT"), ("MISSION_STATUS_FAULT", 8, "MISSION_STATUS_FAULT")])
def test_p6_every_fault_bit_and_shutdown_cause_is_exposed(flag, cause, cause_name):
    core = BridgeCore(P)
    feed(core, vcu_frames(as_state=4, **{flag: 1, "SHUTDOWN_CAUSE": cause, "FAULT_STATUS": 1}))
    core.step(0.0)
    assert core.last_status["faults"] == [flag]
    assert core.last_status["shutdown_cause"] == cause_name
    assert core.last_status["as_state"] == "AS_EMERGENCY_BRAKE"


# ---------------------------------------------------------------- P7 FSG logging fields hard-coded to zero
def test_p7_fsg_logging_fields_are_populated():
    b = Bench(P)
    assert b.arm(4) and b.go()
    b.bridge.set_logging(lap=3, cones_actual=17, cones_all=402)
    for _ in range(300):
        b.bridge.set_command(DriveCommand(speed_mps=8), b.t)
        b.step()
    f = sent(b.bridge.step(b.t))[0x510]
    assert (f["LAP_COUNTER"], f["CONES_COUNT_ACTUAL"], f["CONES_COUNT_ALL"]) == (3, 17, 402)
    assert f["VEH_SPEED_ACTUAL"] > 0 and f["VEH_SPEED_DEMAND"] == round(8 * 3.6)


# ---------------------------------------------------------------- P8 stale DBC filename / unpinned files
def test_p8_dbc_is_pinned_by_commit_and_hash_and_uses_the_2026_name():
    src = (ROOT / "tools" / "fetch_dbc.py").read_text()
    assert "ADS-DV_2026_CAN1.dbc" in src and "ADS-DV_2025_CAN1.dbc" not in src
    assert "bb8f1624936f13c10a3406ea060088f2ffb4b908" in src
    path = ROOT / "third_party" / "dbc" / "ADS-DV_2026_CAN1.dbc"
    if path.exists():
        assert hashlib.sha256(path.read_bytes()).hexdigest() == \
            "fdc895edde31f2b8f3c8638e3d341e05e231eb434e36a9311d65a4f243cd4a0a"


# ---------------------------------------------------------------- P9 HiL brake quirk (documented divergence)
def test_p9_hil_brake_quirk_is_modelled_but_off_by_default():
    for quirk, expect in ((False, AS_FINISHED), (True, AS_EBRAKE)):
        b = Bench(load(vcu__hil_brake_estop_quirk=quirk))
        assert b.arm(1) and b.go()
        for _ in range(200):
            b.bridge.set_command(DriveCommand(speed_mps=5), b.t)
            b.step()
        b.bridge.request_finish()
        b.run(10, until=lambda x: x.vcu.state in (AS_FINISHED, AS_EBRAKE))
        assert b.vcu.state == expect


# ---------------------------------------------------------------- P10 the 8 ms send gate / realtime clock
def test_p10_runner_uses_monotonic_clock_and_has_no_send_gate():
    src = (ROOT / "fsai" / "bridge" / "runner.py").read_text()
    assert "time.monotonic()" in src and "CLOCK_REALTIME" not in src.split('"""', 2)[2]
    core_src = (ROOT / "fsai" / "bridge" / "core.py").read_text()
    assert "time." not in core_src.replace("timeout", "")      # bridge logic has no clock of its own


# ---------------------------------------------------------------- stale or invalid team commands
def test_stale_command_brakes_to_a_stop_without_estop():
    b = Bench(P)
    assert b.arm(1) and b.go()
    for _ in range(300):
        b.bridge.set_command(DriveCommand(speed_mps=8), b.t)
        b.step()
    assert b.vcu.v > 4
    b.run(6)                                                    # team software freezes
    assert b.vcu.state == AS_DRIVING and b.vcu.v == 0 and not b.violations
    assert b.bridge.last_status["command_stale"]


def test_nan_and_infinite_commands_are_rejected():
    core = BridgeCore(P)
    assert not core.set_command(DriveCommand(speed_mps=float("nan")), 0.0)
    assert not core.set_command(DriveCommand(steer_deg=float("-inf")), 0.0)
    assert core.rejected_commands == 2 and core.cmd is None
