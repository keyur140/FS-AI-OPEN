# fsai-open: known-answer gates, determinism, and "test the tests" (a broken bridge must be caught)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
import hashlib

import pytest

from fsai.bridge import core as core_mod
from fsai.bridge.core import BridgeCore, DriveCommand
from fsai.can import codec, signals as S
from fsai.params import load
from fsai.vcu.lockstep import Bench

P = load()


# ---------------------------------------------------------------- hand-computed bytes (worked out on paper)
@pytest.mark.parametrize("msg, values, expected_hex", [
    (S.AI2VCU_STEER, {"STEER_REQUEST": 12.3}, "7b00"),          # 123 = 0x007B, little-endian
    (S.AI2VCU_STEER, {"STEER_REQUEST": -12.3}, "85ff"),         # -123 = 0xFF85 two's complement
    (S.AI2VCU_DRIVE_R, {"REAR_AXLE_TRQ_REQUEST": 195.0, "REAR_MOTOR_SPEED_MAX": 4000}, "9e07a00f"),
    (S.AI2VCU_BRAKE, {"HYD_PRESS_F_REQ_pct": 100.0, "HYD_PRESS_R_REQ_pct": 0.5}, "c801"),
    (S.AI2VCU_STATUS, {"HANDSHAKE": 1, "ESTOP_REQUEST": 0, "MISSION_STATUS": 2, "DIRECTION_REQUEST": 1,
                       "LAP_COUNTER": 3}, "01600300" + "00000000"),  # byte1 = 0b0110_0000
])
def test_known_answer_encoding(msg, values, expected_hex):
    data = codec.encode(msg, values)
    assert data.hex() == expected_hex
    back = codec.decode(msg, data)
    for k, v in values.items():
        assert back[k] == pytest.approx(v)


def test_out_of_range_values_raise_instead_of_changing():
    with pytest.raises(codec.EncodeError):
        codec.encode(S.AI2VCU_STEER, {"STEER_REQUEST": 30.1})
    with pytest.raises(codec.EncodeError):
        codec.encode(S.AI2VCU_BRAKE, {"HYD_PRESS_F_REQ_pct": -1})
    with pytest.raises(codec.EncodeError):
        codec.encode(S.AI2VCU_STEER, {"NOT_A_SIGNAL": 1})


# ---------------------------------------------------------------- determinism: byte-identical runs
def _mission_digest() -> str:
    b = Bench(P)
    b.arm(4)
    b.go()
    h = hashlib.sha256()
    for i in range(3000):
        b.bridge.set_command(DriveCommand(speed_mps=6 + (i % 300) / 100, steer_deg=((i % 400) - 200) / 20), b.t)
        b.step()
        h.update(repr(b.trace[-1]).encode())
    b.bridge.request_finish()
    b.run(10, until=lambda x: x.vcu.state == 5)
    h.update(repr(b.vcu.events).encode())
    return h.hexdigest()


def test_two_runs_are_byte_identical():
    assert _mission_digest() == _mission_digest()


# ---------------------------------------------------------------- test the tests
class NaiveBridge(BridgeCore):
    """A bridge with the safety interlocks removed, to prove the observers would catch one."""

    def step(self, now):
        frames = super().step(now)
        if self.as_state == 3 and self.cmd is not None and (self.cmd.brake_pct or 0) > 0:
            frames = [(fid, d) for fid, d in frames if fid not in (0x511, 0x512)]
            frames += [(0x511, codec.encode(S.AI2VCU_DRIVE_F, {"FRONT_AXLE_TRQ_REQUEST": 0,
                                                               "FRONT_MOTOR_SPEED_MAX": 0})),
                       (0x512, codec.encode(S.AI2VCU_DRIVE_R, {"REAR_AXLE_TRQ_REQUEST": 20.0,
                                                               "REAR_MOTOR_SPEED_MAX": 1000}))]
        return frames


def test_observer_and_emulator_catch_a_bridge_without_interlocks():
    b = Bench(P)
    b.bridge = NaiveBridge(P)
    assert b.arm(1) and b.go()
    for _ in range(100):
        b.bridge.set_command(DriveCommand(torque_nm=20, motor_speed_max_rpm=1000, brake_pct=10), b.t)
        b.step()
    assert any("torque and brake" in v for v in b.violations)
    assert b.vcu.state == 4 and b.vcu.faults.brake_plausibility


def test_observer_catches_finished_while_moving(monkeypatch):
    b = Bench(P)
    assert b.arm(1) and b.go()
    for _ in range(300):
        b.bridge.set_command(DriveCommand(speed_mps=8), b.t)
        b.step()
    monkeypatch.setattr(b.bridge, "_at_standstill", lambda: True)     # a bridge that lies about speed
    b.bridge.request_finish()
    b.run(0.1)
    assert any("FINISHED while moving" in v for v in b.violations)
    assert b.vcu.state == 4 and b.vcu.faults.mission_status
