# fsai-open: named, deterministic desk scenarios with a fingerprint per run (re-run must match byte for byte)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: a set of pretend test days. Each scenario is a script (arm the car, drive, cause a
problem on purpose...) with the result the spec says must happen. Each run leaves a fingerprint of
every frame and state change; running it again must give exactly the same fingerprint."""
from __future__ import annotations

import hashlib
import math

from .bridge.core import DriveCommand
from .can import signals as S
from .params import Params, load
from .vcu.lockstep import Bench

EXPECT_FINISHED = "AS_FINISHED, no faults"


def _drive(b: Bench, seconds, speed=8.0, steer=0.0):
    for _ in range(int(round(seconds / 0.01))):
        b.bridge.set_command(DriveCommand(speed_mps=speed, steer_deg=steer), b.t)
        b.step()


def _finish(b: Bench):
    b.bridge.request_finish()
    b.run(20, until=lambda x: x.vcu.state == 5)


def acceleration(b):          # AMI 1: straight 75 m
    b.arm(1); b.go()
    while b.vcu.x < 75 and b.t < 60:
        _drive(b, 0.01, speed=15)
    _finish(b)


def skidpad(b):               # AMI 2: circle of radius 9.125 m (FS skidpad centre line), steer from bicycle model
    b.arm(2); b.go()
    steer = math.degrees(math.atan(b.params.vehicle.wheelbase_m / 9.125))
    _drive(b, 20, speed=5, steer=steer)
    _finish(b)


def trackdrive_slalom(b):     # AMI 4: weaving at speed, lap counter and cone logging populated
    b.arm(4); b.go()
    for lap in range(1, 4):
        b.bridge.set_logging(lap=lap, cones_actual=20, cones_all=60 * lap)
        for k in range(500):
            _drive(b, 0.01, speed=9, steer=15 * math.sin(k / 40))
    _finish(b)


def autonomous_demo(b):       # AMI 7: scrutineering demo, short run
    b.arm(7); b.go()
    _drive(b, 5, speed=4)
    _finish(b)


def stale_team_software(b):   # team software freezes mid-run -> controlled stop, no e-stop
    b.arm(3); b.go()
    _drive(b, 4, speed=10)
    b.run(8)
    _finish(b)


def ai_stall_150ms(b):        # AI computer freezes 150 ms -> AI_COMMS_LOST e-stop (spec 2.4)
    b.arm(1); b.go()
    _drive(b, 2)
    start = b.cycle
    b.ai_stall = lambda c: start < c <= start + 15
    _drive(b, 3)


def res_estop(b):             # marshal presses the RES e-stop
    b.arm(4); b.go()
    _drive(b, 3)
    b.vcu.inputs.sdc_closed = False
    _drive(b, 4)


def team_estop(b):            # team software requests an e-stop
    b.arm(4); b.go()
    _drive(b, 3)
    b.bridge.request_estop()
    _drive(b, 4)


SCENARIOS = {
    "acceleration": (acceleration, EXPECT_FINISHED),
    "skidpad": (skidpad, EXPECT_FINISHED),
    "trackdrive_slalom": (trackdrive_slalom, EXPECT_FINISHED),
    "autonomous_demo": (autonomous_demo, EXPECT_FINISHED),
    "stale_team_software": (stale_team_software, EXPECT_FINISHED),
    "ai_stall_150ms": (ai_stall_150ms, "AS_EMERGENCY_BRAKE, cause AI_COMMS_FAULT"),
    "res_estop": (res_estop, "AS_EMERGENCY_BRAKE, stopped"),
    "team_estop": (team_estop, "AS_EMERGENCY_BRAKE, cause AI_COMPUTER_REQUEST"),
}


def run_one(name: str, params: Params | None = None) -> dict:
    fn, expected = SCENARIOS[name]
    b = Bench(params or load())
    fn(b)
    b.run(0.05)
    st = S.AS_STATE[b.vcu.state]
    cause = S.SHUTDOWN_CAUSE.get(b.vcu.shutdown_cause, "?")
    faults = [k for k, v in vars(b.vcu.faults).items() if v]
    if expected == EXPECT_FINISHED:
        observed = EXPECT_FINISHED if st == "AS_FINISHED" and not faults else f"{st}, faults {faults}"
    elif "cause" in expected:
        observed = f"{st}, cause {cause}"
    else:
        observed = f"{st}, stopped" if b.vcu.v == 0 else f"{st}, moving"
    h = hashlib.sha256()
    for row in b.trace:
        h.update(repr(row).encode())
    h.update(repr(b.vcu.events).encode())
    return {"scenario": name, "expected": expected, "observed": observed, "pass": observed == expected,
            "spec_violations_by_bridge": b.violations, "sim_seconds": b.t,
            "displacement_m": round(math.hypot(b.vcu.x, b.vcu.y), 2),
            "max_speed_mps": round(max(r[2] for r in b.trace), 2), "events": b.vcu.events,
            "trace_sha256": h.hexdigest()}


def run_all() -> list[dict]:
    return [run_one(n) for n in SCENARIOS]
