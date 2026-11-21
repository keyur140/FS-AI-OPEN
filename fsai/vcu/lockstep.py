# fsai-open: deterministic desk test bench - bridge and VCU emulator exchanging real CAN bytes in simulated time
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: plugs the translator into the pretend car and runs them together, 10 ms at a time.

Both sides only ever see encoded CAN bytes, exactly as on the real bus. Faults can be injected:
the AI can stall (skip cycles), frames can be dropped, and a "bad AI" can override frames to prove
that the emulator catches every rule break.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from ..bridge.core import BridgeCore
from ..can import codec
from ..can import signals as S
from ..params import Params
from .emulator import VcuEmulator


@dataclass
class Bench:
    params: Params = field(default_factory=Params)
    ai_stall: Callable[[int], bool] = lambda cycle: False            # True = AI sends nothing this cycle
    drop: Callable[[int, int], bool] = lambda cycle, fid: False       # True = this AI frame is lost
    override: Callable[[int, int, dict], dict | None] | None = None   # tamper with AI frames (bad AI)
    trace: list = field(default_factory=list)

    def __post_init__(self):
        self.vcu = VcuEmulator(self.params)
        self.bridge = BridgeCore(self.params)
        self.cycle = 0
        self.sent_ids: set[int] = set()
        self.violations: list[str] = []

    @property
    def t(self) -> float:
        return round(self.cycle * 0.01, 6)

    def step(self):
        self.cycle += 1
        for fid, data in self.vcu.tick():
            self.bridge.on_frame(fid, data, self.t)
        if self.ai_stall(self.cycle):
            return
        sent = {}
        for fid, data in self.bridge.step(self.t):
            self.sent_ids.add(fid)
            msg = S.BY_ID[fid]
            if self.override is not None:
                vals = self.override(self.cycle, fid, codec.decode(msg, data))
                if vals is not None:
                    data = codec.encode(msg, vals)
            sent[fid] = codec.decode(msg, data)
            if not self.drop(self.cycle, fid):
                self.vcu.receive(fid, data)
        self._check(sent)
        self.trace.append((self.t, self.vcu.state, round(self.vcu.v, 3)))

    def _check(self, f: dict):
        """Independent observer: records any cycle whose frames would break a spec rule."""
        trq = max(f[0x511]["FRONT_AXLE_TRQ_REQUEST"], f[0x512]["REAR_AXLE_TRQ_REQUEST"])
        brk = max(f[0x514]["HYD_PRESS_F_REQ_pct"], f[0x514]["HYD_PRESS_R_REQ_pct"])
        if trq > 0 and brk > 0:
            self.violations.append(f"{self.t}: torque and brake in the same cycle")
        moving = self.vcu.wheel_rpm > self.params.vcu.standstill_rpm
        if moving and self.vcu.state == 3:
            if f[0x510]["DIRECTION_REQUEST"] == 0:
                self.violations.append(f"{self.t}: NEUTRAL while moving")
            if f[0x510]["MISSION_STATUS"] == 3:
                self.violations.append(f"{self.t}: FINISHED while moving")
        if self.vcu.state == 2 and (trq > 0 or f[0x513]["STEER_REQUEST"] != 0 or f[0x510]["DIRECTION_REQUEST"] != 0):
            self.violations.append(f"{self.t}: non-zero request in AS_READY")

    def run(self, seconds: float, until: Callable[["Bench"], bool] | None = None):
        n = int(round(seconds / 0.01))
        for _ in range(n):
            self.step()
            if until is not None and until(self):
                return True
        return False

    # ------------------------------------------------------------------ the usual way to get to DRIVING
    def arm(self, mission: int = 1):
        """Driver's checklist: power, ASMS, TSMS, pick mission. Returns when AS_READY (or fails)."""
        i = self.vcu.inputs
        i.asms = True
        i.tsms = True
        i.ami = mission
        return self.run(2.0, until=lambda b: b.vcu.state == 2)

    def go(self):
        """Wait the 5 s in AS_READY, then flick the RES Go switch (rising edge)."""
        self.run(self.params.vcu.ready_min_s + 0.1)
        self.vcu.inputs.res_go = True
        return self.run(0.5, until=lambda b: b.vcu.state == 3)
