# fsai-open: real-time 10 ms loops that connect the bridge (or the VCU emulator) to a CAN bus
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: the clock. Every 10 ms it reads what the car said, asks the bridge what to say, and
sends it. It uses a monotonic clock (immune to the computer's time being changed, unlike the 2021
library's CLOCK_REALTIME) and schedules against fixed deadlines so it never drifts.

Bus choices (one line to switch, see docs/TEAM_SETUP_GUIDE.md):
  sim   -> python-can "udp_multicast": works across processes and inside Docker on any OS, no kernel module
  vcan0 -> Linux virtual CAN (socketcan), the classic desk set-up
  can0  -> the real ADS-DV CAN bus through a SocketCAN adapter (IMechE tests with PEAK)
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

BUS_PRESETS = {
    "sim": {"interface": "udp_multicast", "channel": "239.74.163.2"},
    "vcan0": {"interface": "socketcan", "channel": "vcan0", "bitrate": None},
    "can0": {"interface": "socketcan", "channel": "can0", "bitrate": None},
}


def open_bus(name: str, receive_own: bool = False):
    import can
    if name in BUS_PRESETS:
        cfg = {k: v for k, v in BUS_PRESETS[name].items() if v is not None}
    elif ":" in name:                                  # e.g. "socketcan:can1" or "pcan:PCAN_USBBUS1"
        iface, ch = name.split(":", 1)
        cfg = {"interface": iface, "channel": ch}
        if iface == "pcan":
            cfg["bitrate"] = 500_000
    else:
        cfg = {"interface": "socketcan", "channel": name}
    if cfg["interface"] == "udp_multicast":
        cfg["receive_own_messages"] = receive_own
    return can.Bus(**cfg)


@dataclass
class LoopStats:
    cycles: int = 0
    overruns: int = 0                 # cycles that started more than one period late
    max_period_ms: float = 0.0
    periods_ms: list = field(default_factory=list)

    def record(self, period_ms: float, keep: int = 100_000):
        self.cycles += 1
        self.max_period_ms = max(self.max_period_ms, period_ms)
        if len(self.periods_ms) < keep:
            self.periods_ms.append(period_ms)

    def summary(self) -> dict:
        p = sorted(self.periods_ms) or [0.0]
        return {"cycles": self.cycles, "overruns": self.overruns, "max_period_ms": round(self.max_period_ms, 3),
                "p50_period_ms": round(p[len(p) // 2], 3), "p99_period_ms": round(p[int(len(p) * 0.99) - 1], 3)}


def periodic(step, stop: threading.Event, period: float = 0.01, stats: LoopStats | None = None):
    """Call step(now) every `period` seconds against fixed monotonic deadlines."""
    stats = stats or LoopStats()
    next_t = time.monotonic()
    last = None
    while not stop.is_set():
        now = time.monotonic()
        if last is not None:
            stats.record((now - last) * 1000)
        last = now
        step(now)
        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        elif delay < -period:                         # fell behind by more than a cycle: resync, count it
            stats.overruns += 1
            next_t = time.monotonic()
    return stats


def drain(bus, handler, now):
    while True:
        msg = bus.recv(timeout=0)
        if msg is None:
            return
        if not msg.is_extended_id and not msg.is_error_frame:
            handler(msg.arbitration_id, bytes(msg.data), now)


def send_frames(bus, frames):
    import can
    for fid, data in frames:
        bus.send(can.Message(arbitration_id=fid, data=data, is_extended_id=False))


def bridge_step_fn(core, bus, on_status=None):
    def step(now):
        drain(bus, core.on_frame, now)
        send_frames(bus, core.step(now))
        if on_status is not None:
            on_status(core.last_status)
    return step


def emulator_step_fn(vcu, bus):
    def step(now):
        drain(bus, lambda fid, data, _now: vcu.receive(fid, data), now)
        send_frames(bus, vcu.tick())
    return step
