# fsai-open: encode and decode ADS-DV CAN frames (Intel byte order), with no silent changes
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: turns numbers like "steer 12.3 degrees" into the 8 bytes the car reads, and back.

Rule: a value that does not fit is an error, never quietly changed. Clamping is a safety decision
and lives in the bridge (fsai/bridge/core.py), where it is logged and tested.
"""
from __future__ import annotations

from .signals import BY_ID, Message, Signal


class EncodeError(ValueError):
    pass


def _raw_range(sig: Signal) -> tuple[int, int]:
    if sig.signed:
        return -(1 << (sig.length - 1)), (1 << (sig.length - 1)) - 1
    return 0, (1 << sig.length) - 1


def to_raw(sig: Signal, value: float) -> int:
    if sig.minimum is not None and value < sig.minimum - 1e-9:
        raise EncodeError(f"{sig.name}={value} below minimum {sig.minimum}")
    if sig.maximum is not None and value > sig.maximum + 1e-9:
        raise EncodeError(f"{sig.name}={value} above maximum {sig.maximum}")
    raw = int(round((value - sig.offset) / sig.scale))
    lo, hi = _raw_range(sig)
    if not lo <= raw <= hi:
        raise EncodeError(f"{sig.name} raw {raw} outside {lo}..{hi}")
    return raw


def from_raw(sig: Signal, raw: int) -> float | int:
    value = raw * sig.scale + sig.offset
    if sig.scale == 1 and sig.offset == 0:
        return int(value)
    return round(value, 6)


def encode(msg: Message, values: dict) -> bytes:
    unknown = set(values) - {s.name for s in msg.signals}
    if unknown:
        raise EncodeError(f"{msg.name}: unknown signals {sorted(unknown)}")
    word = 0
    for sig in msg.signals:
        raw = to_raw(sig, values.get(sig.name, 0))
        if raw < 0:
            raw += 1 << sig.length
        word |= (raw & ((1 << sig.length) - 1)) << sig.start
    return word.to_bytes(8, "little")[: msg.dlc]


def decode(msg: Message, data: bytes) -> dict:
    if len(data) < msg.dlc:
        raise ValueError(f"{msg.name}: expected {msg.dlc} bytes, got {len(data)}")
    word = int.from_bytes(bytes(data[: msg.dlc]).ljust(8, b"\0"), "little")
    out = {}
    for sig in msg.signals:
        raw = (word >> sig.start) & ((1 << sig.length) - 1)
        if sig.signed and raw & (1 << (sig.length - 1)):
            raw -= 1 << sig.length
        out[sig.name] = from_raw(sig, raw)
    return out


def decode_frame(frame_id: int, data: bytes) -> tuple[str, dict] | None:
    msg = BY_ID.get(frame_id)
    if msg is None:
        return None
    return msg.name, decode(msg, data)
