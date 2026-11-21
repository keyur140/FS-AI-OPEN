# fsai-open: independent recomputation - our codec versus the third-party cantools decoder on the official DBCs
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
import random

import pytest

from fsai import MASTER_SEED
from fsai.can import codec, signals as S

from ..conftest import dbc_2021, dbc_2026

# Messages that exist in the 2021 v2 DBC, and the 2026-only signals to ignore when comparing to it.
ONLY_2026 = {"OP_STATE", "TS_STATE", "EBS_STATE"}
# Signal renames between our table (spec v4.0 wording) and each DBC.
RENAMES_2026 = {("VCU2AI_Drive_R", "REAR_AXLE_TRQ"): "REAR_AXLE_TRQ_Nm",
                ("VCU2AI_Drive_R", "REAR_AXLE_TRQ_REQUEST"): "REAR_AXLE_TRQ_REQUEST_Nm",
                ("VCU2AI_Drive_R", "REAR_AXLE_TRQ_MAX"): "REAR_AXLE_TRQ_MAX_Nm"}


def _dbc_signal(db, msg, sig, renames):
    m = db.get_message_by_frame_id(msg.frame_id)
    name = renames.get((msg.name, sig.name), sig.name)
    found = [s for s in m.signals if s.name == name]
    return m, (found[0] if found else None)


@pytest.mark.parametrize("which", ["2021", "2026"])
def test_layout_matches_official_dbc(which):
    """Every signal: same start bit, length, scale, offset and DLC as the official file."""
    db = dbc_2021() if which == "2021" else dbc_2026()
    renames = RENAMES_2026 if which == "2026" else {}
    checked = 0
    for msg in S.AI_TX_WATCHED + S.VCU_TX:
        m = db.get_message_by_frame_id(msg.frame_id)
        assert m.length == msg.dlc, f"{msg.name} DLC"
        for sig in msg.signals:
            if which == "2021" and sig.name in ONLY_2026:
                continue
            _, ds = _dbc_signal(db, msg, sig, renames)
            assert ds is not None, f"{which}: {msg.name}.{sig.name} missing from DBC"
            assert ds.byte_order == "little_endian", sig.name
            assert (ds.start, ds.length) == (sig.start, sig.length), f"{msg.name}.{sig.name}"
            assert ds.scale == pytest.approx(sig.scale) and ds.offset == pytest.approx(sig.offset), sig.name
            if sig.length > 1:  # 1-bit flags are a documented deliberate difference (see test_pitfalls)
                assert ds.is_signed == sig.signed, f"{msg.name}.{sig.name} signedness"
            checked += 1
    assert checked > 60


def test_random_round_trip_agrees_with_cantools():
    """10,000 seeded random frames: our decode equals cantools decode on every multi-bit signal."""
    db = dbc_2026()
    rng = random.Random(MASTER_SEED)
    for _ in range(10_000):
        msg = rng.choice(S.AI_TX_WATCHED + S.VCU_TX)
        data = bytes(rng.randrange(256) for _ in range(msg.dlc))
        ours = codec.decode(msg, data)
        theirs = db.get_message_by_frame_id(msg.frame_id).decode(data, decode_choices=False, scaling=True)
        for sig in msg.signals:
            _, ds = _dbc_signal(db, msg, sig, RENAMES_2026)
            t = theirs[ds.name]
            if sig.length == 1:
                assert abs(t) == ours[sig.name]          # DBC says -1 where the spec means 1
            else:
                assert t == pytest.approx(ours[sig.name], abs=1e-6), (msg.name, sig.name)
