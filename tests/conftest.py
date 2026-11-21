# fsai-open: shared test fixtures (official DBC loaders, skipped with a clear reason if absent)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
from functools import lru_cache
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
DBC_DIR = ROOT / "third_party" / "dbc"


@lru_cache(maxsize=None)
def _load(name):
    cantools = pytest.importorskip("cantools")
    path = DBC_DIR / name
    if not path.exists():
        pytest.skip(f"{name} not present; run python tools/fetch_dbc.py (needs internet once)")
    return cantools.database.load_file(str(path), strict=False)


def dbc_2021():
    return _load("ADSDV_2021_VCU_AI_interface_v2.dbc")


def dbc_2026():
    return _load("ADS-DV_2026_CAN1.dbc")
