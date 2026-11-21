#!/usr/bin/env python3
# fsai-open: fetch the official 2026 ADS-DV CAN database at a pinned commit and check its SHA-256
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""The 2026 DBC lives in FS-AI/FS-AI_ADS-DV_Documentation, which carries no open licence, so it is
not redistributed here. This script downloads exactly the pinned version and refuses any other.

Usage: python tools/fetch_dbc.py            (exit 0 = file present and verified)
"""
import hashlib
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMIT = "bb8f1624936f13c10a3406ea060088f2ffb4b908"
SHA256 = "fdc895edde31f2b8f3c8638e3d341e05e231eb434e36a9311d65a4f243cd4a0a"
URL = f"https://raw.githubusercontent.com/FS-AI/FS-AI_ADS-DV_Documentation/{COMMIT}/ADS-DV_2026_CAN1.dbc"
DEST = ROOT / "third_party" / "dbc" / "ADS-DV_2026_CAN1.dbc"


def main() -> int:
    if DEST.exists() and hashlib.sha256(DEST.read_bytes()).hexdigest() == SHA256:
        print(f"OK  {DEST.relative_to(ROOT)} already present, SHA-256 verified")
        return 0
    print(f"Downloading {URL}")
    try:
        data = urllib.request.urlopen(URL, timeout=60).read()
    except Exception as exc:  # network is optional: tests that need the file will say so
        print(f"WARNING could not download the 2026 DBC ({exc}). DBC cross-check tests will be skipped.")
        return 1
    got = hashlib.sha256(data).hexdigest()
    if got != SHA256:
        print(f"REFUSED: SHA-256 {got} does not match pinned {SHA256}")
        return 2
    DEST.parent.mkdir(parents=True, exist_ok=True)
    DEST.write_bytes(data)
    print(f"OK  saved {DEST.relative_to(ROOT)}, SHA-256 verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
