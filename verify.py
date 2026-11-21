#!/usr/bin/env python3
# fsai-open: verify every file fingerprint and re-run every scenario to prove byte-identical results
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""python verify.py -> exit 0 only if (1) every file matches SHA256SUMS.txt, (2) the Merkle root matches
MERKLE_ROOT.txt, (3) re-running every stage 1 scenario gives exactly the recorded fingerprints, and
(4) re-running the short stage 2 events (acceleration, skidpad, autocross, seed 0) gives the recorded fingerprints."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from fsai.provenance import merkle_root, sha256_file  # noqa: E402


def main():
    ok = True
    entries = [l.split("  ", 1) for l in (ROOT / "SHA256SUMS.txt").read_text(encoding="utf-8").splitlines() if l.strip()]
    bad = [p for h, p in entries if not (ROOT / p).exists() or sha256_file(ROOT / p) != h]
    print(f"[1] File fingerprints: {len(entries) - len(bad)}/{len(entries)} match")
    for p in bad[:20]:
        print("    CHANGED OR MISSING:", p)
    ok &= not bad
    root = merkle_root([h for h, _ in entries])
    expected = (ROOT / "MERKLE_ROOT.txt").read_text().strip()
    print(f"[2] Merkle root {'matches' if root == expected else 'DOES NOT match'}: {root}")
    ok &= root == expected
    from fsai.scenarios import run_one
    recorded = json.loads((ROOT / "runs" / "scenarios" / "results.json").read_text())["scenarios"]
    same = 0
    for r in recorded:
        again = run_one(r["scenario"])
        if again["trace_sha256"] == r["trace_sha256"]:
            same += 1
        else:
            print("    DIFFERENT:", r["scenario"])
    print(f"[3] Re-run scenarios byte-identical: {same}/{len(recorded)}")
    ok &= same == len(recorded)
    ev_file = ROOT / "runs" / "events" / "results.json"
    if ev_file.exists():
        from fsai.autonomy.sim import World
        runs = json.loads(ev_file.read_text())["runs"]
        pick = [r for r in runs if r["seed"] == 0 and r["event"] in ("acceleration", "skidpad", "autocross")]
        same2 = 0
        for r in pick:
            again = World(r["event"], r["seed"]).run(900)
            if again["trace_sha256"] == r["trace_sha256"]:
                same2 += 1
            else:
                print("    DIFFERENT:", r["event"], r["seed"])
        print(f"[4] Re-run driverless events byte-identical: {same2}/{len(pick)}")
        ok &= same2 == len(pick)
    print("VERIFIED" if ok else "VERIFICATION FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
