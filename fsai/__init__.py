# fsai-open: package metadata shared by every module, report and run manifest
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""fsai-open: an open, tested software layer for the IMechE FS-AI ADS-DV.

Plain words: this package lets a team's driving software talk to the ADS-DV car safely,
and lets anyone test that on a desk with a software copy of the car's safety system.
"""

import os as _os

# One maths thread: results are then the same on every machine and run (multi-threaded linear
# algebra can add numbers in a different order). Set before numpy is first imported.
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    _os.environ.setdefault(_var, "1")

__version__ = "0.2.0"

AUTHOR = {
    "name": "KEYUR SHAILESHKUMAR PATEL",
    "given_names": "KEYUR SHAILESHKUMAR",
    "family_names": "PATEL",
    "orcid": "https://orcid.org/0009-0008-5261-043X",
    "affiliations": [
        "MSc Artificial Intelligence student, Department of Computer Science, Brunel University London, UK",
        "Independent researcher",
    ],
    "note": "Conducted independently; not an official Brunel University London project.",
}

AI_ASSISTANCE = (
    "Code and documents were drafted with an AI assistant (Claude, by Anthropic). "
    "Every design decision was made and approved by the author; every result comes from "
    "deterministic code and tests that anyone can re-run."
)

MASTER_SEED = 20261117
