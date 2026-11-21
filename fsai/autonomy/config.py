# fsai-open: load the driving-stack settings (config/autonomy.yaml) with optional overrides
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
from __future__ import annotations

import copy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
AUTONOMY_FILE = ROOT / "config" / "autonomy.yaml"


class Cfg(dict):
    """A dict whose sections can also be read as attributes: cfg.speed["max_mps"] or cfg["speed"]["max_mps"]."""
    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name) from None       # so copy and pickle (multiprocessing) work

    def __reduce__(self):
        return (Cfg, (dict(self),))


def load(path: Path | str = AUTONOMY_FILE, **overrides) -> Cfg:
    """overrides use section__key, e.g. load(speed__max_mps=12)."""
    import yaml
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    cfg = Cfg({sec: {k: (v["value"] if isinstance(v, dict) else v) for k, v in body.items()}
               for sec, body in raw.items()})
    for key, value in overrides.items():
        sec, name = key.split("__", 1)
        if name not in cfg[sec]:
            raise KeyError(f"unknown setting {sec}.{name}")
        cfg[sec][name] = value
    return cfg


def with_overrides(cfg: Cfg, **overrides) -> Cfg:
    out = Cfg(copy.deepcopy(dict(cfg)))
    for key, value in overrides.items():
        sec, name = key.split("__", 1)
        out[sec][name] = value
    return out
