# fsai-open: provenance - SHA-256 fingerprints, Merkle root and run manifests
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
# Adapted from the same author's Vidya Open (vidya/provenance.py, build v0.1.0+ddcb4d5e90eb).
"""Plain words: every file gets a fingerprint. Change one character anywhere and the fingerprint
changes, so anyone can prove nothing was altered after the results were written."""
from __future__ import annotations

import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from . import AI_ASSISTANCE, AUTHOR, __version__

ROOT = Path(__file__).resolve().parent.parent
EXCLUDE_NAMES = {"SHA256SUMS.txt", "MERKLE_ROOT.txt", "ADS-DV_2026_CAN1.dbc"}   # last: not redistributed
EXCLUDE_DIRS = {"local_runs", ".git", "__pycache__", ".pytest_cache", "build", "install", "log", "dist"}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_json(obj) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def write_json(path: Path, obj) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = (json.dumps(obj, sort_keys=True, indent=1, ensure_ascii=False, default=str) + "\n").encode("utf-8")
    path.write_bytes(data)
    return sha256_bytes(data)


def merkle_root(hashes: list[str]) -> str:
    """Binary Merkle tree over hex digests (caller sorts by path). An odd leaf is duplicated."""
    if not hashes:
        return sha256_bytes(b"")
    level = [bytes.fromhex(h) for h in hashes]
    while len(level) > 1:
        if len(level) % 2:
            level.append(level[-1])
        level = [hashlib.sha256(level[i] + level[i + 1]).digest() for i in range(0, len(level), 2)]
    return level[0].hex()


def tracked_files(root: Path = ROOT) -> list[Path]:
    out = []
    for p in sorted(root.rglob("*")):
        if not p.is_file():
            continue
        rel = p.relative_to(root)
        if any(part in EXCLUDE_DIRS for part in rel.parts) or p.name in EXCLUDE_NAMES or p.suffix in (".zip", ".pyc"):
            continue
        out.append(p)
    return out


def build_manifest(root: Path = ROOT) -> tuple[list[tuple[str, str]], str]:
    entries = sorted(((sha256_file(p), p.relative_to(root).as_posix()) for p in tracked_files(root)),
                     key=lambda e: e[1])
    return entries, merkle_root([h for h, _ in entries])


def write_sha256sums(root: Path = ROOT) -> str:
    entries, root_hash = build_manifest(root)
    (root / "SHA256SUMS.txt").write_text("".join(f"{h}  {p}\n" for h, p in entries), encoding="utf-8")
    (root / "MERKLE_ROOT.txt").write_text(root_hash + "\n", encoding="utf-8")
    return root_hash


def _pkg_versions() -> dict:
    out = {}
    for name in ("can", "cantools", "yaml", "pytest", "msgpack"):
        try:
            mod = __import__(name)
            out[name] = getattr(mod, "__version__", "?")
        except Exception:
            out[name] = None
    return out


def environment() -> dict:
    env = {"python": sys.version.split()[0], "platform": platform.platform(), "machine": platform.machine(),
           "packages": _pkg_versions()}
    try:
        env["ros_distro"] = subprocess.run(["bash", "-c", "echo $ROS_DISTRO"], capture_output=True,
                                           text=True, timeout=5).stdout.strip() or None
    except Exception:
        env["ros_distro"] = None
    return env


def run_manifest(run_name: str, inputs: dict, outputs: dict, params: dict, notes: str = "",
                 evidence_level: str = "Synthetic-validated") -> dict:
    """Who, when, where, with what settings, and the fingerprint of every input and output."""
    def entry(v):
        v = Path(v)
        try:
            rel = v.resolve().relative_to(ROOT).as_posix()
        except ValueError:
            rel = str(v)
        return {"path": rel, "sha256": sha256_file(v) if v.exists() else None}

    return {
        "project": "fsai-open", "version": __version__, "run": run_name,
        "utc_time": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "author": AUTHOR, "ai_assistance": AI_ASSISTANCE, "evidence_level": evidence_level,
        "environment": environment(), "parameters": params,
        "inputs": {k: entry(v) for k, v in inputs.items()},
        "outputs": {k: entry(v) for k, v in outputs.items()},
        "notes": notes,
    }
