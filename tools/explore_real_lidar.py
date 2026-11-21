# fsai-open: stage 3 exploratory run - the built-in LiDAR cone detector on real, labelled LiDAR scans
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: the simulator's LiDAR is cleaner than a real one. This tool takes real scans recorded by a
Formula Student car, with every cone labelled, and asks: how many real cones does our detector find,
how many things does it wrongly call cones, and how far off are the positions? It runs the same measure on
our own simulated scans, so the gap between simulation and reality is one table.

Input: a coneScenes scene (Chalmers Formula Student, https://github.com/Chalmers-Formula-Student/coneScenes),
as a folder or zip with metadata.json, points/*.bin (float32 x, y, z, intensity, LiDAR frame) and
labels/*.txt (x y z w l h yaw class). The scene is NOT redistributed with fsai-open (its own licence and
access rules apply); only our counts and the input fingerprints are kept.

    python tools/explore_real_lidar.py <scene.zip | scene_dir> [--out runs/exploratory_real]

Exploratory only: one scene, one sensor, no tuning to the data. Settings are the shipped defaults.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import platform
import sys
import zipfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from fsai.autonomy import config as acfg  # noqa: E402
from fsai.autonomy.perception import lidar_cones  # noqa: E402

MATCH_M = 0.5                    # a detection within this of a labelled cone counts as finding it
EGO_M = 1.5                      # points this close to the LiDAR are the car itself
BINS = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 30)]
MAX_RANGE = 20.0                 # headline numbers: cones up to 20 m (our camera range is 22 m)


class Scene:
    """Reads a coneScenes folder or zip without unpacking it."""

    def __init__(self, path: Path):
        self.path = path
        self.hashes: dict[str, str] = {}
        self.zip = zipfile.ZipFile(path) if path.suffix == ".zip" else None
        names = self.zip.namelist() if self.zip else [str(p.relative_to(path)) for p in path.rglob("*") if p.is_file()]
        meta = [n for n in names if n.endswith("metadata.json") and "__MACOSX" not in n]
        if not meta:
            raise SystemExit(f"no metadata.json in {path}")
        self.base = meta[0][: -len("metadata.json")]
        self.meta = json.loads(self.read(meta[0]))

    def read(self, name: str) -> bytes:
        data = self.zip.read(name) if self.zip else (self.path / name).read_bytes()
        self.hashes[name] = hashlib.sha256(data).hexdigest()
        return data

    def frames(self):
        for item in self.meta["data"]:
            pts = np.frombuffer(self.read(self.base + item["pointcloud"]["file"]), dtype=np.float32).reshape(-1, 4)
            labels = []
            for line in self.read(self.base + item["labels"]["file"]).decode().splitlines():
                f = line.split()
                if len(f) >= 8:
                    labels.append((float(f[0]), float(f[1]), float(f[5]), f[7]))
            yield item["pointcloud"]["file"], pts[:, :3].astype(float), labels


def visible(labels, pts):
    """Keep labels with at least one LiDAR point inside their footprint (the dataset's own rule)."""
    out = []
    for x, y, h, k in labels:
        m = (np.abs(pts[:, 0] - x) < 0.23) & (np.abs(pts[:, 1] - y) < 0.23)
        if m.any():
            out.append((x, y, h, k))
    return out


def score(frames, cfg, front_only: bool, rng):
    """Greedy nearest matching per frame; counts per range bin."""
    rows = {b: {"labels": 0, "found": 0, "false": 0, "err": []} for b in BINS}
    big = {"labels": 0, "called_big": 0}
    for _, pts, labels in frames:
        r = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(r > EGO_M) & np.isfinite(pts).all(axis=1)]
        labels = visible(labels, pts)
        if front_only:
            labels = [lb for lb in labels if lb[0] > 0 and abs(math.atan2(lb[1], lb[0])) < math.radians(80)]
        det = lidar_cones(pts, cfg, rng)
        if front_only:
            det = [d for d in det if d.x > 0 and abs(math.atan2(d.y, d.x)) < math.radians(80)]
        used = set()
        for x, y, h, k in sorted(labels, key=lambda lb: math.hypot(lb[0], lb[1])):
            d = math.hypot(x, y)
            b = next((bb for bb in BINS if bb[0] <= d < bb[1]), None)
            if b is None:
                continue
            rows[b]["labels"] += 1
            best, bj = MATCH_M, None
            for j, c in enumerate(det):
                e = math.hypot(c.x - x, c.y - y)
                if j not in used and e < best:
                    best, bj = e, j
            if bj is not None:
                used.add(bj)
                rows[b]["found"] += 1
                rows[b]["err"].append(best)
            if k == "Cone_Big":
                big["labels"] += 1
                big["called_big"] += int(bj is not None and det[bj].colour == 3)
        for j, c in enumerate(det):
            if j not in used:
                d = math.hypot(c.x, c.y)
                b = next((bb for bb in BINS if bb[0] <= d < bb[1]), None)
                if b is not None:
                    rows[b]["false"] += 1
    out = []
    for b in BINS:
        v = rows[b]
        out.append({"range_m": f"{b[0]}-{b[1]}", "labelled_cones": v["labels"], "found": v["found"],
                    "false_cones": v["false"],
                    "recall": round(v["found"] / v["labels"], 3) if v["labels"] else None,
                    "precision": round(v["found"] / (v["found"] + v["false"]), 3) if v["found"] + v["false"] else None,
                    "mean_position_error_m": round(float(np.mean(v["err"])), 3) if v["err"] else None})
    head = [rows[b] for b in BINS if b[1] <= MAX_RANGE]
    L, F, X = sum(v["labels"] for v in head), sum(v["found"] for v in head), sum(v["false"] for v in head)
    errs = [e for v in head for e in v["err"]]
    summary = {"labelled_cones": L, "found": F, "false_cones": X,
               "recall": round(F / L, 3) if L else None, "precision": round(F / (F + X), 3) if F + X else None,
               "mean_position_error_m": round(float(np.mean(errs)), 3) if errs else None,
               "big_cones_labelled": big["labels"], "big_cones_called_big": big["called_big"]}
    return summary, out


def synthetic_frames(cfg, n_frames: int, seed: int):
    """The same measure on our own simulator: autocross tracks, scans every 0.5 s of a clean desk run."""
    from fsai.autonomy.sensors import Lidar
    from fsai.autonomy.sim import MISSION_OF, World
    from fsai.autonomy.track import BIG_ORANGE, BLUE, YELLOW, load_geometry
    names = {BLUE: "Cone_Blue", YELLOW: "Cone_Yellow", BIG_ORANGE: "Cone_Big"}
    w = World("autocross", seed, cfg=cfg)
    lidar = Lidar(cfg, load_geometry(), np.random.default_rng([seed, 99]))
    w.bench.arm(MISSION_OF["autocross"])
    w.bench.go()
    frames, k = [], 0
    mx = cfg["sensors"]["lidar_x_m"]
    while w.t < 120 and len(frames) < n_frames and w.bench.vcu.state in (1, 2, 3):
        w.step()
        k += 1
        if w.bench.vcu.state == 3 and k % 50 == 0:
            v = w.bench.vcu
            pts = lidar.scan(w.track.cones, (v.x, v.y, v.yaw))
            c, s = math.cos(v.yaw), math.sin(v.yaw)
            labels = []
            for x, y, col in w.track.cones[:, :3]:
                dx, dy = x - v.x, y - v.y
                lx, ly = c * dx + s * dy - mx, -s * dx + c * dy
                labels.append((lx, ly, 0.33, names.get(int(col), "Cone_Orange")))
            pts = pts.copy()
            pts[:, 0] -= mx                                   # into the LiDAR's own frame, like the real data
            frames.append((f"synthetic_{k}", pts, labels))
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "runs" / "exploratory_real")
    ap.add_argument("--synthetic-frames", type=int, default=60)
    a = ap.parse_args()
    cfg = acfg.load()
    scene = Scene(a.scene)
    real = list(scene.frames())
    res = {"what": "Exploratory: built-in LiDAR cone detector (shipped settings, no tuning) on real labelled "
                   "LiDAR scans, and the same measure on fsai-open's simulated scans",
           "scene": {"file": a.scene.name, "sha256": hashlib.sha256(a.scene.read_bytes()).hexdigest()
                     if a.scene.is_file() else None, "frames": len(real),
                     "metadata_info": {k: v for k, v in (scene.meta.get("info") or {}).items() if k != "user"}, "source": "coneScenes (Chalmers Formula Student)"},
           "method": {"match_within_m": MATCH_M, "ego_radius_m": EGO_M, "headline_max_range_m": MAX_RANGE,
                      "label_rule": "labels with at least one LiDAR point in their 0.46 m footprint",
                      "views": "all around (360 deg) and front only (+-80 deg, like the stack's field of view)"},
           "real": {}, "synthetic": {}, "input_file_sha256": None,
           "environment": {"python": platform.python_version(), "numpy": np.__version__}}
    for view, front in (("all_around", False), ("front_80deg", True)):
        s, rows = score(real, cfg, front, np.random.default_rng(0))
        res["real"][view] = {"summary": s, "by_range": rows}
    syn = synthetic_frames(cfg, a.synthetic_frames, 0)
    for view, front in (("all_around", False), ("front_80deg", True)):
        s, rows = score(syn, cfg, front, np.random.default_rng(0))
        res["synthetic"][view] = {"summary": s, "by_range": rows, "frames": len(syn)}
    res["input_file_sha256"] = hashlib.sha256(json.dumps(scene.hashes, sort_keys=True).encode()).hexdigest()
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "results.json").write_text(json.dumps(res, indent=1))
    (a.out / "input_files.sha256.json").write_text(json.dumps(scene.hashes, indent=1, sort_keys=True))
    for side in ("real", "synthetic"):
        print(side, json.dumps(res[side]["front_80deg"]["summary"]))


if __name__ == "__main__":
    main()
