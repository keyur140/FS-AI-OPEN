# fsai-open: cone perception - LiDAR point cloud to cones, then camera colour fusion
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: finds cones. The LiDAR gives exact positions but no colour; the camera gives colour
but rougher positions. This file finds cone-shaped clumps of LiDAR points standing on the ground,
then borrows the colour from the nearest camera detection.

The LiDAR part works on real point clouds too (any [N,3] array in the vehicle frame), which is how
the stage 3 real-data run uses it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from .sensors import UNKNOWN, CameraDetection
from .track import BIG_ORANGE, ORANGE


@dataclass
class Cone:
    x: float
    y: float
    colour: int            # 0 blue, 1 yellow, 2 orange, 3 big orange, -1 unknown
    confidence: float      # colour confidence 0..1 (0 when unknown)
    source: str            # "lidar+camera", "lidar", "camera"
    height: float = 0.0
    n_points: int = 0


def fit_ground(points: np.ndarray, rng: np.random.Generator, iters: int = 60, tol: float = 0.05):
    """RANSAC plane z = a x + b y + c over the lowest points. Returns (a, b, c)."""
    if len(points) < 3:
        return 0.0, 0.0, 0.0
    low = points[points[:, 2] < np.percentile(points[:, 2], 60)]
    if len(low) < 3:
        low = points
    if len(low) > 3000:                                  # score candidate planes on a seeded sample
        low = low[rng.choice(len(low), 3000, replace=False)]
    best, best_n = (0.0, 0.0, float(np.median(points[:, 2]))), -1
    for _ in range(iters):
        s = low[rng.choice(len(low), 3, replace=False)]
        A = np.column_stack([s[:, 0], s[:, 1], np.ones(3)])
        if abs(np.linalg.det(A)) < 1e-9:
            continue
        a, b, c = np.linalg.solve(A, s[:, 2])
        if math.hypot(a, b) > 0.15:                     # the track is close to flat
            continue
        n = int(np.sum(np.abs(low[:, 2] - (a * low[:, 0] + b * low[:, 1] + c)) < tol))
        if n > best_n:
            best, best_n = (a, b, c), n
    a, b, c = best
    inl = low[np.abs(low[:, 2] - (a * low[:, 0] + b * low[:, 1] + c)) < tol]
    if len(inl) >= 3:                                   # least-squares refit on inliers
        A = np.column_stack([inl[:, 0], inl[:, 1], np.ones(len(inl))])
        a, b, c = np.linalg.lstsq(A, inl[:, 2], rcond=None)[0]
    return float(a), float(b), float(c)


def lidar_cones(points: np.ndarray, cfg, rng: np.random.Generator, ground=None) -> list[Cone]:
    p = cfg["perception"]
    if len(points) == 0:
        return []
    a, b, c = ground if ground is not None else fit_ground(points, rng, tol=p["ground_tolerance_m"])
    h = points[:, 2] - (a * points[:, 0] + b * points[:, 1] + c)
    obj = points[(h > p["ground_tolerance_m"]) & (h < p["cone_max_height_m"] + 0.2)]
    hh = h[(h > p["ground_tolerance_m"]) & (h < p["cone_max_height_m"] + 0.2)]
    if len(obj) == 0:
        return []
    # 5 cm voxels first: near cones return hundreds of points, and clustering every pair is slow
    key = np.floor(obj[:, :2] / 0.05).astype(np.int64)
    _, inv, cnt = np.unique(key[:, 0] * 1_000_003 + key[:, 1], return_inverse=True, return_counts=True)
    inv = inv.ravel()
    vox = np.zeros((len(cnt), 3))
    np.add.at(vox, inv, obj)
    vox /= cnt[:, None]
    vh = np.full(len(cnt), -np.inf)
    np.maximum.at(vh, inv, hh)
    obj, hh, weight = vox, vh, cnt
    pairs = cKDTree(obj[:, :2]).query_pairs(p["cluster_radius_m"], output_type="ndarray")
    n = len(obj)
    if len(pairs):
        g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
        _, labels = connected_components(g, directed=False)
    else:
        labels = np.arange(n)
    out = []
    vtree = cKDTree(obj[:, :2])
    for lab in np.unique(labels):
        m = labels == lab
        if weight[m].sum() < p["min_points"]:
            continue
        pts, hts = obj[m], hh[m]
        ext = np.ptp(pts[:, :2], axis=0)
        top = float(hts.max())
        if max(ext) > p["cone_max_width_m"] or top > p["cone_max_height_m"] or top < p["cone_min_height_m"]:
            continue
        cxy = (pts[:, :2] * weight[m, None]).sum(axis=0) / weight[m].sum()
        rng_ = float(np.hypot(*cxy))
        if rng_ < p["near_range_m"] and top < p["cone_min_height_near_m"]:
            continue                                  # near and low: grass, kerb or shimmer, not a cone
        # a cone stands alone: raised points just outside the cluster mean grass, a barrier or a person
        ring = vtree.query_ball_point(cxy, max(ext) / 2 + p["isolation_m"])
        if sum(1 for j in ring if not m[j]) > p["isolation_max_points"]:
            continue
        # the LiDAR sees only the near face: push the centre back by about half the visible radius
        push = 0.06 if top < p["big_cone_height_m"] else 0.08
        cxy = cxy + push * cxy / max(rng_, 1e-6)
        big = top >= p["big_cone_height_m"]
        out.append(Cone(float(cxy[0]), float(cxy[1]), BIG_ORANGE if big else UNKNOWN, 0.6 if big else 0.0,
                        "lidar", round(top, 3), int(weight[m].sum())))
    return out


def fuse(lidar: list[Cone], camera: list[CameraDetection], cfg, lidar_range: float,
         camera_range: float | None = None) -> list[Cone]:
    """Give each LiDAR cone the colour of the closest camera detection inside a distance gate.
    Camera-only detections are kept beyond LiDAR range (or where the LiDAR found nothing nearby).
    camera_range: if given (the camera is working), LiDAR blobs the camera does not confirm are dropped
    (on a real track they are mostly grass, kerbs or trackside objects; a far barrier seen by only its
    lower beams can even look like a big orange cone and fake a timing line)."""
    p = cfg["perception"]
    out = []
    used = set()
    if camera:
        cam_xy = np.array([[d.x, d.y] for d in camera])
        tree = cKDTree(cam_xy)
    for c in lidar:
        dist = math.hypot(c.x, c.y)
        gate = p["fusion_gate_m"] + p["fusion_gate_per_m"] * dist
        best = None
        if camera:
            for j in tree.query_ball_point([c.x, c.y], gate):
                if j in used:
                    continue
                dj = math.hypot(cam_xy[j, 0] - c.x, cam_xy[j, 1] - c.y)
                if best is None or dj < best[0]:
                    best = (dj, j)
        if best is not None:
            used.add(best[1])
            det = camera[best[1]]
            colour, conf = det.colour, det.confidence
            if c.colour == BIG_ORANGE and colour == ORANGE:
                colour = BIG_ORANGE               # the LiDAR height decides small vs big orange
            elif c.colour != BIG_ORANGE and colour == BIG_ORANGE:
                colour = ORANGE
            out.append(Cone(c.x, c.y, colour, conf, "lidar+camera", c.height, c.n_points))
        elif camera_range is None:
            out.append(c)
    lidar_xy = np.array([[c.x, c.y] for c in lidar]) if lidar else np.zeros((0, 2))
    for j, det in enumerate(camera):
        if j in used:
            continue
        d = math.hypot(det.x, det.y)
        near_lidar = len(lidar_xy) and np.min(np.hypot(*(lidar_xy - [det.x, det.y]).T)) < 1.0
        if d > lidar_range * 0.95 and not near_lidar:
            out.append(Cone(det.x, det.y, det.colour, det.confidence * 0.8, "camera"))
    return out
