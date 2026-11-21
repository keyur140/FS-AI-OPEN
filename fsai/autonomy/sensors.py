# fsai-open: pretend sensors for the desk simulator - LiDAR point clouds, camera cone detections, IMU
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: makes up what the car's sensors would see on a pretend track, with realistic flaws:
noise, missed cones, cones hidden behind other cones, wrong colours and phantom cones. The driving
software never sees the true track, only these imperfect readings. All randomness comes from one seed.

Vehicle frame: x forward from the rear axle, y left, z up. Angles in radians unless named _deg.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .track import BIG_ORANGE, Track

UNKNOWN = -1


@dataclass
class CameraDetection:
    x: float
    y: float
    colour: int
    confidence: float


def world_to_vehicle(pts: np.ndarray, pose) -> np.ndarray:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    d = pts - np.array([x, y])
    return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])


def vehicle_to_world(pts: np.ndarray, pose) -> np.ndarray:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]])


def make_clutter(track, rng: np.random.Generator, level: float = 1.0) -> np.ndarray:
    """Things beside a real track that are not cones, as upright cylinders [M,4] = (x, y, radius, height):
    barrier runs, standing people, crouching photographers, and patches of long grass. All stay at
    least 1.5 m outside the cone lines, as at a real event."""
    if level <= 0:
        return np.zeros((0, 4))
    from .track import _normals
    c = track.centreline
    nrm = _normals(c, track.closed)
    L = len(c)
    out = []
    edge = track.width_m / 2
    # barrier runs (water-filled barriers or fences): 0.8 m tall, 4-8 m outside, about a third of the track
    for _ in range(int(level * max(2, L * 0.25 / 40))):
        i0, n = int(rng.integers(L)), int(rng.integers(60, 160))
        off, side = rng.uniform(4.0, 8.0), rng.choice([-1, 1])
        for i in range(i0, i0 + n, 1):
            p = c[i % L] + side * (edge + off) * nrm[i % L]
            out.append([p[0], p[1], 0.18, 0.8])
    # people (1.75 m) and crouching photographers (0.9 m, wider)
    for _ in range(int(level * max(4, L * 0.25 / 25))):
        i, side = int(rng.integers(L)), rng.choice([-1, 1])
        p = c[i] + side * (edge + rng.uniform(1.5, 7.0)) * nrm[i]
        out.append([p[0], p[1], 0.22, 1.75] if rng.random() < 0.6 else [p[0], p[1], 0.32, 0.9])
    # long grass: patches of small tufts, 0.08-0.22 m tall
    for _ in range(int(level * max(6, L * 0.25 / 12))):
        i, side = int(rng.integers(L)), rng.choice([-1, 1])
        centre = c[i] + side * (edge + rng.uniform(1.5, 6.0)) * nrm[i]
        for _ in range(int(rng.integers(4, 12))):
            q = centre + rng.normal(0, 0.35, 2)
            out.append([q[0], q[1], rng.uniform(0.03, 0.07), rng.uniform(0.08, 0.22)])
    out = np.array(out, dtype=float).reshape(-1, 4)
    from scipy.spatial import cKDTree
    d, _ = cKDTree(c).query(out[:, :2])                 # never inside another part of the track
    return out[d > edge + 1.2 + out[:, 2]]


class Lidar:
    """Ray-sampled spinning LiDAR. Each cone is a cone shape (DS26 sizes); the ground is flat.
    Returns points [N,3] in the vehicle frame, with occlusion: per (channel, azimuth) only the nearest hit."""

    def __init__(self, cfg, geo, rng: np.random.Generator):
        s = cfg["sensors"]
        self.mx, self.mz = s["lidar_x_m"], s["lidar_z_m"]
        lo, hi = s["lidar_vfov_deg"]
        self.elev = np.radians(np.linspace(lo, hi, int(s["lidar_channels"])))
        self.hres = math.radians(s["lidar_hres_deg"])
        self.half_fov = math.radians(s["lidar_hfov_deg"]) / 2
        self.rmax = s["lidar_range_m"]
        self.noise = s["lidar_noise_m"]
        self.dropout = s["lidar_dropout"]
        c = geo["cones"]
        self.small = (c["small_base_m"] / 2, c["small_height_m"])
        self.big = (c["big_base_m"] / 2, c["big_height_m"])
        self.rng = rng
        self.shimmer = s.get("shimmer_per_scan", 0) if s.get("clutter_level", 0) > 0 else 0
        self.az_bins = np.arange(-self.half_fov, self.half_fov, self.hres)
        # ground ring for each downward channel
        down = self.elev[self.elev < -1e-3]
        self.ground_r = self.mz / np.tan(-down)
        self.ground_ch = np.nonzero(self.elev < -1e-3)[0]

    def scan(self, cones: np.ndarray, pose, clutter: np.ndarray | None = None) -> np.ndarray:
        """clutter: optional upright cylinders [M,4] (x, y, radius, height) that are not cones."""
        xy = cones[:, :2]
        shape = [(self.big if int(k) == BIG_ORANGE else self.small) + (True,) for k in cones[:, 2]]
        if clutter is not None and len(clutter):
            xy = np.vstack([xy, clutter[:, :2]])
            shape += [(float(r), float(h), False) for r, h in clutter[:, 2:4]]
        rel = world_to_vehicle(xy, pose) - np.array([self.mx, 0.0])
        d = np.hypot(rel[:, 0], rel[:, 1])
        az = np.arctan2(rel[:, 1], rel[:, 0])
        vis = (d < self.rmax) & (d > 0.3) & (np.abs(az) < self.half_fov + 0.05)
        ch_l, az_l, rng_l = [], [], []
        for i in np.nonzero(vis)[0]:
            r_base, h, taper = shape[i]
            z = self.mz + d[i] * np.tan(self.elev)            # beam height at the object's distance
            hit = (z >= 0.0) & (z <= h)
            if not np.any(hit):
                continue
            for ch in np.nonzero(hit)[0]:
                r = r_base * (1 - z[ch] / h) if taper else r_base    # cone radius at that height
                half = math.asin(min(1.0, r / d[i]))
                a0 = math.ceil((az[i] - half + self.half_fov) / self.hres)
                a1 = math.floor((az[i] + half + self.half_fov) / self.hres)
                if a1 < a0:
                    continue
                k = np.arange(a0, a1 + 1)
                beam = -self.half_fov + k * self.hres
                off = d[i] * np.sin(beam - az[i])
                surf = d[i] * np.cos(beam - az[i]) - np.sqrt(np.maximum(r * r - off * off, 0.0))
                ch_l.append(np.full(len(k), ch))
                az_l.append(k)
                rng_l.append(surf / math.cos(self.elev[ch]))
        # ground returns
        n_az = len(self.az_bins)
        for ch, gr in zip(self.ground_ch, self.ground_r):
            if gr > self.rmax:
                continue
            ch_l.append(np.full(n_az, ch))
            az_l.append(np.arange(n_az))
            rng_l.append(np.full(n_az, gr / math.cos(self.elev[ch])))
        if not ch_l:
            return np.zeros((0, 3))
        ch = np.concatenate(ch_l)
        ak = np.concatenate(az_l)
        rg = np.concatenate(rng_l)
        ok = (ak >= 0) & (ak < n_az)
        ch, ak, rg = ch[ok], ak[ok], rg[ok]
        # occlusion: keep the nearest return per (channel, azimuth)
        key = ch * n_az + ak
        order = np.lexsort((rg, key))
        key, rg, ch, ak = key[order], rg[order], ch[order], ak[order]
        first = np.ones(len(key), bool)
        first[1:] = key[1:] != key[:-1]
        ch, ak, rg = ch[first], ak[first], rg[first]
        keep = self.rng.random(len(rg)) > self.dropout
        ch, ak, rg = ch[keep], ak[keep], rg[keep]
        rg = rg + self.rng.normal(0, self.noise, len(rg))
        beam = -self.half_fov + ak * self.hres
        e = self.elev[ch]
        x = rg * np.cos(e) * np.cos(beam) + self.mx
        y = rg * np.cos(e) * np.sin(beam)
        z = self.mz + rg * np.sin(e)
        pts = np.column_stack([x, y, z])
        if self.shimmer:                                # heat shimmer / dust: random low returns, never twice
            k = self.rng.poisson(self.shimmer)
            r_ = self.rng.uniform(2.0, 25.0, k)
            b_ = self.rng.uniform(-self.half_fov, self.half_fov, k)
            sh = np.column_stack([self.mx + r_ * np.cos(b_), r_ * np.sin(b_), self.rng.uniform(0.03, 0.3, k)])
            pts = np.vstack([pts, sh])
        return pts


class Camera:
    """Cone detector output as a trained network plus depth would give it: position, colour, confidence.
    Misses rise with distance, colours get confused, and a few phantom cones appear."""

    def __init__(self, cfg, rng: np.random.Generator):
        s = cfg["sensors"]
        self.s = s
        self.mx = s["camera_x_m"]
        self.half_fov = math.radians(s["camera_hfov_deg"]) / 2
        self.rng = rng

    def detect(self, cones: np.ndarray, pose, clutter: np.ndarray | None = None) -> list[CameraDetection]:
        s, rng = self.s, self.rng
        rel = world_to_vehicle(cones[:, :2], pose) - np.array([self.mx, 0.0])
        d = np.hypot(rel[:, 0], rel[:, 1])
        az = np.arctan2(rel[:, 1], rel[:, 0])
        out = []
        rmax = s["camera_range_m"]
        # simple occlusion: a cone hidden if a nearer cone covers its bearing (within its angular size)
        order = np.argsort(d)
        taken: list[tuple[float, float]] = []
        for i in order:
            if not (0.5 < d[i] < rmax and abs(az[i]) < self.half_fov):
                continue
            hidden = any(abs(az[i] - a) < w for a, w in taken)
            taken.append((az[i], 0.6 * 0.114 / d[i]))
            if hidden:
                continue
            f = d[i] / rmax
            if rng.random() < s["camera_miss_near"] + f * (s["camera_miss_far"] - s["camera_miss_near"]):
                continue
            col = int(cones[i, 2])
            perr = s["camera_colour_error_near"] + f * (s["camera_colour_error_far"] - s["camera_colour_error_near"])
            conf = 1.0 - perr - 0.05 * rng.random()
            if rng.random() < perr:
                col = int(rng.choice([c for c in (0, 1, 2, 3) if c != col]))
                conf = 0.4 + 0.3 * rng.random()
            rr = d[i] * (1 + rng.normal(0, s["camera_range_noise_frac"]))
            bb = az[i] + math.radians(rng.normal(0, s["camera_bearing_noise_deg"]))
            out.append(CameraDetection(self.mx + rr * math.cos(bb), rr * math.sin(bb), col, round(conf, 3)))
        for _ in range(rng.poisson(s["camera_false_per_frame"])):
            rr = rng.uniform(2, rmax)
            bb = rng.uniform(-self.half_fov, self.half_fov)
            out.append(CameraDetection(self.mx + rr * math.cos(bb), rr * math.sin(bb),
                                       int(rng.integers(0, 4)), round(0.3 + 0.3 * rng.random(), 3)))
        if clutter is not None and len(clutter) and s.get("clutter_level", 0) > 0:
            # a person in a yellow jacket or a barrier post can look like a cone to a trained detector
            big = clutter[clutter[:, 3] > 0.5]
            if len(big):
                rel = world_to_vehicle(big[:, :2], pose) - np.array([self.mx, 0.0])
                dd = np.hypot(rel[:, 0], rel[:, 1])
                aa = np.arctan2(rel[:, 1], rel[:, 0])
                for i in np.nonzero((dd > 1.0) & (dd < rmax) & (np.abs(aa) < self.half_fov))[0]:
                    if rng.random() < s["camera_clutter_as_cone"]:
                        out.append(CameraDetection(self.mx + rel[i, 0], rel[i, 1], int(rng.choice([0, 1, 1, 2])),
                                                   round(0.5 + 0.3 * rng.random(), 3)))
        return out


class Imu:
    def __init__(self, cfg, rng):
        s = cfg["sensors"]
        self.noise = math.radians(s["imu_yaw_rate_noise_dps"])
        self.bias = math.radians(s["imu_yaw_rate_bias_dps"])
        self.rng = rng

    def yaw_rate(self, true_rate: float) -> float:
        return true_rate + self.bias + self.rng.normal(0, self.noise)
