# fsai-open: path planning - a local path through unseen track, then a racing line on the known map
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: two planners.

1. Local planner, for the first lap when the track is unknown: joins nearby cones into triangles,
   takes the middle of every blue-to-yellow edge as a possible "gate", and searches for the chain of
   gates that looks most like a track (smooth, right width, blue on the left, yellow on the right).
2. Racing line, once the whole track is mapped: bends the centreline as far as the cones allow to
   make the turns as gentle as possible, then works out the fastest safe speed at every point.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import Delaunay, cKDTree

from .sensors import UNKNOWN
from .track import BIG_ORANGE, BLUE, ORANGE, YELLOW, curvature, resample


# ====================================================================================== local
def _to_vehicle(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    d = pts - np.array([x, y])
    return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])


def _to_world(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]])


def _compatible(a: int, b: int) -> bool:
    if a == b and a in (BLUE, YELLOW):
        return False
    return True


def local_path(cones: np.ndarray, pose, cfg, track_width: float = 3.0) -> tuple[np.ndarray, dict]:
    """cones [N,3] world (x, y, colour). Returns (path [M,2] world, info). Path starts at the car."""
    p = cfg["planner"]
    info = {"gates": 0, "mode": "none"}
    if len(cones) == 0:
        return _straight(pose, p["horizon_m"]), {**info, "mode": "straight (no cones)"}
    v = _to_vehicle(cones[:, :2], pose)
    d = np.hypot(v[:, 0], v[:, 1])
    m = (d < p["horizon_m"]) & (v[:, 0] > -4.0)
    v, col = v[m], cones[m, 2].astype(int)
    gates = []                                          # (mid_x, mid_y, width, left_colour, right_colour)
    if len(v) >= 3:
        try:
            tri = Delaunay(v)
            edges = set()
            for simplex in tri.simplices:
                for a, b in ((0, 1), (1, 2), (0, 2)):
                    i, j = sorted((simplex[a], simplex[b]))
                    edges.add((i, j))
        except Exception:
            edges = set()
        for i, j in sorted(edges):
            w = math.hypot(*(v[i] - v[j]))
            if not (p["min_gate_m"] <= w <= p["max_gate_m"]) or not _compatible(col[i], col[j]):
                continue
            mid = (v[i] + v[j]) / 2
            gates.append((mid[0], mid[1], w, i, j))
    path_v = None
    if gates:
        path_v, score = _beam(np.array(gates), v, col, p, track_width)
        info.update(gates=len(gates), mode="delaunay", score=score)
    if path_v is None or len(path_v) < 2:
        path_v = _one_side(v, col, track_width, p["horizon_m"])
        info["mode"] = "one-side offset" if path_v is not None else "straight (no gates)"
        if path_v is None:
            return _straight(pose, p["horizon_m"]), info
    path_v = np.vstack([[0.0, 0.0], path_v])
    path_v = _smooth_open(path_v)
    return _to_world(path_v, pose), info


def _straight(pose, length):
    xs = np.arange(0, length, 0.5)
    return _to_world(np.column_stack([xs, np.zeros(len(xs))]), pose)


def _one_side(v, col, w, horizon):
    """Only one colour visible: follow it at half a track width on the correct side."""
    for colour, sign in ((BLUE, -1), (YELLOW, 1)):
        pts = v[(col == colour) & (v[:, 0] > 0.5)]
        if len(pts) >= 2:
            pts = pts[np.argsort(pts[:, 0])]
            t = np.gradient(pts, axis=0)
            t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-9
            n = np.column_stack([-t[:, 1], t[:, 0]])
            return pts + sign * n * w / 2
    return None


def _beam(gates: np.ndarray, v, col, p, w_nom):
    """Beam search over gate midpoints, from the car forwards."""
    mids = gates[:, :2]
    tree = cKDTree(mids)
    max_turn = math.radians(p["max_turn_deg"])
    beams = [(0.0, [], np.array([0.0, 0.0]), 0.0)]       # (cost, gate indices, position, heading)
    finished = []
    for _ in range(int(p["beam_depth"])):
        nxt = []
        for cost, seq, pos, hd in beams:
            extended = False
            for g in tree.query_ball_point(pos, 6.5):
                if g in seq:
                    continue
                step = mids[g] - pos
                dist = math.hypot(*step)
                if dist < 0.8:
                    continue
                h = math.atan2(step[1], step[0])
                dh = abs((h - hd + math.pi) % (2 * math.pi) - math.pi)
                if dh > max_turn or (not seq and abs(h) > math.radians(70)):
                    continue
                gx, gy, gw, i, j = gates[g]
                i, j = int(i), int(j)
                # which end is on the left of the direction of travel?
                left, right = (i, j) if (step[0] * (v[i, 1] - gy) - step[1] * (v[i, 0] - gx)) > 0 else (j, i)
                c = 0.0
                c += 2.0 * dh ** 2
                c += 0.4 * (gw - w_nom) ** 2
                c += 0.15 * (dist - 3.0) ** 2
                if col[left] == YELLOW or col[right] == BLUE:
                    c += 25.0                                       # colours on the wrong sides
                if col[left] == UNKNOWN or col[right] == UNKNOWN:
                    c += 0.5
                c -= 1.0                                            # reward: going further is good
                nxt.append((cost + c, seq + [g], mids[g], h))
                extended = True
            if not extended and seq:
                finished.append((cost, seq))
        if not nxt:
            break
        nxt.sort(key=lambda b: b[0])
        beams = nxt[: int(p["beam_width"])]
    finished += [(c, s) for c, s, _, _ in beams if s]
    if not finished:
        return None, None
    cost, seq = min(finished, key=lambda f: f[0])
    return mids[seq], round(float(cost), 3)


def _smooth_open(path: np.ndarray) -> np.ndarray:
    """Resample to 0.5 m first, then light smoothing that keeps both ends fixed. Smoothing the sparse
    gate points directly would cut tight corners towards the inside cones."""
    p = resample(path, 0.5, False)
    if len(p) < 3:
        return p
    for _ in range(4):
        p[1:-1] = 0.25 * p[:-2] + 0.5 * p[1:-1] + 0.25 * p[2:]
    return p


# ====================================================================================== global
def centreline_from_map(cone_map: np.ndarray, driven: np.ndarray, step: float = 0.5):
    """Closed centreline and left/right free widths from a frozen cone map and the lap-1 trajectory."""
    # lap 1 starts behind the line (staging) and ends past it, so its start overlaps its end: cut the
    # overlap, or the closed loop folds back on itself at the line
    driven = np.asarray(driven, dtype=float)
    q = max(2, len(driven) // 4)
    i0 = int(np.argmin(np.hypot(*(driven[:q] - driven[-1]).T)))
    if np.hypot(*(driven[i0] - driven[-1])) < 3.0:
        driven = driven[i0 + 1:]
    ref = resample(driven, step, True)
    blue = cone_map[cone_map[:, 2] == BLUE, :2]
    yellow = cone_map[cone_map[:, 2] == YELLOW, :2]
    other = cone_map[(cone_map[:, 2] != BLUE) & (cone_map[:, 2] != YELLOW), :2]
    for _ in range(3):
        n = _normals_closed(ref)
        wl = _side_offset(ref, n, blue, other, +1)
        wr = _side_offset(ref, n, yellow, other, -1)
        shift = (wl - wr) / 2
        ref = ref + n * shift[:, None]
        ref = _smooth_closed(ref, 6)
        ref = resample(ref, step, True)
    n = _normals_closed(ref)
    wl = _side_offset(ref, n, blue, other, +1)
    wr = _side_offset(ref, n, yellow, other, -1)
    return ref, wl, wr


def _normals_closed(p):
    d = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-12
    return np.column_stack([-d[:, 1], d[:, 0]])


def _side_offset(ref, n, side_cones, other, sign):
    """Distance from each reference point to the boundary on one side, measured along the normal,
    interpolated between the cones of that side (periodic)."""
    pts = side_cones if len(side_cones) >= 3 else np.vstack([side_cones, other]) if len(other) else side_cones
    N = len(ref)
    if len(pts) < 3:
        return np.full(N, 1.5)
    tree = cKDTree(ref)
    _, idx = tree.query(pts)
    rel = pts - ref[idx]
    lat = sign * np.einsum("ij,ij->i", rel, n[idx])
    ok = (lat > 0.3) & (lat < 5.0)
    idx, lat = idx[ok], lat[ok]
    if len(idx) < 3:
        return np.full(N, 1.5)
    order = np.argsort(idx)
    idx, lat = idx[order], lat[order]
    xs = np.concatenate([idx - N, idx, idx + N])
    ys = np.concatenate([lat, lat, lat])
    return np.interp(np.arange(N), xs, ys)


def _smooth_closed(p, it):
    for _ in range(it):
        p = 0.25 * np.roll(p, 1, axis=0) + 0.5 * p + 0.25 * np.roll(p, -1, axis=0)
    return p


def racing_line(ref: np.ndarray, wl: np.ndarray, wr: np.ndarray, cfg, iterations: int | None = None):
    """Minimum-curvature line: offsets a_i along the normals, bounded by the free widths minus a margin,
    chosen to minimise the sum of squared second differences (a convex least-squares problem)."""
    from scipy.optimize import minimize
    from scipy.sparse import csr_matrix, diags
    from scipy.sparse import vstack as sp_vstack
    margin = cfg["planner"]["racing_margin_m"]
    its = iterations or int(cfg["planner"]["racing_iterations"])
    line = ref.copy()
    lo_w, hi_w = wr.copy(), wl.copy()                        # free space to the right and left
    for _ in range(its):
        N = len(line)
        n = _normals_closed(line)
        lb = -(lo_w - margin)
        ub = hi_w - margin
        bad = lb > ub
        mid = (lb + ub) / 2
        lb = np.where(bad, mid - 1e-3, lb)
        ub = np.where(bad, mid + 1e-3, ub)
        # second-difference operator (periodic), applied separately to x and y
        i = np.arange(N)
        D = csr_matrix((np.tile([1.0, -2.0, 1.0], N),
                        (np.repeat(i, 3), np.column_stack([(i - 1) % N, i, (i + 1) % N]).ravel())), shape=(N, N))
        A = sp_vstack([D @ diags(n[:, 0]), D @ diags(n[:, 1])]).tocsr()
        b = -np.concatenate([D @ line[:, 0], D @ line[:, 1]])
        # bounded least squares by L-BFGS-B on the sparse normal form: the same convex problem, solved in
        # well under a second (the earlier trust-region solver took over a minute for a 300 m lap, and on
        # the car that minute is spent driving with no new plan)
        AT = A.T.tocsr()

        def f(x):
            r = A @ x - b
            return float(r @ r), 2.0 * (AT @ r)
        x0 = np.clip(np.zeros(N), lb, ub)
        res = minimize(f, x0, jac=True, method="L-BFGS-B", bounds=list(zip(lb, ub)),
                       options={"maxiter": 3000, "ftol": 1e-12, "gtol": 1e-8})
        a = res.x
        new = line + n * a[:, None]
        lo_w = lo_w + a
        hi_w = hi_w - a
        new_rs = resample(new, 0.5, True)
        # carry the widths over to the resampled points
        _, idx = cKDTree(new).query(new_rs)
        lo_w, hi_w = lo_w[idx], hi_w[idx]
        line = new_rs
    return line


def speed_profile(path: np.ndarray, closed: bool, cfg, vehicle=None, v_start: float = 0.0,
                  v_end: float | None = None, v_cap: float | None = None) -> np.ndarray:
    """Fastest speed at each point within lateral grip, drive force and braking limits (forward-backward)."""
    sp = cfg["speed"]
    vmax = v_cap if v_cap is not None else sp["max_mps"]
    gf = sp.get("grip_factor", 1.0)                    # on-site grip knob (cold tyres, damp track)
    a_lat, a_acc, a_brk = gf * sp["a_lat_mps2"], gf * sp["a_long_mps2"], gf * sp["a_brake_mps2"]
    k = np.abs(curvature(path, closed))
    k = np.maximum(k, np.convolve(np.concatenate([k[-3:], k, k[:3]]) if closed else np.pad(k, 3, mode="edge"),
                                  np.ones(7) / 7, "valid"))
    seg = np.hypot(*np.diff(np.vstack([path, path[:1]]) if closed else path, axis=0).T)
    v = np.minimum(vmax, np.sqrt(a_lat / np.maximum(k, 1e-6)))
    power, mass = (vehicle.electrical_power_max_w, vehicle.mass_kg) if vehicle is not None else (None, None)
    N = len(path)

    def avail(vi, ki, a_max, driving):
        lat = vi * vi * ki / a_lat
        a = a_max * math.sqrt(max(0.0, 1 - min(1.0, lat) ** 2))
        if power and driving and vi > 0.5:
            a = min(a, power / (mass * vi))
        return a

    passes = 2 if closed else 1
    if not closed:
        v[0] = min(v[0], max(v_start, 0.0))
        if v_end is not None:
            v[-1] = min(v[-1], v_end)
    for _ in range(passes):
        rng = range(N) if closed else range(N - 1)
        for i in rng:
            j = (i + 1) % N
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * avail(v[i], k[i], a_acc, True) * seg[i]))
        for i in reversed(range(N if closed else N - 1)):
            j = (i + 1) % N
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * avail(v[j], k[j], a_brk, False) * seg[i]))
    return v


def lap_time(path: np.ndarray, v: np.ndarray, closed: bool) -> float:
    seg = np.hypot(*np.diff(np.vstack([path, path[:1]]) if closed else path, axis=0).T)
    vv = np.maximum((v + np.roll(v, -1)) / 2 if closed else (v[:-1] + v[1:]) / 2, 0.1)
    return float(np.sum(seg / vv[: len(seg)]))
