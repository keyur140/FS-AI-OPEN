# fsai-open: seeded track generator - acceleration, skidpad, autocross/trackdrive, laid out with cones
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: draws pretend race tracks with cones, the way the rules say they are laid out:
blue cones on the left, yellow on the right, small orange for entry and exit lanes, big orange at the
start, finish and timing lines, never more than 5 m apart. The same seed always gives the same track.

Frame: the car starts at (0, 0) facing +x, which is the emulator's world frame.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
TRACKS_FILE = ROOT / "config" / "tracks.yaml"

BLUE, YELLOW, ORANGE, BIG_ORANGE = 0, 1, 2, 3
COLOUR_NAMES = {BLUE: "blue", YELLOW: "yellow", ORANGE: "orange", BIG_ORANGE: "big_orange"}


def load_geometry(path: Path | str = TRACKS_FILE) -> dict:
    import yaml
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    geo = {sec: {k: (v["value"] if isinstance(v, dict) else v) for k, v in body.items()}
           for sec, body in raw.items()}
    # staging is defined by the rules from the front of the car or its front wheels; the car's own
    # origin is the rear axle, so the offsets come from the vehicle dimensions in ads_dv.yaml
    from ..params import load as load_params
    v = load_params().vehicle
    geo["car"] = {"nose_m": round(v.overall_length_m - v.rear_overhang_m, 3), "wheelbase_m": v.wheelbase_m}
    return geo


@dataclass
class Track:
    """A track: cones [N,3] = (x, y, colour), the true centreline [M,2], and the lines the car must cross."""
    kind: str
    seed: int
    cones: np.ndarray
    centreline: np.ndarray
    closed: bool
    start_line: tuple                      # ((x1, y1), (x2, y2))
    finish_line: tuple | None = None
    width_m: float = 3.0
    laps: int = 1
    meta: dict = field(default_factory=dict)

    @property
    def length_m(self) -> float:
        c = self.centreline
        d = np.diff(np.vstack([c, c[:1]]) if self.closed else c, axis=0)
        return float(np.hypot(d[:, 0], d[:, 1]).sum())

    def colour(self, k: int) -> np.ndarray:
        return self.cones[self.cones[:, 2] == k, :2]

    def to_dict(self) -> dict:
        return {"kind": self.kind, "seed": self.seed, "closed": self.closed, "laps": self.laps,
                "width_m": self.width_m, "length_m": round(self.length_m, 3),
                "start_line": [list(map(float, p)) for p in self.start_line],
                "finish_line": [list(map(float, p)) for p in self.finish_line] if self.finish_line else None,
                "cones": [[round(float(x), 4), round(float(y), 4), int(c)] for x, y, c in self.cones],
                "centreline": [[round(float(x), 4), round(float(y), 4)] for x, y in self.centreline],
                "meta": self.meta}


# ---------------------------------------------------------------------------------- geometry helpers
def _normals(path: np.ndarray, closed: bool) -> np.ndarray:
    if closed:
        d = np.roll(path, -1, axis=0) - np.roll(path, 1, axis=0)
    else:
        d = np.gradient(path, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    return np.stack([-d[:, 1], d[:, 0]], axis=1)          # left-hand normal


def curvature(path: np.ndarray, closed: bool) -> np.ndarray:
    """Signed curvature (1/m, + = turning left) by the three-point (Menger) formula."""
    if closed:
        a, b, c = np.roll(path, 1, axis=0), path, np.roll(path, -1, axis=0)
    else:
        a = np.vstack([path[:1], path[:-1]]); b = path; c = np.vstack([path[1:], path[-1:]])
    ab, bc, ca = b - a, c - b, a - c
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ca, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        k = np.where(den > 1e-12, 2 * cross / den, 0.0)
    return k


def resample(path: np.ndarray, step: float, closed: bool) -> np.ndarray:
    p = np.vstack([path, path[:1]]) if closed else path
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(2, int(round(s[-1] / step)))
    q = np.linspace(0, s[-1], n, endpoint=not closed)
    return np.stack([np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])], axis=1)


def _place_along(boundary: np.ndarray, kappa: np.ndarray, geo: dict, closed: bool) -> np.ndarray:
    """Cones along a boundary: max spacing on straights, tighter in corners (DS 1.1)."""
    c = geo["cones"]
    p = np.vstack([boundary, boundary[:1]]) if closed else boundary
    k = np.concatenate([kappa, kappa[:1]]) if closed else kappa
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    out, pos = [], 0.0
    end = s[-1] - (0.5 * c["corner_spacing_m"] if closed else -1e-9)
    while pos <= end:
        i = min(np.searchsorted(s, pos, side="right") - 1, len(p) - 2)
        f = (pos - s[i]) / max(seg[i], 1e-12)
        out.append(p[i] + f * (p[i + 1] - p[i]))
        tight = abs(k[i]) > 1.0 / c["corner_radius_m"]
        pos += c["corner_spacing_m"] if tight else c["max_spacing_m"]
    if closed and len(out) > 2:
        gap = s[-1] - (pos - (c["corner_spacing_m"] if tight else c["max_spacing_m"]))
        if gap > c["max_spacing_m"] - 0.5:            # close the loop without a long last gap
            q = s[-1] - gap / 2
            i = min(np.searchsorted(s, q, side="right") - 1, len(p) - 2)
            f = (q - s[i]) / max(seg[i], 1e-12)
            out.append(p[i] + f * (p[i + 1] - p[i]))
    return np.array(out)


def _boundaries(centre: np.ndarray, width: float, closed: bool):
    n = _normals(centre, closed)
    return centre + n * width / 2, centre - n * width / 2


def _cones_for(centre, width, geo, closed, rng):
    left, right = _boundaries(centre, width, closed)
    kappa = curvature(centre, closed)
    blue = _place_along(left, kappa, geo, closed)
    yellow = _place_along(right, kappa, geo, closed)
    return blue, yellow


def _jitter(pts: np.ndarray, rng, sigma: float) -> np.ndarray:
    return pts + rng.normal(0.0, sigma, pts.shape) if len(pts) else pts


def _line_cones(p: np.ndarray, heading: float, width: float, colour: int) -> list:
    """Big orange cones a little before and after a line, on both sides (DS 1.1)."""
    t = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-t[1], t[0]])
    out = []
    for along in (-0.5, 0.5):
        for side in (1, -1):
            q = p + along * t + side * (width / 2 + 0.3) * n
            out.append([q[0], q[1], colour])
    return out


def _with_big(cones: np.ndarray, big: list, clearance: float = 1.2) -> np.ndarray:
    """Add big orange cones and remove ordinary cones standing too close to them (they replace them)."""
    big = np.array(big, dtype=float).reshape(-1, 3)
    d = np.min(np.linalg.norm(cones[:, None, :2] - big[None, :, :2], axis=2), axis=1)
    return np.vstack([cones[d > clearance], big])


def _fill_gaps(cones: np.ndarray, centre: np.ndarray, closed: bool, max_gap: float) -> np.ndarray:
    """Removing boundary cones next to the big orange ones can leave a gap longer than allowed (DS 1.1).
    Order each side along the centreline and add a cone half-way across any gap that is too long."""
    from scipy.spatial import cKDTree
    tree = cKDTree(centre)
    tang = np.roll(centre, -1, axis=0) - np.roll(centre, 1, axis=0) if closed else np.gradient(centre, axis=0)
    _, ic = tree.query(cones[:, :2])
    rel = cones[:, :2] - centre[ic]
    side = np.sign(tang[ic, 0] * rel[:, 1] - tang[ic, 1] * rel[:, 0])
    extra = []
    for sign, colour in ((1, BLUE), (-1, YELLOW)):
        m = (side == sign) & np.isin(cones[:, 2], (colour, BIG_ORANGE))
        pts, idx = cones[m, :2], ic[m]
        if len(pts) < 2:
            continue
        seq = pts[np.argsort(idx, kind="stable")]
        if closed:
            seq = np.vstack([seq, seq[:1]])
        for a, b in zip(seq[:-1], seq[1:]):
            gap = math.hypot(*(b - a))
            if gap > max_gap:
                n = int(math.ceil(gap / max_gap))
                for k in range(1, n):
                    q = a + (b - a) * k / n
                    extra.append([q[0], q[1], colour])
    return np.vstack([cones, np.array(extra)]) if extra else cones


def _stack(*groups) -> np.ndarray:
    rows = []
    for pts, colour in groups:
        rows += [[x, y, colour] for x, y in pts]
    return np.array(rows, dtype=float).reshape(-1, 3)


# ---------------------------------------------------------------------------------- the events
def acceleration(seed: int = 0, geo: dict | None = None) -> Track:
    geo = geo or load_geometry()
    rng = np.random.default_rng(seed)
    a, w = geo["acceleration"], geo["track"]["width_m"]
    L, stop = a["length_m"], a["stop_area_m"]
    start_x = geo["car"]["nose_m"] + a["staging_m"]  # FSAI26 D5.3.3: foremost part 0.30 m before the start line
    xs = np.arange(-2.0, start_x + L + stop + 0.01, 0.5)
    centre = np.column_stack([xs, np.zeros(len(xs))])
    drive = centre[centre[:, 0] <= start_x + L]
    blue, yellow = _cones_for(drive, w, geo, False, rng)
    stop_x = np.arange(start_x + L + 5.0, start_x + L + stop + 0.01, 5.0)
    orange = [[x, s * w / 2] for x in stop_x for s in (1, -1)]
    end_y = np.linspace(-w / 2, w / 2, int(math.ceil(w / 1.0)) + 1)          # end of stop area: <= 1 m apart
    orange += [[start_x + L + stop, y] for y in end_y[1:-1]]
    cones = _stack((blue, BLUE), (yellow, YELLOW), (orange, ORANGE))
    cones[:, :2] = _jitter(cones[:, :2], rng, geo["cones"]["placement_noise_m"])
    big = _line_cones(np.array([start_x, 0.0]), 0.0, w, BIG_ORANGE) + \
        _line_cones(np.array([start_x + L, 0.0]), 0.0, w, BIG_ORANGE)
    cones = _with_big(cones, big)
    cones = _fill_gaps(cones, drive, False, geo["cones"]["max_spacing_m"])
    return Track("acceleration", seed, cones, centre, False,
                 start_line=((start_x, w / 2), (start_x, -w / 2)),
                 finish_line=((start_x + L, w / 2), (start_x + L, -w / 2)), width_m=w, laps=1,
                 meta={"length_m": L, "stop_area_m": stop, "start_x": start_x})


def skidpad(seed: int = 0, geo: dict | None = None) -> Track:
    """Figure of eight: right circle (clockwise) laps, then left circle (anticlockwise) laps, exit +x.
    Circles centred at (cx, -R) and (cx, +R) with R = centreline radius; the car crosses (cx, 0)."""
    geo = geo or load_geometry()
    rng = np.random.default_rng(seed)
    g = geo["skidpad"]
    r_in, r_out = g["inner_diameter_m"] / 2, g["outer_diameter_m"] / 2
    w = r_out - r_in
    cx = g["entry_m"] + geo["car"]["nose_m"]       # FSAI26 D4.3.3: foremost part 15 m before the line
    laps = int(g["laps_per_side"])
    R = g["centre_distance_m"] / 2                  # FSAI26 D4.1.2: centres 18.25 m apart
    cR, cL = np.array([cx, -R]), np.array([cx, R])

    def circle(c, r, n):
        a = np.linspace(0, 2 * math.pi, n, endpoint=False)
        return np.column_stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)])

    # FSAI26 D4.1.3: 16 cones inside each inner circle, 13 outside each outer circle. The outer 13 are
    # spread evenly over the part of the outer circle that is clear of the middle crossing and the
    # entry and exit lanes (the pattern of HB21 Fig. 3); cone centres sit half a base off the line
    half = geo["cones"]["small_base_m"] / 2
    ri, ro = r_in - half, r_out + half
    # two of the 16 are the big orange cones beside the timekeeping line, 1 m either side of it
    # (HB21 Fig. 3); the other 14 are spread evenly round the rest of the circle
    ab = math.asin(1.0 / ri)

    def inner_ring(c, toward):                       # toward = angle pointing at the other circle
        rest = np.linspace(toward + ab, toward - ab + 2 * math.pi, int(g["inner_cones"]))[1:-1]
        return np.column_stack([c[0] + ri * np.cos(rest), c[1] + ri * np.sin(rest)])
    in_R, in_L = inner_ring(cR, math.pi / 2), inner_ring(cL, -math.pi / 2)
    clear = w / 2 + 0.6
    a0 = math.asin(min(1.0, (R - clear) / ro))           # outer cones stop where |y| < clear
    n_out = int(g["outer_cones"])
    a_R = np.linspace(math.pi - a0, 2 * math.pi + a0, n_out)        # right circle: the far (-y) side
    out_R = np.column_stack([cR[0] + ro * np.cos(a_R), cR[1] + ro * np.sin(a_R)])
    out_L = np.column_stack([cL[0] + ro * np.cos(a_R + math.pi), cL[1] + ro * np.sin(a_R + math.pi)])
    # right circle driven clockwise: left border is the outer circle (blue), right border inner (yellow)
    # left circle driven anticlockwise: left border is the inner circle (blue), right border outer (yellow)
    entry = np.arange(-2.0, cx - r_out + 0.5, 3.0)
    exit_ = np.arange(cx + r_out - 0.5, cx + g["exit_m"] + 0.01, 3.0)
    lane = [[x, s * w / 2] for x in np.concatenate([entry, exit_]) for s in (1, -1)]
    end_y = np.linspace(-w / 2, w / 2, int(math.ceil(w / 1.0)) + 1)          # end of stop area: <= 1 m apart
    lane += [[cx + g["exit_m"], y] for y in end_y[1:-1]]
    cones = _stack((out_R, BLUE), (in_L, BLUE), (in_R, YELLOW), (out_L, YELLOW), (lane, ORANGE))
    cones[:, :2] = _jitter(cones[:, :2], rng, geo["cones"]["placement_noise_m"])
    # timing line between the circles: big orange cones stand on the inner circle lines, 1 m either side
    yb = R - math.sqrt(ri ** 2 - 1.0)
    cones = np.vstack([cones, [[cx + dx, sy * yb, BIG_ORANGE] for dx in (-1.0, 1.0) for sy in (1, -1)]])
    # reference centreline, in driving order (used for scoring and as the planner's nominal path)
    path = [np.column_stack([np.arange(-2.0, cx, 0.25), np.zeros(len(np.arange(-2.0, cx, 0.25)))])]
    th = np.linspace(math.pi / 2, math.pi / 2 - 2 * math.pi * laps, int(400 * laps), endpoint=False)
    path.append(np.column_stack([cR[0] + R * np.cos(th), cR[1] + R * np.sin(th)]))
    th = np.linspace(-math.pi / 2, -math.pi / 2 + 2 * math.pi * laps, int(400 * laps), endpoint=False)
    path.append(np.column_stack([cL[0] + R * np.cos(th), cL[1] + R * np.sin(th)]))
    ex = np.arange(cx, cx + g["exit_m"] + 0.01, 0.25)
    path.append(np.column_stack([ex, np.zeros(len(ex))]))
    centre = np.vstack(path)
    return Track("skidpad", seed, cones, centre, False,
                 start_line=((cx, w / 2), (cx, -w / 2)), finish_line=((cx, w / 2), (cx, -w / 2)),
                 width_m=w, laps=laps,
                 meta={"centre_right": cR.tolist(), "centre_left": cL.tolist(), "radius_m": R,
                       "inner_radius_m": r_in, "outer_radius_m": r_out, "laps_per_side": laps,
                       "exit_x": cx + g["exit_m"], "timing_x": cx})


def _segments_intersect(p, q) -> bool:
    """True if any two non-adjacent segments of closed polyline p cross (q unused: kept for clarity)."""
    n = len(p)
    a, b = p, np.roll(p, -1, axis=0)
    for i in range(n):
        j = np.arange(i + 2, n if i > 0 else n - 1)
        if not len(j):
            continue
        c, d = a[j], b[j]
        def orient(u, v, w_):
            return np.sign((v[..., 0] - u[..., 0]) * (w_[..., 1] - u[..., 1]) -
                           (v[..., 1] - u[..., 1]) * (w_[..., 0] - u[..., 0]))
        o1 = orient(a[i], b[i], c); o2 = orient(a[i], b[i], d)
        o3 = orient(c, d, a[i]); o4 = orient(c, d, b[i])
        if np.any((o1 != o2) & (o3 != o4)):
            return True
    return False


def _min_clearance(left: np.ndarray, right: np.ndarray, centre: np.ndarray, width: float) -> float:
    """Smallest distance from any boundary point to the centreline, ignoring its own neighbourhood.
    Below width/2 means two parts of the track overlap."""
    from scipy.spatial import cKDTree
    tree = cKDTree(centre)
    worst = math.inf
    n = len(centre)
    for b in (left, right):
        d, idx = tree.query(b, k=12)
        own = np.arange(n)[:, None]
        gap = np.minimum(np.abs(idx - own), n - np.abs(idx - own))
        far = gap > 12
        if np.any(far):
            worst = min(worst, float(d[far].min()))
    return worst


def longest_straight_m(k: np.ndarray, step: float, closed: bool, radius_m: float = 100.0) -> float:
    """Longest run of the centreline whose radius stays above radius_m (counted as a straight)."""
    straight = np.abs(k) < 1.0 / radius_m
    if straight.all():
        return len(k) * step
    if closed:                                      # start the scan at a bend so runs do not wrap
        straight = np.roll(straight, -int(np.argmin(straight)))
    best = run = 0
    for f in straight:
        run = run + 1 if f else 0
        best = max(best, run)
    return best * step


def trackdrive(seed: int = 0, geo: dict | None = None, kind: str = "trackdrive") -> Track:
    """A random closed circuit: a smooth periodic spline through jittered points around a loop,
    rejected and redrawn until it meets every constraint (length, tightest turn, no overlaps)."""
    from scipy.interpolate import CubicSpline
    geo = geo or load_geometry()
    t = geo["trackdrive"]
    w = geo["track"]["width_m"]
    rng = np.random.default_rng(seed)
    for attempt in range(2000):
        n = int(rng.integers(10, 18))
        ang = np.sort(rng.uniform(0, 2 * math.pi, n))
        ang[0] = 0.0
        rad = rng.uniform(14, 55, n)
        sx, sy = rng.uniform(0.7, 1.3), rng.uniform(0.6, 1.0)
        pts = np.column_stack([sx * rad * np.cos(ang), sy * rad * np.sin(ang)])
        pts = np.vstack([pts, pts[:1]])
        u = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        cs = CubicSpline(u, pts, bc_type="periodic")
        dense = cs(np.linspace(0, u[-1], 4000, endpoint=False))
        # scale the whole loop up just enough for its tightest turn to meet the minimum radius
        k0 = curvature(resample(dense, 0.25, True), True)
        L0 = float(np.hypot(*np.diff(np.vstack([dense, dense[:1]]), axis=0).T).sum())
        scale = max(1.0, float(np.max(np.abs(k0))) * t["min_radius_m"] * 1.03, t["length_min_m"] / L0)
        if L0 * scale > t["length_max_m"]:
            continue
        centre = resample(dense * scale, 0.25, True)
        L = len(centre) * 0.25
        k = curvature(centre, True)
        if np.max(np.abs(k)) > 1.0 / t["min_radius_m"]:
            continue
        if longest_straight_m(k, 0.25, True) > t["max_straight_m"]:      # FSAI26 D6.1.1 / D8.1.1
            continue
        left, right = _boundaries(centre, w, True)
        if _segments_intersect(left[::4], None) or _segments_intersect(right[::4], None):
            continue
        if _min_clearance(left, right, centre, w) < w / 2 + 1.5:
            continue
        # start on the straightest stretch, driving anticlockwise; move origin to the staged car
        win = 40
        kk = np.convolve(np.abs(np.concatenate([k, k[:win]])), np.ones(win) / win, "valid")[:len(k)]
        i0 = int(np.argmin(kk))
        centre = np.roll(centre, -i0, axis=0)
        # FSAI26 D6.3.5: front wheels 6 m before the line; the car's origin is the rear axle
        off = int(round((t["start_offset_m"] + geo["car"]["wheelbase_m"]) / 0.25))
        car = centre[-off]
        d = centre[(-off + 1) % len(centre)] - car
        yaw = math.atan2(d[1], d[0])
        rot = np.array([[math.cos(-yaw), -math.sin(-yaw)], [math.sin(-yaw), math.cos(-yaw)]])
        centre = (centre - car) @ rot.T
        centre = np.roll(centre, off, axis=0)            # index 0 is the staged car
        blue, yellow = _cones_for(centre, w, geo, True, rng)
        cones = _stack((blue, BLUE), (yellow, YELLOW))
        cones[:, :2] = _jitter(cones[:, :2], rng, geo["cones"]["placement_noise_m"])
        sl = centre[off]
        dd = centre[off + 1] - centre[off - 1]
        h = math.atan2(dd[1], dd[0])
        nrm = np.array([-math.sin(h), math.cos(h)])
        # drop the ordinary cones right at the start line: the big orange ones mark it
        cones = _with_big(cones, _line_cones(sl, h, w, BIG_ORANGE))
        cones = _fill_gaps(cones, centre, True, geo["cones"]["max_spacing_m"])
        line = (tuple(sl + nrm * w / 2), tuple(sl - nrm * w / 2))
        laps = 1 if kind == "autocross" else int(t["laps"])
        return Track(kind, seed, cones, centre, True, start_line=line, finish_line=line, width_m=w,
                     laps=laps, meta={"attempts": attempt + 1, "min_radius_m": round(float(1 / np.max(np.abs(k))), 3),
                                      "start_index": off})
    raise RuntimeError(f"no valid track found for seed {seed}")


def autocross(seed: int = 0, geo: dict | None = None) -> Track:
    return trackdrive(seed, geo, kind="autocross")


GENERATORS = {"acceleration": acceleration, "skidpad": skidpad, "autocross": autocross, "trackdrive": trackdrive}


def validate(track: Track, geo: dict | None = None) -> list[str]:
    """Every rule the layout must meet. Returns the list of problems (empty = valid)."""
    geo = geo or load_geometry()
    problems = []
    max_sp = geo["cones"]["max_spacing_m"] + 3 * geo["cones"]["placement_noise_m"] * 2
    from scipy.spatial import cKDTree
    if track.kind == "skidpad":
        # each inner circle is a full ring: gaps between angular neighbours around its centre
        for colour, key in ((YELLOW, "centre_right"), (BLUE, "centre_left")):
            c0 = np.array(track.meta[key])
            pts = track.colour(colour)
            r = np.hypot(*(pts - c0).T)
            ring = pts[np.abs(r - track.meta["inner_radius_m"]) < 0.5]
            ring = np.vstack([ring, track.colour(BIG_ORANGE)[np.abs(np.hypot(*(track.colour(BIG_ORANGE) - c0).T)
                                                                     - track.meta["inner_radius_m"]) < 0.5]])
            ang = np.sort(np.arctan2(ring[:, 1] - c0[1], ring[:, 0] - c0[0]))
            gaps = np.diff(np.concatenate([ang, ang[:1] + 2 * math.pi])) * track.meta["inner_radius_m"]
            if len(ring) < 3 or gaps.max() > max_sp:
                problems.append(f"{COLOUR_NAMES[colour]} inner circle cone gap {gaps.max():.2f} m > {max_sp:.2f} m")
    else:
        # spacing in the driving direction: order each side's cones (plus the big orange cones on that
        # side, which stand in for boundary cones at timing lines) along the centreline
        c = track.centreline
        tree = cKDTree(c)
        tang = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0) if track.closed else np.gradient(c, axis=0)
        big = track.colour(BIG_ORANGE)
        for colour, sign in ((BLUE, 1), (YELLOW, -1)):
            pts = track.colour(colour)
            if len(big):
                _, ib = tree.query(big)
                rel = big - c[ib]
                bside = np.sign(tang[ib, 0] * rel[:, 1] - tang[ib, 1] * rel[:, 0])
                pts = np.vstack([pts, big[bside == sign]])
            if len(pts) < 2:
                continue
            _, i = tree.query(pts)
            order = np.argsort(i, kind="stable")
            seq = pts[order]
            if track.closed:
                seq = np.vstack([seq, seq[:1]])
            gaps = np.hypot(*np.diff(seq, axis=0).T)
            if gaps.max() > max_sp:
                problems.append(f"{COLOUR_NAMES[colour]} cone gap {gaps.max():.2f} m > {max_sp:.2f} m")
    if track.kind in ("trackdrive", "autocross"):
        # blue must be on the left of the driving direction, yellow on the right
        c = track.centreline
        tree = cKDTree(c)
        tang = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
        for colour, sign in ((BLUE, 1), (YELLOW, -1)):
            pts = track.colour(colour)
            _, i = tree.query(pts)
            rel = pts - c[i]
            side = np.sign(tang[i, 0] * rel[:, 1] - tang[i, 1] * rel[:, 0])
            if np.any(side != sign):
                problems.append(f"{int(np.sum(side != sign))} {COLOUR_NAMES[colour]} cones on the wrong side")
        kc = curvature(c, True)
        if np.max(np.abs(kc)) > 1.0 / geo["trackdrive"]["min_radius_m"] + 1e-6:
            problems.append("turn tighter than the minimum radius")
        ls = longest_straight_m(kc, float(np.mean(np.hypot(*np.diff(c, axis=0).T))), True)
        if ls > geo["trackdrive"]["max_straight_m"]:
            problems.append(f"straight of {ls:.0f} m is longer than {geo['trackdrive']['max_straight_m']:.0f} m")
    return problems
