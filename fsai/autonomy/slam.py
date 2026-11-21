# fsai-open: EKF-SLAM for cone maps - where am I, and where are the cones?
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: builds a map of the cones while working out where the car is on that map. It uses the
wheel speed and a gyro to guess how the car moved, then corrects that guess each time it sees cones it
already knows. Phantom cones that are not seen again are thrown away. Once the first lap is done, the
map can be frozen and the car only has to find itself on it.

State: [x, y, yaw, gyro_bias, l1x, l1y, l2x, l2y, ...] in the start frame (car starts at 0, 0, 0).
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

from .perception import Cone
from .sensors import UNKNOWN

POSE = 5                                    # x, y, yaw, gyro bias, rear slip gain


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class EkfSlam:
    def __init__(self, cfg, start_pose=(0.0, 0.0, 0.0)):
        s = cfg["slam"]
        self.s = s
        k0 = s["rear_slip_gain"]
        self.mu = np.array([*start_pose, 0.0, k0], dtype=float)
        # the slip gain is learnt while driving (real tyres: cold, warm, damp change it); a car with
        # no slip model (gain 0, the kinematic desk model) keeps it fixed
        self.slip_est = bool(s.get("slip_estimation", True)) and k0 > 0
        self.P = np.diag([1e-6, 1e-6, 1e-6, math.radians(0.5) ** 2,
                          (s.get("slip_sigma", 0.004) ** 2) if self.slip_est else 0.0])
        if not s["bias_estimation"]:
            self.P[3, 3] = 0.0
        self.votes = np.zeros((0, 4))       # colour votes per landmark
        self.seen = np.zeros(0, int)
        self.chances = np.zeros(0, int)
        self.confirmed = np.zeros(0, bool)
        self.frozen = False
        self.wide_m = 0.0                   # extra match distance while re-finding the map (see inflate)
        self.stats = {"updates": 0, "matched": 0, "new": 0, "rejected": 0, "pruned": 0}

    # ------------------------------------------------------------------ views
    @property
    def pose(self):
        return float(self.mu[0]), float(self.mu[1]), float(self.mu[2])

    @property
    def n(self) -> int:
        return (len(self.mu) - POSE) // 2

    def landmarks(self) -> np.ndarray:
        return self.mu[POSE:].reshape(-1, 2)

    def colours(self) -> np.ndarray:
        if self.n == 0:
            return np.zeros(0, int)
        tot = self.votes.sum(axis=1)
        col = np.argmax(self.votes, axis=1)
        return np.where(tot >= 0.5, col, UNKNOWN)

    def cone_map(self, confirmed_only: bool = True) -> np.ndarray:
        """[N,3] = x, y, colour of mapped cones."""
        lm = self.landmarks()
        m = self.confirmed if confirmed_only else np.ones(self.n, bool)
        return np.column_stack([lm[m], self.colours()[m]]) if self.n else np.zeros((0, 3))

    # ------------------------------------------------------------------ predict (every control step)
    def predict(self, v: float, yaw_rate_meas: float, dt: float):
        s = self.s
        x, y, th, b, k = self.mu[:POSE]
        w = yaw_rate_meas - b
        th_mid = th + 0.5 * w * dt
        # rear-axle side slip in a corner: the tyres need a slip angle to make side force. Steady-state
        # linear-tyre estimate vlat = -k * v * (v * yaw_rate); k starts from the calibration value and
        # is estimated as a state (it grows when grip falls)
        vlat = -k * v * v * w
        c, sn = math.cos(th_mid), math.sin(th_mid)
        self.mu[0] = x + (v * c - vlat * sn) * dt
        self.mu[1] = y + (v * sn + vlat * c) * dt
        self.mu[2] = wrap(th + w * dt)
        F = np.eye(POSE)
        F[0, 2] = (-v * sn - vlat * c) * dt
        F[1, 2] = (v * c - vlat * sn) * dt
        F[0, 3] = 0.5 * v * math.sin(th_mid) * dt * dt
        F[1, 3] = -0.5 * v * math.cos(th_mid) * dt * dt
        F[2, 3] = -dt
        F[0, 4] = v * v * w * sn * dt
        F[1, 4] = -v * v * w * c * dt
        sv = s["speed_sigma_frac"] * abs(v) + 0.02
        sw = math.radians(s["yaw_rate_sigma_dps"])
        G = np.array([[math.cos(th_mid) * dt, 0], [math.sin(th_mid) * dt, 0], [0, dt], [0, 0], [0, 0]])
        Q = G @ np.diag([sv * sv, sw * sw]) @ G.T
        sl = (s["lateral_sigma_mps"] + s["lateral_sigma_frac"] * abs(vlat)) * dt   # unmodelled side slip
        nrm = np.array([-sn, c])
        Q[:2, :2] += np.outer(nrm, nrm) * sl * sl
        Q[3, 3] = (math.radians(0.01) ** 2) * dt if s["bias_estimation"] else 0.0
        Q[4, 4] = (s.get("slip_walk_per_s", 2e-4) ** 2) * dt if self.slip_est else 0.0
        P = self.P
        P[:POSE, :POSE] = F @ P[:POSE, :POSE] @ F.T + Q
        if self.n and not self.frozen:           # frozen map: landmark cross terms no longer used
            P[:POSE, POSE:] = F @ P[:POSE, POSE:]
            P[POSE:, :POSE] = P[:POSE, POSE:].T

    # ------------------------------------------------------------------ update (every perception frame)
    def _obs_sigma(self, d: float) -> float:
        return self.s["obs_sigma_m"] + self.s["obs_sigma_per_m"] * d

    def _h(self, li: int):
        """Predicted observation of landmark li in the vehicle frame and its Jacobian blocks."""
        x, y, th = self.mu[:3]
        lx, ly = self.mu[POSE + 2 * li: POSE + 2 * li + 2]
        c, s = math.cos(th), math.sin(th)
        dx, dy = lx - x, ly - y
        z = np.array([c * dx + s * dy, -s * dx + c * dy])
        Hr = np.array([[-c, -s, -s * dx + c * dy, 0.0, 0.0], [s, -c, -c * dx - s * dy, 0.0, 0.0]])
        Hl = np.array([[c, s], [-s, c]])
        return z, Hr, Hl

    def update(self, cones: list[Cone], fov_check=None):
        """cones in the vehicle frame. fov_check(points_vehicle) -> bool mask of what the sensors could see."""
        s = self.s
        self.stats["updates"] += 1
        obs = [c for c in cones if math.hypot(c.x, c.y) <= s["max_landmark_range_m"]]
        if self.n and fov_check is not None:          # bookkeeping for pruning phantom landmarks
            lm_v = self._to_vehicle(self.landmarks())
            vis = fov_check(lm_v) & (np.hypot(lm_v[:, 0], lm_v[:, 1]) <= s["max_landmark_range_m"] * 0.8)
            self.chances[vis] += 1
        if not obs:
            self._prune()
            return
        x, y, th = self.mu[:3]
        c, sn = math.cos(th), math.sin(th)
        zw = np.array([[x + c * o.x - sn * o.y, y + sn * o.x + c * o.y] for o in obs])
        pairs = []                                     # (mahalanobis, obs index, landmark index)
        # a match may never move a cone further than this: the filter's own pose uncertainty grows with
        # distance driven (it can reach metres on a long first lap), and a gate that wide would let a cone
        # 2.5 m along the edge pass as "the same cone" and drag the whole map with it
        cap = s.get("max_match_m", 1.0) + self.wide_m
        if self.n:
            tree = cKDTree(self.landmarks())
            for i, o in enumerate(obs):
                R = np.eye(2) * self._obs_sigma(math.hypot(o.x, o.y)) ** 2
                for li in tree.query_ball_point(zw[i], min(2.5, cap)):
                    if self.frozen and not self._colour_ok(o.colour, li):
                        continue                       # localising: a blue cone never matches a yellow one
                    z, Hr, Hl = self._h(li)
                    idx = np.r_[0:POSE, POSE + 2 * li, POSE + 2 * li + 1]
                    H = np.hstack([Hr, Hl])
                    S = H @ self.P[np.ix_(idx, idx)] @ H.T + R
                    nu = np.array([o.x, o.y]) - z
                    pairs.append((float(nu @ np.linalg.solve(S, nu)), i, li))
        pairs.sort()
        match: dict[int, int] = {}
        used_l = set()
        best_d2 = {}
        for d2, i, li in pairs:
            best_d2[i] = min(best_d2.get(i, math.inf), d2)
            if d2 > s["gate_chi2"] or i in match or li in used_l:
                continue
            match[i] = li
            used_l.add(li)
        # joint update with every matched observation
        if match:
            m = len(match)
            n = len(self.mu)
            H = np.zeros((2 * m, n))
            nu = np.zeros(2 * m)
            Rd = np.zeros(2 * m)
            for k, (i, li) in enumerate(match.items()):
                z, Hr, Hl = self._h(li)
                H[2 * k:2 * k + 2, :POSE] = Hr
                H[2 * k:2 * k + 2, POSE + 2 * li:POSE + 2 * li + 2] = Hl
                nu[2 * k:2 * k + 2] = [obs[i].x - z[0], obs[i].y - z[1]]
                Rd[2 * k:2 * k + 2] = self._obs_sigma(math.hypot(obs[i].x, obs[i].y)) ** 2
            if self.frozen:                             # localisation only: landmarks are fixed
                H[:, POSE:] = 0.0
                PH = self.P[:, :POSE] @ H[:, :POSE].T
                Hl_part = np.zeros((2 * m, 2 * m))
                for k, (i, li) in enumerate(match.items()):
                    _, _, Hl = self._h(li)
                    sl = slice(POSE + 2 * li, POSE + 2 * li + 2)
                    Hl_part[2 * k:2 * k + 2, 2 * k:2 * k + 2] = Hl @ self.P[sl, sl] @ Hl.T
                S = H[:, :POSE] @ PH[:POSE] + np.diag(Rd) + Hl_part
                K = np.zeros((n, 2 * m))
                K[:POSE] = np.linalg.solve(S, PH[:POSE].T).T
            else:
                PH = self.P @ H.T
                S = H @ PH + np.diag(Rd)
                K = np.linalg.solve(S, PH.T).T
            self.mu = self.mu + K @ nu
            self.mu[2] = wrap(self.mu[2])
            if self.frozen:
                Kp = K[:POSE]
                self.P[:POSE, :POSE] -= Kp @ S @ Kp.T
            else:
                self.P = self.P - K @ S @ K.T
                self.P = 0.5 * (self.P + self.P.T)
            for i, li in match.items():
                self._vote(li, obs[i])
                self.seen[li] += 1
                if fov_check is None:                   # no field-of-view model: count sightings only
                    self.chances[li] += 1
            self.stats["matched"] += m
            if m >= 3:
                self.wide_m = 0.0                       # map found again: back to the normal match distance
        # new landmarks
        if not self.frozen:
            for i, o in enumerate(obs):
                if i in match:
                    continue
                if best_d2.get(i, math.inf) < s["new_landmark_chi2"]:
                    self.stats["rejected"] += 1
                    continue
                p = self._to_world(np.array([[o.x, o.y]]))[0]
                if self.n and np.min(np.hypot(*(self.landmarks() - p).T)) < s["min_landmark_spacing_m"]:
                    self.stats["rejected"] += 1
                    continue
                self._add(o)
        self._prune()

    def _colour_ok(self, colour: int, li: int) -> bool:
        """Colour gate for data association. Blue and yellow mark opposite track edges, so matching one
        to the other would shift the car by a track width and still look consistent (seen in a real-time
        trackdrive after a pose jump). Orange and big orange may match each other (size is unsure)."""
        if colour < 0:
            return True
        v = self.votes[li]
        tot = float(v.sum())
        if tot < 1.0:
            return True                                # landmark colour not settled yet
        lc = int(np.argmax(v))
        if v[lc] < 0.7 * tot:
            return True
        side = {0: "edge_l", 1: "edge_r", 2: "orange", 3: "orange"}
        return side[colour] == side[lc]

    def inflate(self, pos_sigma_m: float, yaw_sigma_rad: float):
        """Admit that the pose is less certain than the filter thinks (after a stall or a slide), so the
        next scans are matched with a wider gate. Used to re-find the map instead of giving up."""
        self.P[0, 0] += pos_sigma_m ** 2
        self.P[1, 1] += pos_sigma_m ** 2
        self.P[2, 2] += yaw_sigma_rad ** 2
        self.wide_m = min(1.5, self.wide_m + 3.0 * pos_sigma_m)

    def _vote(self, li: int, o: Cone):
        if o.colour >= 0:
            self.votes[li, o.colour] += max(0.05, o.confidence)

    def _to_world(self, pts):
        x, y, th = self.mu[:3]
        c, s = math.cos(th), math.sin(th)
        return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]])

    def _to_vehicle(self, pts):
        x, y, th = self.mu[:3]
        c, s = math.cos(th), math.sin(th)
        d = pts - np.array([x, y])
        return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])

    def _add(self, o: Cone):
        x, y, th = self.mu[:3]
        c, s = math.cos(th), math.sin(th)
        l = np.array([x + c * o.x - s * o.y, y + s * o.x + c * o.y])
        Gr = np.array([[1, 0, -s * o.x - c * o.y, 0, 0], [0, 1, c * o.x - s * o.y, 0, 0]])
        Gz = np.array([[c, -s], [s, c]])
        R = np.eye(2) * self._obs_sigma(math.hypot(o.x, o.y)) ** 2
        n = len(self.mu)
        Pll = Gr @ self.P[:POSE, :POSE] @ Gr.T + Gz @ R @ Gz.T
        Plx = Gr @ self.P[:POSE, :]
        P = np.zeros((n + 2, n + 2))
        P[:n, :n] = self.P
        P[n:, :n] = Plx
        P[:n, n:] = Plx.T
        P[n:, n:] = Pll
        self.P = P
        self.mu = np.concatenate([self.mu, l])
        self.votes = np.vstack([self.votes, np.zeros(4)])
        self._vote(self.n - 1, o)
        self.seen = np.append(self.seen, 1)
        self.chances = np.append(self.chances, 1)
        self.confirmed = np.append(self.confirmed, False)
        self.stats["new"] += 1

    def _prune(self):
        s = self.s
        ready = self.chances >= s["prune_after_seen"]
        ratio = self.seen / np.maximum(self.chances, 1)
        drop = ready & ~self.confirmed & (ratio < s["prune_ratio"])
        self.confirmed |= ready & ~drop
        if not np.any(drop) or self.frozen:
            return
        keep_l = np.nonzero(~drop)[0]
        idx = np.concatenate([np.arange(POSE), (POSE + 2 * keep_l[:, None] + np.arange(2)).ravel()])
        self.mu = self.mu[idx]
        self.P = self.P[np.ix_(idx, idx)]
        self.votes, self.seen = self.votes[keep_l], self.seen[keep_l]
        self.chances, self.confirmed = self.chances[keep_l], self.confirmed[keep_l]
        self.stats["pruned"] += int(drop.sum())

    def freeze(self):
        """Stop adding cones: from now on only the car's position is estimated (localisation mode).
        Landmarks that were never confirmed are removed first."""
        keep_l = np.nonzero(self.confirmed)[0]
        idx = np.concatenate([np.arange(POSE), (POSE + 2 * keep_l[:, None] + np.arange(2)).ravel()])
        self.stats["pruned"] += int(self.n - len(keep_l))
        self.mu, self.P = self.mu[idx], self.P[np.ix_(idx, idx)]
        self.votes, self.seen = self.votes[keep_l], self.seen[keep_l]
        self.chances, self.confirmed = self.chances[keep_l], self.confirmed[keep_l]
        self.frozen = True
