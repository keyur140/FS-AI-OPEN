# fsai-open: path following - steering (pure pursuit or Stanley) and speed targets for the bridge
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: turns the planned path into a steering angle and a target speed. Pure pursuit aims
the car at a point a little way down the path (further when faster), like a driver looking ahead.
The output is a DriveCommand for the stage 1 bridge, which enforces every safety rule on top."""
from __future__ import annotations

import math

import numpy as np

from ..bridge.core import DriveCommand


class PathFollower:
    def __init__(self, cfg, wheelbase_m: float):
        self.c = cfg["control"]
        self.sp = cfg["speed"]
        self.L = wheelbase_m
        self.path = np.zeros((0, 2))
        self.speeds = np.zeros(0)
        self.closed = False
        self.idx = 0
        self.prev_steer = 0.0
        self.cross_track = 0.0
        self.steer_scale = 1.0          # > 1 when the road wheels turn less than asked (learnt by the stack)

    def set_path(self, path: np.ndarray, speeds: np.ndarray, closed: bool, keep_progress: bool = False):
        self.path, self.speeds, self.closed = path, speeds, closed
        if not keep_progress:
            self.idx = 0

    def _nearest(self, pos) -> int:
        """Nearest path index, searched only a short way ahead of the last one, so a path that crosses
        itself (skidpad) is never followed from the wrong branch."""
        N = len(self.path)
        if N == 0:
            return 0
        window = np.arange(self.idx - 10, self.idx + 80)
        window = window % N if self.closed else np.clip(window, 0, N - 1)
        d = np.hypot(*(self.path[window] - pos).T)
        best = int(window[int(np.argmin(d))])
        if d.min() > 6.0:                              # lost the thread: global search
            best = int(np.argmin(np.hypot(*(self.path - pos).T)))
        self.idx = best
        return best

    def progress(self) -> float:
        return self.idx / max(1, len(self.path) - 1)

    def command(self, pose, v: float) -> DriveCommand:
        x, y, th = pose
        tau = self.c.get("latency_comp_s", 0.0)
        if tau > 0 and v > 0.5:
            # steer for where the car will be when the actuator acts (dead time + smoothing)
            th_dot = v / self.L * math.tan(math.radians(self.prev_steer))
            x, y = x + v * math.cos(th + 0.5 * th_dot * tau) * tau, y + v * math.sin(th + 0.5 * th_dot * tau) * tau
            th = th + th_dot * tau
        N = len(self.path)
        if N < 2:
            return DriveCommand(speed_mps=0.0, steer_deg=0.0, decel_mps2=self.sp["a_brake_mps2"])
        i = self._nearest(np.array([x, y]))
        c = self.c
        ld = min(c["lookahead_max_m"], max(c["lookahead_min_m"], c["lookahead_min_m"] + c["lookahead_gain_s"] * v))
        # walk along the path to the lookahead distance
        j, acc = i, 0.0
        while acc < ld:
            k = (j + 1) % N if self.closed else min(j + 1, N - 1)
            if k == j:
                break
            acc += math.hypot(*(self.path[k] - self.path[j]))
            j = k
        tx, ty = self.path[j]
        dx, dy = tx - x, ty - y
        lx = math.cos(th) * dx + math.sin(th) * dy
        ly = -math.sin(th) * dx + math.cos(th) * dy
        # signed cross-track error at the nearest point (for logs and Stanley)
        nx, ny = self.path[i]
        self.cross_track = -math.sin(th) * (nx - x) + math.cos(th) * (ny - y)
        if c["lateral"] == "feedforward":
            # path curvature ahead (feedforward) + error feedback tuned as a damped second-order system
            # in distance: k_y = L / d0^2, k_psi = 2 zeta L / d0 (speed-independent for the kinematic model)
            pv = max(1, int(round(max(v, 1.0) * c["preview_s"] / 0.5)))
            ia = (i + pv) % N if self.closed else min(i + pv, N - 2)
            ib = (ia + 2) % N if self.closed else min(ia + 2, N - 1)
            ic = (ia - 2) % N if self.closed else max(ia - 2, 0)
            p0, p1, p2 = self.path[ic], self.path[ia], self.path[ib]
            cross = (p1[0] - p0[0]) * (p2[1] - p1[1]) - (p1[1] - p0[1]) * (p2[0] - p1[0])
            den = math.hypot(*(p1 - p0)) * math.hypot(*(p2 - p1)) * math.hypot(*(p2 - p0))
            kappa = 2 * cross / den if den > 1e-9 else 0.0
            k2 = (i + 1) % N if self.closed else min(i + 1, N - 1)
            k1 = (i - 1) % N if self.closed else max(i - 1, 0)
            seg = self.path[k2] - self.path[k1]
            e_psi = (math.atan2(seg[1], seg[0]) - th + math.pi) % (2 * math.pi) - math.pi
            d0, zeta = c["ff_distance_m"], c["ff_damping"]
            steer = math.atan(self.L * kappa) + (self.L / d0 ** 2) * self.cross_track + (2 * zeta * self.L / d0) * e_psi
        elif c["lateral"] == "stanley":
            k2 = (i + 1) % N if self.closed else min(i + 1, N - 1)
            seg = self.path[k2] - self.path[i]
            heading_err = (math.atan2(seg[1], seg[0]) - th + math.pi) % (2 * math.pi) - math.pi
            steer = heading_err + math.atan2(c["stanley_k"] * self.cross_track, v + 1.0)
        else:
            dist = max(0.5, math.hypot(lx, ly))
            steer = math.atan2(2 * self.L * ly, dist * dist)
        steer_deg = math.degrees(steer) * self.steer_scale
        a = c["steer_smoothing"]
        steer_deg = a * self.prev_steer + (1 - a) * steer_deg
        self.prev_steer = steer_deg
        # speed: the slowest planned speed over the next second of travel, so braking starts in time
        horizon = max(3, int(max(v, 2.0) * 1.0 / 0.5))
        ahead = (np.arange(i, i + horizon) % N) if self.closed else np.clip(np.arange(i, i + horizon), 0, N - 1)
        target = float(np.min(self.speeds[ahead]))
        decel = None
        if target < v - 0.3:
            decel = min(self.sp["a_brake_mps2"], max(1.0, (v * v - target * target) / (2 * max(acc, 2.0)) + 1.0))
        return DriveCommand(speed_mps=target, steer_deg=steer_deg, accel_mps2=self.sp["a_long_mps2"],
                            decel_mps2=decel)
