# fsai-open: closed-loop desk simulator - track + pretend sensors + driving stack + bridge + VCU emulator
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: a whole pretend event. The pretend car (stage 1 emulator) drives on a pretend track;
the pretend sensors look at the track from where the car really is; the driving stack sees only
those readings and sends commands through the real bridge code, as encoded CAN frames, 100 times a
second. A referee watches from outside: cones hit, leaving the track, lap times, and every safety
rule the bridge must keep. The same seed always gives the same run, byte for byte.
"""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

import numpy as np

from ..params import Params, load as load_params
from ..vcu.lockstep import Bench
from . import config as acfg
from .sensors import Camera, Imu, Lidar, make_clutter
from .stack import AutonomyStack
from .track import BIG_ORANGE, GENERATORS, Track, load_geometry

MISSION_OF = {"acceleration": 1, "skidpad": 2, "autocross": 3, "trackdrive": 4,
              "inspection_a": 5, "inspection_b": 6, "demo": 7}
TRACK_OF = {"inspection_a": "acceleration", "inspection_b": "acceleration", "demo": "acceleration"}
STATIC = ("inspection_a", "inspection_b")
# FSAI26 D9.1.7 penalty table: seconds added per cone down or out (DOO) and per off course (OC);
# "DNF" = the run scores nothing. Unsafe stop (USS): DNF, except trackdrive (-50 points).
PENALTIES = {"acceleration": {"doo_s": 2.0, "oc": "DNF", "uss": "DNF"},
             "skidpad": {"doo_s": 0.2, "oc": "DNF", "uss": "DNF"},
             "autocross": {"doo_s": 2.0, "oc": 10.0, "uss": "DNF"},
             "trackdrive": {"doo_s": 2.0, "oc": 10.0, "uss": "-50 points"}}


@dataclass
class Referee:
    """Watches the true car against the true track. Nothing here is visible to the stack."""
    track: Track
    params: Params
    hit: set = field(default_factory=set)
    max_lateral_m: float = 0.0
    off_track_s: float = 0.0
    lat_sum: float = 0.0
    lat_n: int = 0
    lap_marks: list = field(default_factory=list)
    oc_count: int = 0
    _off: bool = False

    def __post_init__(self):
        from scipy.spatial import cKDTree
        self.tree = cKDTree(self.track.centreline)
        v = self.params.vehicle
        self.front = v.overall_length_m - v.rear_overhang_m
        self.back = -v.rear_overhang_m
        self.half_w = v.overall_width_m / 2
        self.cone_tree = cKDTree(self.track.cones[:, :2])
        self._prev_side = None
        self._cursor = 0

    def observe(self, t, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        for i in self.cone_tree.query_ball_point([x + c * 1.0, y + s * 1.0], 3.0):
            dx, dy = self.track.cones[i, 0] - x, self.track.cones[i, 1] - y
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            r = 0.16 if int(self.track.cones[i, 2]) == BIG_ORANGE else 0.12
            if self.back - r <= lx <= self.front + r and abs(ly) <= self.half_w + r:
                self.hit.add(int(i))
        if self.track.kind == "skidpad":       # self-overlapping path: search near the last index only
            cl = self.track.centreline
            lo = max(0, self._cursor - 20)
            seg = cl[lo:self._cursor + 120]
            k = int(np.argmin(np.hypot(seg[:, 0] - x, seg[:, 1] - y)))
            self._cursor = lo + k
            d = float(np.hypot(*(cl[self._cursor] - [x, y])))
        else:
            d, _ = self.tree.query([x, y])
        self.max_lateral_m = max(self.max_lateral_m, float(d))
        self.lat_sum += float(d)
        self.lat_n += 1
        if d > self.track.width_m / 2 + self.half_w:      # whole car outside the lane (FSAI26 D9.1.5 OC)
            self.off_track_s += 0.01
            if not self._off:
                self.oc_count += 1
            self._off = True
        else:
            self._off = False
        # true line crossings: the front of the car crosses a timing line (start, and finish if separate)
        fx, fy = x + c * self.front, y + s * self.front
        lines = [self.track.start_line]
        if self.track.finish_line and self.track.finish_line != self.track.start_line:
            lines.append(self.track.finish_line)
        sides = []
        for (x1, y1), (x2, y2) in lines:
            side = (x2 - x1) * (fy - y1) - (y2 - y1) * (fx - x1)
            near = math.hypot(fx - (x1 + x2) / 2, fy - (y1 + y2) / 2) <= math.hypot(x2 - x1, y2 - y1) / 2 + 0.75
            sides.append((side, near))
        if self._prev_side is not None:
            for (side, near), prev in zip(sides, self._prev_side):
                if near and prev < 0 <= side:
                    self.lap_marks.append(round(t, 2))
        self._prev_side = [sd for sd, _ in sides]


class World:
    def __init__(self, event: str, seed: int = 0, cfg=None, params: Params | None = None,
                 track: Track | None = None, record_every: int = 10, sensor_cut_s: float | None = None,
                 sensor_latency_s: float = 0.0):
        self.event = event
        # tests: deliver each scan this long after it was measured, with its measurement stamp, like the
        # real-time ROS 2 runs and the real car (0 = the deterministic desk default)
        self.sensor_latency_s = sensor_latency_s            # seconds, or (min, max) for a random delay
        self._late: list = []
        self._lat_rng = np.random.default_rng([seed, 6007])
        self.sensor_cut_s = sensor_cut_s           # tests: stop feeding sensors from this time (T4.2.2)
        self.seed = seed
        self.cfg = cfg or acfg.load()
        self.geo = load_geometry()
        self.params = params or load_params()
        if self.params.vehicle.model == "kinematic":   # the no-slip model has no side slip to correct for
            self.cfg = acfg.with_overrides(self.cfg, slam__rear_slip_gain=0.0)
        self.track = track or GENERATORS[TRACK_OF.get(event, event)](seed, self.geo)
        ss = np.random.SeedSequence([seed, 7919])
        r_lidar, r_cam, r_imu, r_stack = (np.random.default_rng(s) for s in ss.spawn(4))
        self.bench = Bench(self.params)
        self.bench.vcu.lifted = event in STATIC
        self.log = {"steer": [], "rpm": [], "v": []}            # 100 Hz truth, for the inspection judge
        self.lidar = Lidar(self.cfg, self.geo, r_lidar)
        self.camera = Camera(self.cfg, r_cam)
        self.imu = Imu(self.cfg, r_imu)
        self.wheel_rng = np.random.default_rng(ss.spawn(1)[0])
        self.clutter = make_clutter(self.track, np.random.default_rng([seed, 4243]),
                                    float(self.cfg["sensors"].get("clutter_level", 0.0)))
        self.stack = AutonomyStack(self.cfg, self.params, self.geo, seed=int(r_stack.integers(1 << 31)))
        self.ref = Referee(self.track, self.params)
        self.record_every = record_every
        self.trace: list = []
        self.cycle = 0
        self._prev_yaw = 0.0
        self.hash = hashlib.sha256()
        self.max_stack_ms = 0.0

    @property
    def t(self):
        return self.bench.t

    def step(self):
        b, vcu = self.bench, self.bench.vcu
        t = b.t
        s = self.cfg["sensors"]
        # odometry: wheel speed as the bridge decodes it from CAN, gyro from the true yaw change
        v_meas = b.bridge.speed_mps() * (1 + self.wheel_rng.normal(0, s["wheel_speed_noise_frac"]))
        yaw_rate = ((vcu.yaw - self._prev_yaw + math.pi) % (2 * math.pi) - math.pi) / 0.01
        self._prev_yaw = vcu.yaw
        cut = self.sensor_cut_s is not None and t >= self.sensor_cut_s
        self.stack.on_odometry(t, max(0.0, v_meas), self.imu.yaw_rate(yaw_rate))
        if not cut:
            self.stack.mark("imu", t)
        pose = (vcu.x, vcu.y, vcu.yaw)
        if self.cycle % int(round(100 / s["lidar_hz"])) == 0 and vcu.state == 3 and not cut:
            pts = self.lidar.scan(self.track.cones, pose, self.clutter)
            cam = self.camera.detect(self.track.cones, pose, self.clutter)
            lat = self.sensor_latency_s
            if isinstance(lat, (tuple, list)):
                lat = float(self._lat_rng.uniform(*lat))
            if lat > 0:
                self._late.append((t + lat, pts, cam, t))
                self._late.sort(key=lambda e: e[0])
            else:
                self.stack.on_sensors(t, pts, cam)
        while self._late and self._late[0][0] <= t + 1e-9:
            _, pts, cam, st = self._late.pop(0)
            self.stack.on_sensors(t, pts, cam, stamp=st, camera_stamp=st)
        st = b.bridge.as_state
        ami = b.bridge._get("VCU2AI_Status", "AMI_STATE", 0)
        out = self.stack.step(t, st, ami, steer_max=b.bridge.last_status.get("angle_max_deg")
                              if b.bridge.last_status else None)
        if out.cmd is not None:
            b.bridge.set_command(out.cmd, t)
        if out.finish and not b.bridge.finish_requested:
            b.bridge.request_finish()
        if out.estop:
            b.bridge.request_estop()
        b.bridge.set_logging(lap=out.lap, cones_actual=out.cones_actual, cones_all=out.cones_all)
        b.step()
        self.cycle += 1
        if vcu.state == 3:
            self.ref.observe(t, vcu.x, vcu.y, vcu.yaw)
        if vcu.state >= 3:
            self.log["steer"].append(vcu.steer)
            self.log["rpm"].append(vcu.wheel_rpm)
            self.log["v"].append(vcu.v)
        if self.cycle % self.record_every == 0:
            sx, sy, sth = self.stack.slam.pose
            row = (round(t, 2), vcu.state, round(vcu.x, 3), round(vcu.y, 3), round(vcu.yaw, 4), round(vcu.v, 3),
                   round(vcu.steer, 2), round(sx, 3), round(sy, 3), round(sth, 4), self.stack.phase, self.stack.lap)
            self.trace.append(row)
            self.hash.update(repr(row).encode())
        return out

    def run(self, max_s: float = 600.0) -> dict:
        b = self.bench
        b.arm(MISSION_OF[self.event])
        b.go()
        # arm()/go() step the bench directly; keep the stack's clock in step
        while b.t < max_s:
            self.step()
            if b.vcu.state in (4, 5):
                break
        for _ in range(20):
            self.step()
        return self.result()

    def result(self) -> dict:
        b, st = self.bench, self.stack
        cmap = st.slam.cone_map()
        from scipy.spatial import cKDTree
        truth = self.track.cones
        if len(cmap):
            d, idx = cKDTree(truth[:, :2]).query(cmap[:, :2])
            matched = d < 1.0
            colour_ok = matched & (cmap[:, 2].astype(int) == truth[idx, 2].astype(int))
            map_err = float(np.sqrt(np.mean(d[matched] ** 2))) if matched.any() else None
            uniq = len(set(idx[matched].tolist()))
        else:
            matched = colour_ok = np.zeros(0, bool)
            map_err, uniq = None, 0
        marks = self.ref.lap_marks
        truth_laps = [round(b2 - a, 2) for a, b2 in zip(marks, marks[1:])]
        pos_err = math.hypot(st.slam.pose[0] - b.vcu.x, st.slam.pose[1] - b.vcu.y)
        final_state = {1: "AS_OFF", 2: "AS_READY", 3: "AS_DRIVING", 4: "AS_EMERGENCY_BRAKE",
                       5: "AS_FINISHED"}[b.vcu.state]
        out = {
            "event": self.event, "seed": self.seed, "track_length_m": round(self.track.length_m, 1),
            "final_state": final_state,
            "faults": [k for k, v in vars(b.vcu.faults).items() if v],
            "bridge_violations": list(b.violations),
            "sim_s": round(b.t, 2), "laps_by_stack": st.lap, "lap_times_stack_s": st.lap_times,
            "lap_marks_referee_s": marks, "lap_times_referee_s": truth_laps,
            "cones_hit": len(self.ref.hit), "off_track_s": round(self.ref.off_track_s, 2),
            "max_lateral_m": round(self.ref.max_lateral_m, 3),
            "mean_lateral_m": round(self.ref.lat_sum / max(1, self.ref.lat_n), 3),
            "max_speed_mps": round(max((r[5] for r in self.trace), default=0.0), 2),
            "map_cones": int(len(cmap)), "map_true_cones": int(uniq), "map_false_cones": int((~matched).sum()),
            "map_colour_accuracy": round(float(colour_ok.sum() / max(1, matched.sum())), 4),
            "map_rmse_m": round(map_err, 4) if map_err is not None else None,
            "final_pose_error_m": round(pos_err, 3),
            "predicted_lap_s": st.global_line["predicted_lap_s"] if st.global_line else None,
            "slam_stats": dict(st.slam.stats), "alerts": list(st.alerts),
            "distance_m": round(st.odo, 1),
            "trace_sha256": self.hash.hexdigest(),
            "oc_count": self.ref.oc_count, "ebs_requested_by_ai": bool(b.bridge.estop_requested),
        }
        if self.event in PENALTIES:
            out.update(self._official(out))
        else:
            out["mission_checks"] = self._mission_checks(out)
        return out

    # ------------------------------------------------------------------ official scoring (FSAI26 D4-D9)
    def stop_distance_m(self) -> tuple[float | None, bool]:
        """How far past the finish line the front of the car stopped, and whether it stopped in the lane."""
        v = self.params.vehicle
        nose = v.overall_length_m - v.rear_overhang_m
        vc, tr = self.bench.vcu, self.track
        fx, fy = vc.x + math.cos(vc.yaw) * nose, vc.y + math.sin(vc.yaw) * nose
        if tr.kind == "acceleration":
            return fx - tr.finish_line[0][0], abs(fy) <= tr.width_m / 2
        if tr.kind == "skidpad":
            return fx - tr.meta["timing_x"], abs(fy) <= tr.width_m / 2
        cl = tr.centreline
        d = np.hypot(cl[:, 0] - fx, cl[:, 1] - fy)
        j = int(np.argmin(d))
        step = float(np.mean(np.hypot(*np.diff(cl, axis=0).T)))
        return ((j - tr.meta["start_index"]) % len(cl)) * step, bool(d[j] <= tr.width_m / 2 + 0.5)

    def _official(self, r: dict) -> dict:
        ev = self.event
        pen = PENALTIES[ev]
        allowed = {"acceleration": self.geo["acceleration"]["stop_area_m"], "skidpad": self.geo["skidpad"]["exit_m"]
                   }.get(ev, self.geo["trackdrive"]["stop_after_finish_m"])
        dist, in_lane = self.stop_distance_m()
        uss = r["final_state"] != "AS_FINISHED" or dist is None or not (0 <= dist <= allowed) or not in_lane
        laps = r["lap_times_referee_s"]
        need = {"acceleration": 1, "skidpad": 4, "autocross": 1, "trackdrive": int(self.geo["trackdrive"]["laps"])}[ev]
        doo, oc = r["cones_hit"], r["oc_count"]
        dnf = []
        if len(laps) < need:
            dnf.append(f"{len(laps)} of {need} laps")
        if oc and pen["oc"] == "DNF":
            dnf.append(f"{oc} off course")
        if uss and pen["uss"] == "DNF":
            dnf.append("unsafe stop")
        raw = None
        if len(laps) >= need:
            raw = laps[0] if ev in ("acceleration", "autocross") else \
                (laps[1] + laps[3]) / 2 if ev == "skidpad" else sum(laps[:need])
        official = None if dnf or raw is None else \
            raw + doo * pen["doo_s"] + (oc * pen["oc"] if isinstance(pen["oc"], float) else 0.0)
        return {"stop_distance_m": round(dist, 2) if dist is not None else None, "stop_allowed_m": allowed,
                "stopped_in_lane": in_lane, "unsafe_stop": uss, "dnf_reasons": dnf,
                "raw_time_s": round(raw, 3) if raw is not None else None,
                "official_time_s": round(official, 3) if official is not None else None,
                "penalty_points": -50 if (uss and ev == "trackdrive") else 0,
                "penalty_rules": pen}

    def _mission_checks(self, r: dict) -> dict:
        """FSAI26 IN2.2.2 (static inspection A and B) and IN2.3.2 (autonomous demo), checked on the truth."""
        steer = np.array(self.log["steer"])
        rpm = np.array(self.log["rpm"])
        v = np.array(self.log["v"])
        am = self.params.vcu.angle_max_deg
        ms = self.cfg["mission"]
        moving = np.flatnonzero(rpm > 5.0)
        first = int(moving[0]) if len(moving) else len(rpm)
        pre = steer[:first]
        sweep_ok = bool(len(pre)) and pre.max() >= 0.95 * am and pre.min() <= -0.95 * am and abs(pre[-1]) < 1.0
        c = {"steering_sweep_full_range_then_straight": sweep_ok}
        if self.event == "inspection_a":
            target = ms["inspect_a_rpm"] if ms["rpm_basis"] == "wheel" else ms["inspect_a_rpm"] / self.params.vehicle.belt_ratio
            reach = np.flatnonzero(rpm >= 0.97 * target)
            ramp_s = (int(reach[0]) - first) * 0.01 if len(reach) else None
            peak = int(np.argmax(rpm)) if len(rpm) else 0
            after = np.flatnonzero(rpm[peak:] < 10.0)
            stop_s = int(after[0]) * 0.01 if len(after) else None
            c.update({"peak_rpm": round(float(rpm.max()), 1) if len(rpm) else 0.0, "ramp_s": ramp_s, "stop_s": stop_s,
                      "reached_target_rpm_in_ramp_time": ramp_s is not None and ramp_s <= ms["inspect_a_ramp_s"] + 0.5,
                      "peak_within_8pct": len(rpm) > 0 and rpm.max() <= target * 1.08,
                      "stopped_within_stop_time": stop_s is not None and stop_s <= ms["inspect_a_stop_s"],
                      "ended_as_finished": r["final_state"] == "AS_FINISHED"})
        elif self.event == "inspection_b":
            target = ms["inspect_b_rpm"] if ms["rpm_basis"] == "wheel" else ms["inspect_b_rpm"] / self.params.vehicle.belt_ratio
            c = {"peak_rpm": round(float(rpm.max()), 1) if len(rpm) else 0.0,
                 "reached_target_rpm": len(rpm) > 0 and 0.95 * target <= rpm.max() <= target * 1.15,
                 "ebs_requested_by_ai": r["ebs_requested_by_ai"],
                 "ended_as_emergency": r["final_state"] == "AS_EMERGENCY_BRAKE"}
        else:                                                      # demo
            legs, d, run = [], 0.0, False
            for x in v:                                            # split the drive at standstills
                if x > 0.05:
                    run, d = True, d + x * 0.01
                elif run:
                    legs.append(d)
                    run, d = False, 0.0
            if run:
                legs.append(d)
            # braking distance in leg 1: from the last moment at full demo speed to the standstill
            stop_m = None
            if legs:
                k_end = int(np.flatnonzero(v > 0.05)[0])
                seg = v[k_end:]
                stop_i = int(np.flatnonzero(seg <= 0.05)[0]) if np.any(seg <= 0.05) else len(seg)
                peak_i = int(np.argmax(seg[:stop_i])) if stop_i else 0
                stop_m = float(np.sum(seg[peak_i:stop_i]) * 0.01)
            c.update({"legs_m": [round(x, 2) for x in legs], "braking_m": round(stop_m, 2) if stop_m is not None else None,
                      "first_leg_at_least_10m": len(legs) >= 1 and legs[0] >= ms["demo_leg_m"],
                      "stopped_within_10m": stop_m is not None and stop_m <= ms["demo_leg_m"],
                      "second_leg_at_least_10m": len(legs) >= 2 and legs[1] >= ms["demo_leg_m"],
                      "ebs_requested_by_ai": r["ebs_requested_by_ai"],
                      "ended_as_emergency": r["final_state"] == "AS_EMERGENCY_BRAKE",
                      "stayed_within_cones": r["cones_hit"] == 0 and r["oc_count"] == 0})
        c = {k: (bool(val) if isinstance(val, (bool, np.bool_)) else val) for k, val in c.items()}
        c["all_ok"] = all(val for val in c.values() if isinstance(val, bool))
        return c
