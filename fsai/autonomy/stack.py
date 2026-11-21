# fsai-open: the driving stack - perception, SLAM, planning, control and mission logic in one object
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: the car's brain. It takes in sensor readings, keeps a map, decides where to drive and
how fast, and knows the rules of each event (when a lap is done, when to stop, when to say finished).
It never talks to the car directly: it hands a DriveCommand to the stage 1 bridge, which guards it.

The same object runs in the desk simulator, in the ROS 2 node and on the real car. Nothing in here
reads the true track; it only sees what the sensors report.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

import numpy as np

from ..bridge.core import DriveCommand
from ..can import signals as S
from .control import PathFollower
from .perception import Cone, fuse, lidar_cones
from .planning import centreline_from_map, lap_time, local_path, racing_line, speed_profile
from .slam import EkfSlam
from .track import BIG_ORANGE, BLUE, YELLOW, load_geometry, resample

AS_DRIVING = 3
ACCEL, SKIDPAD, AUTOCROSS, TRACKDRIVE, INSPECT_A, INSPECT_B, DEMO = 1, 2, 3, 4, 5, 6, 7


@dataclass
class StackOutput:
    cmd: DriveCommand | None
    finish: bool = False
    estop: bool = False
    lap: int = 0
    cones_actual: int = 0
    cones_all: int = 0
    status: dict = field(default_factory=dict)


class AutonomyStack:
    def __init__(self, cfg, params, geo: dict | None = None, seed: int = 0):
        self.cfg = cfg
        self.p = params
        self.geo = geo or load_geometry()
        self.rng = np.random.default_rng(seed)
        self.slam = EkfSlam(cfg)
        self.follow = PathFollower(cfg, params.vehicle.wheelbase_m)
        self.mission = 0
        self.t_last = None
        self.t_drive0 = None
        self.odo = 0.0
        self.v = 0.0
        self.phase = "waiting"
        self.lap = 0
        self.crossings = 0
        self.lap_times: list[float] = []
        self._t_cross = None
        self._away = True
        self.driven: list[tuple[float, float]] = []
        self.global_line = None
        self.plan_async = False                       # real-time node: build the racing line on a thread
        self._plan: dict | None = None
        self._inflated_at = None
        self._n_inflate = 0
        self.n_relocalise = 0                         # times the pose search was widened (status)
        self.last_cones: list[Cone] = []
        self.plan_info: dict = {}
        self.finish = False
        self.stopping = False
        self.alerts: list[str] = []
        self._last_plan_t = -1.0
        self.track_width = self.geo["track"]["width_m"]
        self.estop = False
        self.seen: dict[str, float] = {}             # last time each input arrived (watchdog, T4.2.2)
        self.steer_max = None                         # live steering limit from the VCU (0x523), deg
        self.ms: dict = {"step": 0}                   # inspection / demo mission state
        self._stop_from = None
        # (t, x, y, yaw) for the last second, for sensor latency. Dead reckoning only (wheel speed and gyro),
        # never the map-corrected pose: a map correction between a scan's stamp and now must not be counted
        # again when the scan is moved to now, or each correction feeds the next one.
        self._pose_hist: list[tuple] = []
        self._dr = [0.0, 0.0, 0.0]
        self.grip_ratio = 1.0                         # measured / expected yaw rate (1 = car turns as asked)
        self.steer_ratio = 1.0                        # same, gentle corners only: steering compliance
        self._grip_applied = 1.0

    def _grip_observe(self, v: float, yaw_rate: float, dt: float):
        """Real tyres: compare how fast the car turns with how fast it should turn for the steering it
        was given. Well under 1 means the tyres are sliding (cold, damp, bumpy), so slow down."""
        d = math.radians(self.follow.prev_steer)
        if v < 4.0 or abs(d) < math.radians(3.0):
            return
        expected = v / self.p.vehicle.wheelbase_m * math.tan(d)
        ratio = max(0.0, min(1.5, yaw_rate / expected))
        k = min(1.0, dt / 2.0)                          # about 2 s of averaging
        self.grip_ratio += k * (ratio - self.grip_ratio)
        c = self.cfg["control"]
        if c.get("steer_gain_adapt") and abs(v * yaw_rate) < c["steer_gain_lat_max_mps2"]:
            self.steer_ratio += min(1.0, dt / 4.0) * (ratio - self.steer_ratio)
            self.follow.steer_scale = 1.0 / max(0.7, min(1.1, self.steer_ratio))

    def _grip_adapt(self):
        """At each lap line on the racing line: if the car turned clearly less than asked, scale the
        speed profile by the square root of the grip ratio (corner speed goes with sqrt(grip))."""
        sp = self.cfg["speed"]
        g = self.grip_ratio / max(0.7, self.steer_ratio)    # compliance is learnt separately; this is grip
        if not sp.get("grip_adapt") or self.phase != "race" or g >= sp["grip_ratio_min"]:
            return
        f = max(0.6, g / 0.95)
        if f < self._grip_applied - 0.02:
            self.follow.speeds = self.follow.speeds * math.sqrt(f / self._grip_applied)
            self._grip_applied = f
            self.alerts.append(f"grip low (car turns {g:.2f} of the asked rate): speeds scaled to "
                               f"{math.sqrt(f):.2f}")

    def mark(self, name: str, t: float):
        """Tell the watchdog an input arrived (the IMU, from the node or simulator)."""
        self.seen[name] = t

    # ================================================================== inputs
    def yaw_rate_guess(self, v: float) -> float:
        """How fast the car is turning, from speed and the steering it was given, corrected by the
        measured grip ratio. Used only to bridge a short IMU gap: in a corner, the last gyro reading
        can be far from the yaw rate a few tenths of a second later."""
        return v / self.p.vehicle.wheelbase_m * math.tan(math.radians(self.follow.prev_steer)) * self.grip_ratio \
            + float(self.slam.mu[3])                      # the filter subtracts the gyro bias again

    def on_odometry(self, t: float, v: float, yaw_rate: float, observe: bool = True):
        if self.t_last is None:
            self.t_last = t
            return
        dt = t - self.t_last
        self.t_last = t
        self.seen["odometry"] = t
        if dt <= 0:
            return
        self.v = v
        self.odo += v * dt
        w = yaw_rate - self.slam.mu[3]
        th = self._dr[2] + 0.5 * w * dt
        self._dr = [self._dr[0] + v * math.cos(th) * dt, self._dr[1] + v * math.sin(th) * dt, self._dr[2] + w * dt]
        self._pose_hist = (self._pose_hist + [(t, *self._dr)])[-100:]   # replaced whole, never edited in place
        if observe:
            self._grip_observe(v, yaw_rate - self.slam.mu[3], dt)
        self.slam.predict(v, yaw_rate, dt)

    def lidar_fov(self, pts_v: np.ndarray) -> np.ndarray:
        s = self.cfg["sensors"]
        rel = pts_v - np.array([s["lidar_x_m"], 0.0])
        az = np.arctan2(rel[:, 1], rel[:, 0])
        return (rel[:, 0] > 1.0) & (np.abs(az) < math.radians(s["lidar_hfov_deg"]) / 2 * 0.9) & \
               (np.abs(az) < math.radians(s["camera_hfov_deg"]) / 2)

    @staticmethod
    def _pose_at(hist: list, stamp: float):
        ts = [h[0] for h in hist]
        k = int(np.clip(np.searchsorted(ts, stamp), 0, len(ts) - 1))
        return hist[k][1:]

    def _latency_shift(self, cones: list, stamp: float | None, to_stamp: float | None = None) -> list:
        """A scan taken at `stamp` but processed now: move its cones into the car's frame of now (or of
        `to_stamp`), using the pose history (on a real car the LiDAR packets arrive tens of ms after they
        are measured, and the camera is measured at other moments than the LiDAR)."""
        hist = list(self._pose_hist)                  # a copy: the perception thread may read while odometry writes
        if stamp is None or not hist:
            return cones
        end = hist[-1][0] if to_stamp is None else to_stamp
        if abs(end - stamp) < 0.005:
            return cones
        x0, y0, a0 = self._pose_at(hist, stamp)
        x1, y1, a1 = hist[-1][1:] if to_stamp is None else self._pose_at(hist, to_stamp)
        c0, s0, c1, s1 = math.cos(a0), math.sin(a0), math.cos(a1), math.sin(a1)
        from dataclasses import replace
        out = []
        for c in cones:
            wx, wy = x0 + c0 * c.x - s0 * c.y, y0 + s0 * c.x + c0 * c.y
            dx, dy = wx - x1, wy - y1
            out.append(replace(c, x=c1 * dx + s1 * dy, y=-s1 * dx + c1 * dy))
        return out

    def on_sensors(self, t: float, points: np.ndarray | None, camera: list, stamp: float | None = None,
                   camera_stamp: float | None = None) -> list[Cone]:
        """Raw sensors in: LiDAR points [N,3] (vehicle frame) and camera detections.
        stamp: when the scan was measured, if earlier than t (latency compensation).
        camera_stamp: when the camera frame was taken; its detections are moved to the scan's moment."""
        cones = self.detect(t, points, camera, stamp, camera_stamp)
        self.on_cones(t, cones, stamp)
        return self.last_cones

    def detect(self, t: float, points: np.ndarray | None, camera: list, stamp: float | None = None,
               camera_stamp: float | None = None) -> list[Cone]:
        """Perception only: cones in the car's frame at the scan's moment. It does not touch the map, so a
        real-time node can run it on its own thread while control keeps its 100 Hz rhythm."""
        if camera and camera_stamp is not None and stamp is not None:
            camera = self._latency_shift(camera, camera_stamp, to_stamp=stamp)
        lc = lidar_cones(points, self.cfg, self.rng) if points is not None else []
        pc = self.cfg["perception"]
        if camera:
            self.seen["camera"] = t
        alive = pc.get("confirm_with_camera") and t - self.seen.get("camera", -1e9) <= pc["camera_alive_s"]
        return fuse(lc, camera or [], self.cfg, self.cfg["sensors"]["lidar_range_m"],
                    camera_range=self.cfg["sensors"]["camera_range_m"] if alive else None)

    def on_cones(self, t: float, cones: list[Cone], stamp: float | None = None):
        """Already-detected cones in (vehicle frame). Teams with their own detector call this directly.
        stamp: when they were measured, if earlier than t."""
        if stamp is not None:
            cones = self._latency_shift(cones, stamp)
        self.last_cones = cones
        self.seen["perception"] = t
        before = self.slam.stats["matched"]
        self.slam.update(cones, fov_check=self.lidar_fov)
        if self.slam.stats["matched"] > before or not cones or self.slam.n == 0:
            self.seen["match"] = t                     # localisation is still tied to the map
            self._inflated_at = None
            self._n_inflate = 0
        elif self.slam.frozen and t - self.seen.get("match", t) > self.cfg["watchdog"]["relocalise_after_s"]:
            # cones in view but none matched the frozen map: widen the search once per interval, so a
            # short stall or slide is recovered long before the lost-match EBS (watchdog.lost_match_s)
            w = self.cfg["watchdog"]
            if self._n_inflate < w["relocalise_max_steps"] and \
                    (self._inflated_at is None or t - self._inflated_at > w["relocalise_after_s"]):
                self._n_inflate += 1
                self.slam.inflate(self.cfg["watchdog"]["relocalise_pos_sigma_m"],
                                  math.radians(self.cfg["watchdog"]["relocalise_yaw_sigma_deg"]))
                self._inflated_at = t
                self.n_relocalise += 1
        if self.phase in ("explore", "accel", "demo"):
            self._replan_local()

    # ================================================================== mission logic
    def step(self, t: float, as_state: int | None, ami: int, steer_max: float | None = None) -> StackOutput:
        if steer_max:
            self.steer_max = float(steer_max)
        if as_state != AS_DRIVING:
            if self.phase == "waiting":
                return StackOutput(None, status=self.status(t))
            return StackOutput(DriveCommand(speed_mps=0.0), finish=self.finish, lap=self.lap,
                               status=self.status(t))
        if self.phase == "waiting":
            self._start(t, ami)
        pose = self.slam.pose
        self.driven.append(pose[:2])
        self._check_crossing(t, pose)
        cmd = self._mission_step(t, pose)
        self._watchdog(t)
        cmap = self.slam.cone_map()
        return StackOutput(cmd, finish=self.finish, estop=self.estop, lap=min(15, self.lap), cones_actual=len(self.last_cones),
                           cones_all=len(cmap), status=self.status(t))

    def _start(self, t, ami):
        self.mission = ami
        self.t_drive0 = t
        self.phase = {ACCEL: "accel", SKIDPAD: "skidpad", AUTOCROSS: "explore", TRACKDRIVE: "explore",
                      INSPECT_A: "inspect", INSPECT_B: "inspect", DEMO: "demo"}.get(ami, "stop")
        if ami == SKIDPAD:
            self._skidpad_path()
        if self.phase in ("explore", "accel", "demo"):
            self._replan_local()

    def _mission_step(self, t, pose) -> DriveCommand:
        sp, ms = self.cfg["speed"], self.cfg["mission"]
        if self.estop:
            return DriveCommand(speed_mps=0.0, steer_deg=0.0, decel_mps2=ms["stop_decel_max_mps2"])
        if self.stopping:
            if self.v < ms["stop_speed_mps"]:
                self.finish = True
            if self._stop_from is None:
                self._stop_from = self.v
            steer = self.follow.prev_steer
            if self.phase not in ("inspect", "demo") and len(self.follow.path) >= 2:
                # keep steering along the path (for acceleration: the straight line fitted through the
                # lane's gates, which carries on through the stop area)
                steer = self.follow.command(pose, self.v).steer_deg
            return DriveCommand(speed_mps=0.0, steer_deg=steer, decel_mps2=self._stop_decel())
        if self.phase == "inspect":
            return self._inspect(t)
        if self.phase == "demo":
            return self._demo(t, pose)
        if self.phase == "accel":
            done = self.crossings >= 2 or self.odo >= self.geo["acceleration"]["length_m"] + 3.0
            if done:
                self.stopping = True
        if self.phase == "skidpad" and self.follow.progress() >= 0.995:
            self.stopping = True
        if self.phase in ("explore", "race"):
            target_laps = 1 if self.mission == AUTOCROSS else int(self.geo["trackdrive"]["laps"])
            if self.lap >= target_laps:
                self.stopping = True
            elif self.phase == "explore" and self.lap >= 1:
                self._go_racing()
        if self.stopping:
            return self._mission_step(t, pose)
        return self.follow.command(pose, self.v)

    def _stop_decel(self) -> float:
        """Braking for the end of an event, firm enough to stop well inside the allowed distance
        (FSAI26 D5.3.6: 100 m, D4.3.6: 25 m, D6.3.7 / D8.2.7: 30 m after the line)."""
        sp, ms = self.cfg["speed"], self.cfg["mission"]
        if self.mission == ACCEL:
            return sp["accel_brake_mps2"]
        allowed = self.geo["skidpad"]["exit_m"] if self.mission == SKIDPAD else \
            self.geo["trackdrive"]["stop_after_finish_m"]
        d = max(3.0, allowed * ms["stop_target_frac"] - self.geo["car"]["nose_m"])
        need = self._stop_from ** 2 / (2 * d)
        return float(min(ms["stop_decel_max_mps2"], max(sp["a_brake_mps2"], need)))

    # ================================================================== inspection and demo (FSAI26 IN2)
    def _rpm_to_mps(self, rpm: float) -> float:
        ms, veh = self.cfg["mission"], self.p.vehicle
        wheel_rpm = rpm / veh.belt_ratio if ms["rpm_basis"] == "motor" else rpm
        return wheel_rpm * 2 * math.pi / 60 * veh.wheel_radius_m

    def _sweep(self, el: float, left_first: bool = True) -> tuple[float, bool]:
        """Steering 0 -> +max -> -max -> 0 at a steady rate; returns (angle, done)."""
        m = self.steer_max or 20.0
        rate = self.cfg["mission"]["steer_sweep_dps"]
        T = 4 * m / rate
        if el >= T + 1.0:                                       # +1 s for the actuator to settle
            return 0.0, True
        u = min(el, T) * rate
        a = u if u <= m else (2 * m - u if u <= 3 * m else u - 4 * m)
        return (a if left_first else -a), False

    def _inspect(self, t) -> DriveCommand:
        """FSAI26 IN2.2.2. A: sweep the full steering range ending straight, ramp to 200 rpm in 10 s,
        stop within 5 s, then AS Finished. B: spin up to 50 rpm, then trigger the EBS."""
        ms, st = self.cfg["mission"], self.ms
        el = t - self.t_drive0
        if self.mission == INSPECT_A:
            if st["step"] == 0:
                steer, done = self._sweep(el)
                if not done:
                    return DriveCommand(speed_mps=0.0, steer_deg=steer)
                st.update(step=1, t1=t)
            if st["step"] == 1:
                r = min(1.0, (t - st["t1"]) / ms["inspect_a_ramp_s"])
                v_top = self._rpm_to_mps(ms["inspect_a_rpm"])
                if r >= 1.0 and self.v >= 0.97 * v_top:
                    st.update(step=2, t2=t)
                    self.alerts.append(f"inspection A: {ms['inspect_a_rpm']:.0f} rpm reached after {t - st['t1']:.2f} s")
                else:
                    # aim slightly above the ramp so the drivetrain lag does not miss the 10 s mark
                    return DriveCommand(speed_mps=min(v_top, v_top * r * 1.04 + 0.05), steer_deg=0.0,
                                        accel_mps2=max(0.6, v_top / ms["inspect_a_ramp_s"] * 1.5))
            self.stopping = True
            self._stop_from = self.v
            return DriveCommand(speed_mps=0.0, steer_deg=0.0,
                                decel_mps2=max(1.0, self.v / (0.5 * ms["inspect_a_stop_s"])))
        # INSPECT_B
        v_top = self._rpm_to_mps(ms["inspect_b_rpm"])
        if self.v >= 0.97 * v_top or el > 15.0:
            self.estop = True
            self.alerts.append(f"inspection B: {self.v / v_top * ms['inspect_b_rpm']:.0f} rpm, EBS requested")
            return DriveCommand(speed_mps=0.0, steer_deg=0.0)
        return DriveCommand(speed_mps=v_top * 1.03, steer_deg=0.0, accel_mps2=1.0)

    def _demo(self, t, pose) -> DriveCommand:
        """FSAI26 IN2.3.2: sweep steering left and right back to straight, accelerate at least 10 m,
        stop within a further 10 m, accelerate a further 10 m, deploy the EBS; stay within the cones."""
        ms, st = self.cfg["mission"], self.ms
        leg = ms["demo_leg_m"] + ms["demo_leg_margin_m"]
        if st["step"] == 0:
            steer, done = self._sweep(t - self.t_drive0)
            if not done:
                return DriveCommand(speed_mps=0.0, steer_deg=steer)
            st.update(step=1, odo0=self.odo)
            self._replan_local()
        if st["step"] in (1, 3):                                 # drive a leg between the cones
            if self.odo - st["odo0"] >= leg:
                if st["step"] == 3:
                    self.estop = True
                    self.alerts.append(f"demo: second leg {self.odo - st['odo0']:.1f} m, EBS requested")
                    return DriveCommand(speed_mps=0.0, steer_deg=0.0)
                st.update(step=2, odo_brake=self.odo)
            else:
                cmd = self.follow.command(pose, self.v)
                return DriveCommand(speed_mps=min(ms["demo_speed_mps"], cmd.speed_mps), steer_deg=cmd.steer_deg,
                                    accel_mps2=self.cfg["speed"]["a_long_mps2"])
        if st["step"] == 2:                                      # stop well inside the next 10 m
            if self.v < 0.05:
                st.update(step=3, odo0=self.odo)
                self.alerts.append(f"demo: stopped {self.odo - st['odo_brake']:.1f} m after braking")
                return DriveCommand(speed_mps=0.0, steer_deg=0.0)
            d = max(2.0, 0.5 * ms["demo_leg_m"])
            return DriveCommand(speed_mps=0.0, steer_deg=self.follow.prev_steer,
                                decel_mps2=min(ms["stop_decel_max_mps2"], max(2.0, self.v ** 2 / (2 * d) + 1.0)))
        return DriveCommand(speed_mps=0.0, steer_deg=0.0)

    # ================================================================== watchdog (FSAI26 T4.2.2)
    def _watchdog(self, t):
        """FSAI26 T4.2.2: the AI must request the EBS if it detects an error, e.g. a lost sensor."""
        if self.estop or self.finish or self.phase in ("inspect", "waiting", "stop"):
            return
        w = self.cfg["watchdog"]
        el = t - self.t_drive0
        why = None
        if el > w["grace_s"]:
            last = self.seen.get("perception")
            if last is None or t - last > w["perception_timeout_s"]:
                why = "no cones from LiDAR or perception for " + \
                      ("the whole run" if last is None else f"{t - last:.2f} s")
            for name, key in (("odometry", "odometry_timeout_s"), ("imu", "imu_timeout_s")):
                if name in self.seen and t - self.seen[name] > w[key]:
                    why = f"{name} signal lost for {t - self.seen[name]:.2f} s"
            lm = self.seen.get("match")
            if lm is not None and self.v > 1.0 and t - lm > w["lost_match_s"]:
                why = f"lost: cones seen but none matched the map for {t - lm:.2f} s"
        if why:
            self.estop = True
            self.alerts.append(f"EBS requested (FSAI26 T4.2.2): {why} at t={t:.2f}")

    # ================================================================== planning hooks
    def _replan_local(self):
        cmap = self.slam.cone_map(confirmed_only=False)
        path, info = local_path(cmap, self.slam.pose, self.cfg, self.track_width)
        self.plan_info = info
        if self.phase in ("accel", "demo") and len(path) >= 4:
            # FSAI26 D5.1.1: the acceleration lane (and the demo lane like it) is straight. Follow the best
            # straight line through the planned gates, so one odd gate near the finish (big orange,
            # stop-area cones) cannot kink the path at 14 m/s
            u = path - path[0]
            d = np.linalg.svd(u - u.mean(axis=0), full_matrices=False)[2][0]
            if d @ (path[-1] - path[0]) < 0:
                d = -d
            c0 = path.mean(axis=0)
            here = np.array(self.slam.pose[:2])
            s0 = (here - c0) @ d
            ss = np.arange(s0, s0 + 120.0, 0.5)
            path = c0 + ss[:, None] * d[None, :]
            info = dict(info, straight=True)
        sp = self.cfg["speed"]
        if self.phase == "accel":
            cap = sp["max_mps"] * 2
        elif self.phase == "demo":
            cap = self.cfg["mission"]["demo_speed_mps"]
        elif self.mission == AUTOCROSS:
            cap = sp["max_mps"]
        else:
            cap = sp["explore_mps"]
        if self.phase == "accel":
            v = np.full(len(path), cap)
        else:
            v = speed_profile(path, False, self.cfg, self.p.vehicle, v_start=max(self.v, 0.0) + 5.0,
                              v_end=0.0, v_cap=cap)
            v = np.maximum(v, min(cap, 1.0))
        self.follow.set_path(path, v, False)

    def _skidpad_path(self):
        """Nominal skidpad path from the event geometry, relative to the start pose."""
        g = self.geo["skidpad"]
        R = g["centre_distance_m"] / 2
        cx, laps = g["entry_m"] + self.geo["car"]["nose_m"], int(g["laps_per_side"])
        start = np.column_stack([np.arange(0.0, cx, 0.5), np.zeros(int(math.ceil(cx / 0.5)))])
        th = np.linspace(math.pi / 2, math.pi / 2 - 2 * math.pi * laps, int(2 * math.pi * R * laps / 0.5), endpoint=False)
        right = np.column_stack([cx + R * np.cos(th), -R + R * np.sin(th)])
        th = np.linspace(-math.pi / 2, -math.pi / 2 + 2 * math.pi * laps, int(2 * math.pi * R * laps / 0.5), endpoint=False)
        left = np.column_stack([cx + R * np.cos(th), R + R * np.sin(th)])
        ex = np.arange(cx, cx + g["exit_m"] * self.cfg["mission"]["stop_target_frac"], 0.5)
        exit_ = np.column_stack([ex, np.zeros(len(ex))])
        path = np.vstack([start, right, left, exit_])
        v = np.full(len(path), self.cfg["speed"]["skidpad_mps"])
        v[: len(start)] = np.minimum(v[: len(start)], np.linspace(2.0, v[0], len(start)) + 2.0)
        n_end = len(exit_)
        v[-n_end:] = np.linspace(self.cfg["speed"]["skidpad_mps"], 1.0, n_end)
        self.skid_n = (len(start), len(right), len(left), len(exit_))
        self.follow.set_path(path, v, False)

    def _go_racing(self):
        """Lap 1 done: freeze the map, build the racing line and its speed profile, follow it.
        With plan_async (the real-time node) the line is built on its own thread while the car keeps
        following the local path, so control never waits for the planner."""
        if self._plan is None:
            self.slam.freeze()
            cmap, driven = self.slam.cone_map(), np.array(self.driven)
            self._plan = {"done": False}
            if not self.plan_async:
                self._plan.update(self._build_line(cmap, driven), done=True)
            else:
                # a separate process, not a thread: the solver holds Python's interpreter lock, and as a
                # thread it starved perception for over 1 s in a real-time run (sensor-loss EBS)
                import concurrent.futures as cf
                import multiprocessing as mp
                pool = cf.ProcessPoolExecutor(max_workers=1, mp_context=mp.get_context("spawn"))
                fut = pool.submit(build_line, cmap, driven, self.cfg, self.p.vehicle)

                def done(f, box=self._plan, pool=pool):
                    try:
                        box.update(f.result())
                    except Exception as e:                  # noqa: BLE001 - reported as an alert
                        box["error"] = str(e)
                    box["done"] = True
                    pool.shutdown(wait=False)
                fut.add_done_callback(done)
        if not self._plan["done"]:
            return
        if "error" in self._plan:                      # keep exploring safely rather than stop
            self.alerts.append(f"racing line failed ({self._plan['error']}); continuing on the local planner")
            self.phase = "explore_more"
            return
        g = self._plan["line"]
        self.global_line = g
        self.follow.set_path(g["line"], g["speeds"], True)
        self.follow.idx = int(np.argmin(np.hypot(*(g["line"] - np.array(self.slam.pose[:2])).T)))
        self.phase = "race"

    def _build_line(self, cmap, driven) -> dict:
        return build_line(cmap, driven, self.cfg, self.p.vehicle)

    # ================================================================== laps
    def _gates(self) -> np.ndarray:
        """Timing lines from mapped big orange cones: groups of cones within 4.5 m, centroid of each."""
        cmap = self.slam.cone_map(confirmed_only=False)
        big = cmap[cmap[:, 2] == BIG_ORANGE, :2]
        groups: list[list] = []
        for b in big:
            for g in groups:
                if np.hypot(*(np.mean(g, axis=0) - b)) < 4.5:
                    g.append(b)
                    break
            else:
                groups.append([b])
        return np.array([np.mean(g, axis=0) for g in groups if len(g) >= 2]).reshape(-1, 2)

    def _check_crossing(self, t, pose):
        x, y, th = pose
        gates = self._gates()
        if len(gates) == 0:
            if self.mission in (AUTOCROSS, TRACKDRIVE) and self.odo > 30 and math.hypot(x, y) < 3.0 and self._away:
                self._cross(t, fallback=True)
            if math.hypot(x, y) > 10.0:
                self._away = True
            return
        d = np.hypot(gates[:, 0] - x, gates[:, 1] - y)
        g = int(np.argmin(d))
        along = math.cos(th) * (gates[g, 0] - x) + math.sin(th) * (gates[g, 1] - y)
        if d[g] < 4.0 and along < 0.0 and self._away:          # the line is now behind the car
            self._cross(t)
        if d.min() > 6.0:
            self._away = True

    def _cross(self, t, fallback=False):
        self._away = False
        self.crossings += 1
        if self.mission in (AUTOCROSS, TRACKDRIVE):
            if self._t_cross is not None:
                self.lap_times.append(round(t - self._t_cross, 3))
                self.lap += 1
                self._grip_adapt()
            self._t_cross = t
        elif self.mission == SKIDPAD:
            self.lap = min(15, self.crossings)
        if fallback:
            self.alerts.append(f"lap line taken from the start position at t={t:.2f} (no big orange cones mapped)")

    # ================================================================== status
    def status(self, t) -> dict:
        x, y, th = self.slam.pose
        P = self.slam.P
        return {"t": round(t, 3), "mission": S.AMI_STATE.get(self.mission, str(self.mission)), "phase": self.phase,
                "pose": [round(x, 3), round(y, 3), round(math.degrees(th), 2)],
                "pose_sigma_m": round(float(math.sqrt(max(P[0, 0], 0) + max(P[1, 1], 0))), 4),
                "gyro_bias_dps": round(math.degrees(self.slam.mu[3]), 4),
                "rear_slip_gain": round(float(self.slam.mu[4]), 5), "slam_stats": dict(self.slam.stats),
                "relocalise_count": self.n_relocalise,
                "map_cones": int(self.slam.n), "map_frozen": self.slam.frozen,
                "lap": self.lap, "lap_times_s": list(self.lap_times), "odometer_m": round(self.odo, 2),
                "plan": {k: v for k, v in self.plan_info.items()},
                "predicted_lap_s": self.global_line["predicted_lap_s"] if self.global_line else None,
                "cross_track_m": round(self.follow.cross_track, 3), "stopping": self.stopping,
                "grip_ratio": round(self.grip_ratio, 3), "steer_ratio": round(self.steer_ratio, 3),
                "finish": self.finish, "estop": self.estop, "alerts": list(self.alerts[-5:])}


def build_line(cmap, driven, cfg, vehicle) -> dict:
    """Racing line, speeds and predicted lap time from the frozen map (a plain function, so it can run
    in its own process)."""
    try:
        ref, wl, wr = centreline_from_map(cmap, driven)
        line = racing_line(ref, wl, wr, cfg)
        v = speed_profile(line, True, cfg, vehicle)
        return {"line": {"centre": ref, "line": line, "speeds": v,
                         "predicted_lap_s": round(lap_time(line, v, True), 3)}}
    except Exception as e:                              # noqa: BLE001 - reported as an alert
        return {"error": str(e)}
