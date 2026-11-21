# fsai-open: ROS 2 node - pretend track, pretend sensors, marshal and referee for desk runs
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plays the world around the pretend car: a seeded FS-AI track, LiDAR, camera and IMU readings taken
from where the emulated car really is, a marshal who flips the switches and presses Go after 5 s, and
a referee who scores the run and writes the result (JSON) and a replay page (HTML) at the end.

  subscribes  /fsai_emulator/truth   std_msgs/String (JSON)   true pose of the emulated car
              /fsai/autonomy/status  std_msgs/String (JSON)   for the replay (estimated pose, phase, lap)
  publishes   /lidar/points          sensor_msgs/PointCloud2  10 Hz, vehicle frame
              /fsai/camera/cones     sensor_msgs/PointCloud2  10 Hz, x, y, z, colour, confidence
              /imu/data              sensor_msgs/Imu          50 Hz
              /fsai_emulator/inputs  std_msgs/String (JSON)   the marshal's switches
Parameters: event (acceleration | skidpad | autocross | trackdrive), seed, result, replay, max_s.
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Header, String

from fsai import AUTHOR
from fsai.autonomy import config as acfg
from fsai.autonomy.sensors import Camera, Imu as ImuModel, Lidar, make_clutter
from fsai.autonomy.sim import MISSION_OF, TRACK_OF, Referee
from fsai.autonomy.track import GENERATORS, load_geometry
from fsai.params import DEFAULT_FILE, load

from .pc2 import CONE_FIELDS, pack


class SimWorldNode(Node):
    def __init__(self):
        super().__init__("fsai_sim_world")
        self.event = self.declare_parameter("event", "autocross").value
        self.seed = int(self.declare_parameter("seed", 0).value)
        self.result_path = Path(self.declare_parameter("result", "/tmp/fsai_sim_result.json").value)
        self.replay_path = Path(self.declare_parameter("replay", "/tmp/fsai_sim_replay.html").value)
        self.max_s = float(self.declare_parameter("max_s", 900.0).value)
        car = load(self.declare_parameter("config", str(DEFAULT_FILE)).value)
        self.debug = bool(self.declare_parameter("debug", False).value)  # also write truth + stack pose logs
        self.dbg = {"truth": [], "stack": []}
        clutter = float(self.declare_parameter("clutter", 0.0).value)   # 1 = trackside clutter + shimmer
        self.cfg = acfg.load(sensors__clutter_level=clutter)
        geo = load_geometry()
        self.track = GENERATORS[TRACK_OF.get(self.event, self.event)](self.seed, geo)
        self.clutter = make_clutter(self.track, np.random.default_rng([self.seed, 4243]), clutter)
        ss = np.random.SeedSequence([self.seed, 7919])
        r1, r2, r3 = (np.random.default_rng(s) for s in ss.spawn(3))
        self.lidar, self.camera, self.imu = Lidar(self.cfg, geo, r1), Camera(self.cfg, r2), ImuModel(self.cfg, r3)
        self.ref = Referee(self.track, car)
        self.pub_pts = self.create_publisher(PointCloud2, "/lidar/points", 2)
        self.pub_cam = self.create_publisher(PointCloud2, "/fsai/camera/cones", 5)
        self.publish_imu = bool(self.declare_parameter("imu", True).value)   # False: the emulator node does it
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 20)
        self.pub_in = self.create_publisher(String, "/fsai_emulator/inputs", 10)
        self.create_subscription(String, "/fsai_emulator/truth", self.on_truth, 50)
        self.create_subscription(String, "/fsai/autonomy/status", self.on_status, 10)
        self.truth = None
        self.est = None
        self.k = 0
        self.trace = []
        self.started = time.monotonic()
        self.ready_at = None
        self.go_sent = False
        self.done = False
        self.drive_t0 = None
        self.create_timer(0.5, self.marshal)
        # sensors render on their own thread, so the 50 Hz truth / IMU stream never waits for a LiDAR scan
        self.create_timer(0.1, self.sense, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(f"{self.event} seed {self.seed}: {len(self.track.cones)} cones, "
                               f"{self.track.length_m:.0f} m")

    def _switches(self, **kw):
        self.pub_in.publish(String(data=json.dumps(kw)))

    def marshal(self):
        if self.done:
            return
        st = self.truth["state"] if self.truth else None
        if st == 1:
            self._switches(asms=True, tsms=True, ami=MISSION_OF[self.event])
        elif st == 2:
            self.ready_at = self.ready_at or time.monotonic()
            if not self.go_sent and time.monotonic() - self.ready_at > 5.6:
                self._switches(res_go=True)
                self.go_sent = True
                self.get_logger().info("marshal: GO")
        elif st in (4, 5) or (time.monotonic() - self.started > self.max_s):
            self.finish()

    def on_status(self, msg):
        s = json.loads(msg.data)
        if self.debug:
            self.dbg["stack"].append([s["t"], *s["pose"], s.get("slam_stats", {}).get("rejected", 0)])
        self.est = (s["pose"][0], s["pose"][1], math.radians(s["pose"][2]), s["phase"], s["lap"])

    def on_truth(self, msg):
        tr = json.loads(msg.data)
        # when this truth (and its IMU sample) was measured: the emulator's own stamp, so a busy world
        # node that reads it late still stamps the IMU and the scans with the right moment
        now = self.get_clock().now()
        ns = int(tr.get("stamp_ns") or now.nanoseconds)
        if not 0 <= now.nanoseconds - ns < 500_000_000:
            ns = now.nanoseconds
        tr["_stamp"] = Time(nanoseconds=ns).to_msg()
        self.truth = tr
        if self.debug and tr["state"] == 3:
            st = tr["_stamp"]
            self.dbg["truth"].append([st.sec + st.nanosec * 1e-9, tr["x"], tr["y"], tr["yaw"]])
        h = Header(frame_id="base_link")
        h.stamp = tr["_stamp"]                          # stamped when measured, like a real IMU driver
        imu = Imu(header=h)
        imu.angular_velocity.z = float(self.imu.yaw_rate(tr["yaw_rate"]))
        if self.publish_imu:
            self.pub_imu.publish(imu)
        if tr["state"] != 3:
            return
        pose = (tr["x"], tr["y"], tr["yaw"])
        self.drive_t0 = self.drive_t0 or tr["t"]
        for _ in range(2):                              # truth arrives at 50 Hz; referee checks every 10 ms
            self.ref.observe(tr["t"] - self.drive_t0, *pose)
    def sense(self):
        tr = self.truth
        if tr is None or tr["state"] != 3:
            return
        pose = (tr["x"], tr["y"], tr["yaw"])
        h = Header(frame_id="base_link")
        h.stamp = tr["_stamp"]                          # the pose the scan is rendered from, not "now"
        if True:                                        # sensors at 10 Hz
            pts = self.lidar.scan(self.track.cones, pose, self.clutter)
            self.pub_pts.publish(pack(h, pts))
            cam = self.camera.detect(self.track.cones, pose, self.clutter)
            arr = np.array([[d.x, d.y, 0.0, d.colour, d.confidence] for d in cam]).reshape(-1, 5)
            self.pub_cam.publish(pack(h, arr, CONE_FIELDS))
            e = self.est or (0, 0, 0, "-", 0)
            self.trace.append((round(tr["t"] - self.drive_t0, 2), tr["state"], tr["x"], tr["y"], tr["yaw"],
                               tr["speed_mps"], tr["steer_deg"], round(e[0], 3), round(e[1], 3), round(e[2], 4),
                               e[3], e[4]))

    def finish(self):
        self.done = True
        st = self.truth["state"] if self.truth else None
        marks = self.ref.lap_marks
        res = {"event": self.event, "seed": self.seed, "track_length_m": round(self.track.length_m, 1),
               "final_state": {1: "AS_OFF", 2: "AS_READY", 3: "AS_DRIVING", 4: "AS_EMERGENCY_BRAKE",
                               5: "AS_FINISHED"}.get(st, str(st)),
               "faults": [k for k, v in (self.truth or {}).get("faults", {}).items() if v],
               "lap_marks_referee_s": marks, "lap_times_referee_s": [round(b - a, 2) for a, b in zip(marks, marks[1:])],
               "cones_hit": len(self.ref.hit), "off_track_s": round(self.ref.off_track_s, 2),
               "max_lateral_m": round(self.ref.max_lateral_m, 3),
               "max_speed_mps": max((r[5] for r in self.trace), default=0.0),
               "map_true_cones": None, "map_false_cones": None, "map_colour_accuracy": None, "map_rmse_m": None,
               "bridge_violations": [], "trace_sha256": "n/a (real-time run)",
               "note": "ROS 2 real-time run: bridge, emulator, sensors and stack as separate processes over CAN (sim bus)."}
        # Static Inspection B and the Autonomous Demo end with the AI's own EBS request (FSAI26 IN2.2.2, IN2.3.2)
        ebs_end = self.event in ("inspection_b", "demo")
        want = "AS_EMERGENCY_BRAKE" if ebs_end else "AS_FINISHED"
        allowed = {"ai_estop_request"} if ebs_end else set()
        res["ok"] = res["final_state"] == want and set(res["faults"]) <= allowed and res["cones_hit"] == 0 \
            and res["off_track_s"] == 0.0
        self.result_path.parent.mkdir(parents=True, exist_ok=True)
        self.result_path.write_text(json.dumps(res, indent=1))
        if self.debug:
            self.result_path.with_suffix(".debug.json").write_text(json.dumps(self.dbg))
        try:
            from fsai.autonomy.replay import write_replay

            class _W:                                    # the replay writer only needs these fields
                pass
            w = _W()
            w.track, w.trace = self.track, self.trace
            w.stack = type("S", (), {"global_line": None})()
            write_replay(w, res, self.replay_path,
                         footer=f"fsai-open - {AUTHOR['name']} (ORCID {AUTHOR['orcid']}) - ROS 2 desk run, synthetic")
        except Exception as exc:                         # the result JSON is the record; the page is a bonus
            self.get_logger().error(f"replay page not written: {exc}")
        self.get_logger().info(f"RESULT {'OK' if res['ok'] else 'NOT OK'}: {json.dumps(res)}")
        self.exit_now = True                             # main() stops the executor (a raise in a worker thread would not)


def main():
    rclpy.init()
    node = SimWorldNode()
    try:
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(node)
        while rclpy.ok() and not getattr(node, "exit_now", False):
            ex.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
