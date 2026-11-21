# fsai-open: ROS 2 node - the driving stack (perception, SLAM, planning, control, missions)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""The car's brain as a ROS 2 node. It talks only to the fsai-open bridge, never to the CAN bus.

  subscribes  /fsai/status            std_msgs/String (JSON)  AS state, mission, speed from the bridge
              /imu/data               sensor_msgs/Imu         yaw rate (angular_velocity.z) [rad/s]
              /lidar/points           sensor_msgs/PointCloud2 raw LiDAR points, vehicle frame (x fwd, y left)
              /fsai/camera/cones      sensor_msgs/PointCloud2 camera cones: x, y, z, colour, confidence
              /fsai/perception/cones  sensor_msgs/PointCloud2 already-fused cones (use instead of the two above)
  publishes   /fsai/cmd               ackermann_msgs/AckermannDriveStamped   to the bridge
              /fsai/fsg_logging       std_msgs/UInt16MultiArray  [lap, cones_actual, cones_all]
              /fsai/autonomy/status   std_msgs/String (JSON)     phase, pose, map size, laps, alerts
              /fsai/autonomy/path     nav_msgs/Path              the path being followed (map frame)
              /fsai/autonomy/map      sensor_msgs/PointCloud2    mapped cones (map frame)
  calls       /fsai/mission_finished, /fsai/ebs   std_srvs/Trigger

Frames: LiDAR points and cones must be in the vehicle frame (origin at the rear axle). If a sensor sits
elsewhere, give its mount offset in config/autonomy.yaml (sensors.lidar_x_m, sensors.lidar_z_m, camera_x_m).
"""
import collections
import json
import math
import threading

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Header, String, UInt16MultiArray
from std_srvs.srv import Trigger

from fsai.autonomy import config as acfg
from fsai.autonomy.perception import Cone
from fsai.autonomy.sensors import CameraDetection
from fsai.autonomy.stack import AutonomyStack
from fsai.can import signals as S
from fsai.params import DEFAULT_FILE, load

from .pc2 import CONE_FIELDS, pack, unpack

AS_BY_NAME = {v: k for k, v in S.AS_STATE.items()}
AMI_BY_NAME = {v: k for k, v in S.AMI_STATE.items()}


class AutonomyNode(Node):
    def __init__(self):
        super().__init__("fsai_autonomy")
        cfg_path = self.declare_parameter("autonomy_config", str(acfg.AUTONOMY_FILE)).value
        car_path = self.declare_parameter("config", str(DEFAULT_FILE)).value
        self.cfg = acfg.load(cfg_path)
        self.stack = AutonomyStack(self.cfg, load(car_path), seed=0)
        self.stack.plan_async = True                          # control never waits for the racing line
        self.as_state = None
        self.ami = 0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.camera: list = []
        self.steer_max = None
        self.camera_stamp = None
        self.last_imu_ts = -1e9
        self.n_alerts = 0
        self.last_lap = 0
        # move each scan to the car's pose of now using its measurement stamp (sensor latency)
        self.latency_comp = bool(self.declare_parameter("latency_comp", True).value)
        self.max_age = float(self.cfg["perception"].get("max_scan_age_s", 0.4))
        self._dropped = 0
        self._gap = {"imu": 0.0, "tick": 0.0}                # longest wait between callbacks, per second
        self._last_cb = {"imu": 0.0, "tick": 0.0}
        self.finish_called = self.ebs_called = False
        self.create_subscription(String, "/fsai/status", self.on_status, 10)
        # The IMU has its own thread: it only stores (stamp, yaw rate). A slow LiDAR callback can then not
        # hold IMU messages in the queue, which would leave the pose up to 0.5 s in the past.
        self._imu_buf = collections.deque(maxlen=200)
        self._imu_lock = threading.Lock()
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50, callback_group=MutuallyExclusiveCallbackGroup())
        # Perception (the slow part) has its own thread too, and hands finished cones to the main thread.
        # Control then keeps its 100 Hz rhythm while a scan is processed, as it must on the car.
        self._cones_buf = collections.deque(maxlen=4)
        sense = MutuallyExclusiveCallbackGroup()
        self.create_subscription(PointCloud2, "/lidar/points", self.on_lidar, 2, callback_group=sense)
        self.create_subscription(PointCloud2, "/fsai/camera/cones", self.on_camera, 5, callback_group=sense)
        self.create_subscription(PointCloud2, "/fsai/perception/cones", self.on_cones, 5)
        self.pub_cmd = self.create_publisher(AckermannDriveStamped, "/fsai/cmd", 10)
        self.pub_log = self.create_publisher(UInt16MultiArray, "/fsai/fsg_logging", 10)
        self.pub_status = self.create_publisher(String, "/fsai/autonomy/status", 10)
        self.pub_path = self.create_publisher(Path, "/fsai/autonomy/path", 2)
        self.pub_map = self.create_publisher(PointCloud2, "/fsai/autonomy/map", 2)
        self.cli_finish = self.create_client(Trigger, "/fsai/mission_finished")
        self.cli_ebs = self.create_client(Trigger, "/fsai/ebs")
        self.create_timer(0.01, self.tick)
        self.create_timer(0.5, self.publish_viz)
        self._health: list = []
        self._stats_prev: dict = {}
        self.create_timer(1.0, self.health)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        st = json.loads(msg.data)
        self.as_state = AS_BY_NAME.get(st.get("as_state"))
        self.ami = AMI_BY_NAME.get(st.get("ami_state"), 0)
        self.speed = float(st.get("speed_mps") or 0.0)
        self.steer_max = st.get("angle_max_deg")           # live steering limit (0x523), for the sweeps

    def on_imu(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.now()
        with self._imu_lock:
            self._imu_buf.append((ts, now, float(msg.angular_velocity.z)))
            self._gap["imu"] = max(self._gap["imu"], now - self._last_cb["imu"])
            self._last_cb["imu"] = now

    def drain_imu(self):
        """Integrate every stored IMU sample on its own measurement time (main thread only), so a late or
        bunched message does not stretch or shrink a turn. Falls back to the 100 Hz tick if stamps are off."""
        with self._imu_lock:
            buf = list(self._imu_buf)
            self._imu_buf.clear()
        for ts, recv, wz in buf:
            self.yaw_rate = wz
            self.stack.mark("imu", recv)                     # watchdog: FSAI26 T4.2.2 sensor loss -> EBS
            if abs(recv - ts) < 0.5:
                self.stack.on_odometry(ts, self.speed, wz)   # samples older than the pose are skipped
                self.last_imu_ts = recv

    def on_camera(self, msg):
        self.camera_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.camera = [CameraDetection(float(x), float(y), int(round(c)), float(p)) for x, y, _, c, p in unpack(msg, CONE_FIELDS)]

    def on_lidar(self, msg):
        if self.as_state != 3:
            return
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.now()
        if self._too_old(now, st):
            return
        ok = self.latency_comp
        cones = self.stack.detect(now, unpack(msg), self.camera, stamp=st if ok else None,
                                  camera_stamp=self.camera_stamp if ok else None)
        if self._too_old(self.now(), st):                   # processing took too long: the result is stale
            return
        self._cones_buf.append((cones, st if ok else None))
        self._health.append((now - st, self.now() - now))   # scan age on arrival, processing time

    def _too_old(self, now: float, stamp: float) -> bool:
        """A stamped scan older than perception.max_scan_age_s (or from the future) is dropped and counted.
        Used as if it were new, an old scan matches the wrong cones on a symmetric layout."""
        if not self.latency_comp:
            return False
        if -0.05 <= now - stamp <= self.max_age:
            return False
        self._dropped += 1
        return True

    def drain_cones(self):
        while self._cones_buf:
            cones, st = self._cones_buf.popleft()
            if st is None or not self._too_old(self.now(), st):
                self.stack.on_cones(self.now(), cones, stamp=st)

    def health(self):
        """Once a second, one console line a team can read on the car: is the stack keeping up?"""
        if self.as_state != 3:
            return
        h, self._health = self._health, []
        gap, self._gap = self._gap, {"imu": 0.0, "tick": 0.0}
        st = self.stack.slam.stats
        d = {k: st[k] - self._stats_prev.get(k, 0) for k in ("matched", "new", "rejected")}
        self._stats_prev = dict(st)
        age = [a for a, _ in h] or [0.0]
        proc = [p for _, p in h] or [0.0]
        x, y, th = self.stack.slam.pose
        self.get_logger().info(
            f"health: scans {len(h)}/s (dropped as too old: {self._dropped} in total), scan age avg {1e3 * sum(age) / len(age):.0f} max {1e3 * max(age):.0f} ms, "
            f"processing avg {1e3 * sum(proc) / len(proc):.0f} max {1e3 * max(proc):.0f} ms, matched {d['matched']} "
            f"new {d['new']} rejected {d['rejected']}, map {self.stack.slam.n}, pose ({x:.2f}, {y:.2f}, "
            f"{math.degrees(th):.1f} deg), slip {float(self.stack.slam.mu[4]):.4f}, grip {self.stack.grip_ratio:.2f}, "
            f"steer {self.stack.steer_ratio:.2f}, longest gap: imu {1e3 * min(gap['imu'], 9.99):.0f} ms, "
            f"control {1e3 * min(gap['tick'], 9.99):.0f} ms")

    def on_cones(self, msg):
        if self.as_state != 3:
            return
        cones = [Cone(float(x), float(y), int(round(c)), float(p), "external") for x, y, _, c, p in unpack(msg, CONE_FIELDS)]
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self._too_old(self.now(), st):
            self._cones_buf.append((cones, st if self.latency_comp else None))

    def tick(self):
        t = self.now()
        self._gap["tick"] = max(self._gap["tick"], t - self._last_cb["tick"])
        self._last_cb["tick"] = t
        self.drain_imu()
        self.drain_cones()
        if t - self.last_imu_ts > 0.1:
            # IMU late: bridge the gap with the yaw rate the steering should give (not the last gyro
            # value, which in a corner entry was 14 deg off after 0.4 s in the real-time runs)
            self.stack.on_odometry(t, self.speed, self.stack.yaw_rate_guess(self.speed), observe=False)
        out = self.stack.step(t, self.as_state, self.ami, steer_max=self.steer_max)
        if out.cmd is not None:
            m = AckermannDriveStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.drive.speed = float(out.cmd.speed_mps or 0.0)
            m.drive.steering_angle = math.radians(out.cmd.steer_deg)
            m.drive.acceleration = -float(out.cmd.decel_mps2) if out.cmd.decel_mps2 else float(out.cmd.accel_mps2 or 0.0)
            self.pub_cmd.publish(m)
        self.pub_log.publish(UInt16MultiArray(data=[out.lap, out.cones_actual, out.cones_all]))
        if out.finish and not self.finish_called and self.cli_finish.service_is_ready():
            self.cli_finish.call_async(Trigger.Request())
            self.finish_called = True
            self.get_logger().info("mission complete: asked the bridge to finish")
        if out.estop and not self.ebs_called and self.cli_ebs.service_is_ready():
            self.cli_ebs.call_async(Trigger.Request())
            self.ebs_called = True
            self.get_logger().error("EBS requested: " + "; ".join(out.status.get("alerts", [])[-1:]))
        while self.n_alerts < len(self.stack.alerts):          # every alert also goes to the console log
            self.get_logger().warn("stack: " + self.stack.alerts[self.n_alerts])
            self.n_alerts += 1
        if out.lap != self.last_lap:
            self.get_logger().info(f"lap {out.lap} (phase {self.stack.phase}, lap times {self.stack.lap_times})")
            self.last_lap = out.lap
        if int(t * 100) % 10 == 0:
            self.pub_status.publish(String(data=json.dumps(out.status, default=float)))

    def publish_viz(self):
        h = Header(frame_id="map")
        h.stamp = self.get_clock().now().to_msg()
        cm = self.stack.slam.cone_map(confirmed_only=False)
        arr = np.column_stack([cm[:, 0], cm[:, 1], np.zeros(len(cm)), cm[:, 2], np.ones(len(cm))]) if len(cm) else np.zeros((0, 5))
        self.pub_map.publish(pack(h, arr, CONE_FIELDS))
        p = Path(header=h)
        for x, y in self.stack.follow.path[::4]:
            ps = PoseStamped(header=h)
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            p.poses.append(ps)
        self.pub_path.publish(p)


def main():
    rclpy.init()
    node = AutonomyNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
