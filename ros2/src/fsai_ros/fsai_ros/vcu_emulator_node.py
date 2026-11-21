# fsai-open: ROS 2 node - the software copy of the ADS-DV VCU, for desk testing without the car
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plays the car on the CAN bus. A person "flips the switches" through one topic:

  ros2 topic pub --once /fsai_emulator/inputs std_msgs/String "{data: '{\\"asms\\": true, \\"tsms\\": true, \\"ami\\": 1}'}"
  ros2 topic pub --once /fsai_emulator/inputs std_msgs/String "{data: '{\\"res_go\\": true}'}"
  ros2 topic pub --once /fsai_emulator/inputs std_msgs/String "{data: '{\\"sdc_closed\\": false}'}"   # e-stop

Keys: lvms, tsms, asms, ami (0-7), ebs_armed, sdc_closed, res_go, power_cycle (true = power cycle).
Publishes /fsai_emulator/truth (JSON: state, speed, position, heading, yaw rate, events) at 50 Hz.
"""
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fsai.bridge.runner import LoopStats, emulator_step_fn, open_bus, periodic
from fsai.params import DEFAULT_FILE, load
from fsai.vcu.emulator import VcuEmulator


class VcuEmulatorNode(Node):
    def __init__(self):
        super().__init__("fsai_vcu_emulator")
        bus_name = self.declare_parameter("bus", "sim").value
        cfg = self.declare_parameter("config", str(DEFAULT_FILE)).value
        self.vcu = VcuEmulator(load(cfg))
        self.vcu.lifted = bool(self.declare_parameter("lifted", False).value)   # static inspection: car raised
        self.lock = threading.Lock()
        self.bus = open_bus(bus_name)
        self.get_logger().warn(f"fsai-open VCU EMULATOR on bus '{bus_name}' - never run this on the real car's bus")
        self.create_subscription(String, "/fsai_emulator/inputs", self.on_inputs, 10)
        self.pub = self.create_publisher(String, "/fsai_emulator/truth", 10)
        self.create_timer(0.02, self.publish_truth)
        # desk runs: the pretend IMU lives here, in a light process of its own, like a real IMU driver.
        # In the busy world node (LiDAR rendering) its messages were held up to 0.4 s.
        if bool(self.declare_parameter("imu", False).value):
            import numpy as np
            from sensor_msgs.msg import Imu
            from fsai.autonomy import config as acfg
            from fsai.autonomy.sensors import Imu as ImuModel
            self.imu_model = ImuModel(acfg.load(), np.random.default_rng([int(self.declare_parameter("seed", 0).value), 3]))
            self.pub_imu = self.create_publisher(Imu, "/imu/data", 20)
            self._Imu = Imu
            self.create_timer(0.01, self.publish_imu)
        self.stop = threading.Event()
        self.stats = LoopStats()
        step = emulator_step_fn(self.vcu, self.bus)

        def locked_step(now):
            with self.lock:
                step(now)
        self.loop = threading.Thread(target=periodic, args=(locked_step, self.stop, 0.01, self.stats), daemon=True)
        self.loop.start()

    def on_inputs(self, msg: String):
        try:
            vals = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"bad JSON on /fsai_emulator/inputs: {exc}")
            return
        with self.lock:
            if vals.pop("power_cycle", False):
                self.vcu.power_cycle()
            for k, v in vals.items():
                if not hasattr(self.vcu.inputs, k):
                    self.get_logger().error(f"unknown input '{k}'")
                    continue
                setattr(self.vcu.inputs, k, type(getattr(self.vcu.inputs, k))(v))
        self.get_logger().info(f"car switches: {vars(self.vcu.inputs)}")

    def publish_imu(self):
        with self.lock:
            r, st = self.vcu.r, self.vcu.state
        m = self._Imu()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.angular_velocity.z = float(self.imu_model.yaw_rate(r)) if st == 3 else float(self.imu_model.yaw_rate(0.0))
        self.pub_imu.publish(m)

    def publish_truth(self):
        with self.lock:
            v = self.vcu
            data = {"state": v.state, "speed_mps": round(v.v, 3), "x": round(v.x, 4), "y": round(v.y, 4),
                    "yaw": round(v.yaw, 5), "yaw_rate": round(v.r, 5), "t": round(v.t, 3),
                    "steer_deg": round(v.steer, 2), "faults": vars(v.faults), "events": v.events[-5:],
                    "inputs": vars(v.inputs), "loop": self.stats.summary(),
                    "stamp_ns": self.get_clock().now().nanoseconds}   # when this state was true
        self.pub.publish(String(data=json.dumps(data)))

    def destroy_node(self):
        self.stop.set()
        self.loop.join(timeout=1)
        self.bus.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = VcuEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
