# fsai-open: ROS 2 node - the ADS-DV CAN bridge (team software <-> car)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""ROS 2 interface (EUFS-style, so team code stays swappable):

  subscribes  /fsai/cmd             ackermann_msgs/AckermannDriveStamped  speed [m/s], steering_angle [rad, +left],
                                                                         acceleration [m/s^2] (>0 accel limit, <0 decel)
              /fsai/fsg_logging     std_msgs/UInt16MultiArray             [lap, cones_actual, cones_all]
  services    /fsai/mission_finished std_srvs/Trigger   brake to standstill, then report FINISHED
              /fsai/ebs              std_srvs/Trigger   request an emergency stop
  publishes   /fsai/state            std_msgs/String    AS state (AS_OFF, AS_READY, AS_DRIVING, ...)
              /fsai/status           std_msgs/String    full bridge status as JSON (faults, limits, notes)
              /fsai/wheel_speeds     std_msgs/Float32MultiArray  [FL, FR, RL, RR] rpm
              /fsai/speed            std_msgs/Float32   vehicle speed from wheel speeds [m/s]
              /diagnostics           diagnostic_msgs/DiagnosticArray (1 Hz)

The 10 ms CAN loop runs on its own thread with a monotonic clock, NOT on a ROS timer, so a busy
ROS executor can never starve the handshake (a 50 ms stall would e-stop the car, FS-AI 2026 rules T1.1.8).
Parameters: bus (sim | vcan0 | can0 | <interface>:<channel>), config (path to ads_dv.yaml).
"""
import json
import math
import threading

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String, UInt16MultiArray
from std_srvs.srv import Trigger

from fsai.bridge.core import BridgeCore, DriveCommand
from fsai.bridge.runner import LoopStats, bridge_step_fn, open_bus, periodic
from fsai.params import DEFAULT_FILE, load


class CanBridgeNode(Node):
    def __init__(self):
        super().__init__("fsai_can_bridge")
        bus_name = self.declare_parameter("bus", "sim").value
        cfg = self.declare_parameter("config", str(DEFAULT_FILE)).value
        self.core = BridgeCore(load(cfg))
        self.lock = threading.Lock()
        self.bus = open_bus(bus_name)
        self.get_logger().info(f"fsai-open CAN bridge on bus '{bus_name}', config {cfg}")

        self.create_subscription(AckermannDriveStamped, "/fsai/cmd", self.on_cmd, 10)
        self.create_subscription(UInt16MultiArray, "/fsai/fsg_logging", self.on_logging, 10)
        self.create_service(Trigger, "/fsai/mission_finished", self.on_finished)
        self.create_service(Trigger, "/fsai/ebs", self.on_ebs)
        self.pub_state = self.create_publisher(String, "/fsai/state", 10)
        self.pub_status = self.create_publisher(String, "/fsai/status", 10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, "/fsai/wheel_speeds", 10)
        self.pub_speed = self.create_publisher(Float32, "/fsai/speed", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(0.05, self.publish_status)
        self.create_timer(1.0, self.publish_diagnostics)

        self.stop = threading.Event()
        self.stats = LoopStats()
        step = bridge_step_fn(self.core, self.bus)

        def locked_step(now):
            with self.lock:
                step(now)
        self.loop = threading.Thread(target=periodic, args=(locked_step, self.stop, 0.01, self.stats), daemon=True)
        self.loop.start()

    # ---------------------------------------------------------------- inputs
    def _now(self):
        import time
        return time.monotonic()

    def on_cmd(self, msg: AckermannDriveStamped):
        d = msg.drive
        accel = d.acceleration if d.acceleration > 0 else None
        decel = -d.acceleration if d.acceleration < 0 else None
        cmd = DriveCommand(steer_deg=math.degrees(d.steering_angle), speed_mps=d.speed,
                           accel_mps2=accel, decel_mps2=decel)
        with self.lock:
            ok = self.core.set_command(cmd, self._now())
        if not ok:
            self.get_logger().warn("rejected a /fsai/cmd containing NaN or infinity")

    def on_logging(self, msg: UInt16MultiArray):
        vals = list(msg.data) + [None] * 3
        with self.lock:
            self.core.set_logging(lap=vals[0], cones_actual=vals[1], cones_all=vals[2])

    def on_finished(self, _req, res):
        with self.lock:
            self.core.request_finish()
        res.success, res.message = True, "finishing: braking to standstill, then MISSION_STATUS=FINISHED"
        return res

    def on_ebs(self, _req, res):
        with self.lock:
            self.core.request_estop()
        self.get_logger().error("EMERGENCY STOP requested by team software")
        res.success, res.message = True, "ESTOP_REQUEST sent"
        return res

    # ---------------------------------------------------------------- outputs
    def publish_status(self):
        with self.lock:
            st = dict(self.core.last_status)
            rpms = self.core.wheel_rpms() or [0.0] * 4
            speed = self.core.speed_mps()
        if not st:
            return
        st["loop"] = self.stats.summary()
        self.pub_state.publish(String(data=st["as_state"]))
        self.pub_status.publish(String(data=json.dumps(st, default=str)))
        self.pub_wheels.publish(Float32MultiArray(data=[float(r) for r in rpms]))
        self.pub_speed.publish(Float32(data=float(speed)))

    def publish_diagnostics(self):
        with self.lock:
            st = dict(self.core.last_status)
        if not st:
            return
        s = DiagnosticStatus(name="fsai_can_bridge", hardware_id="ADS-DV VCU")
        s.level = DiagnosticStatus.ERROR if st["faults"] else (
            DiagnosticStatus.WARN if st["notes"] else DiagnosticStatus.OK)
        s.message = ", ".join(st["faults"]) or st["as_state"]
        s.values = [KeyValue(key=k, value=str(v)) for k, v in st.items()]
        arr = DiagnosticArray(status=[s])
        arr.header.stamp = self.get_clock().now().to_msg()
        self.pub_diag.publish(arr)

    def destroy_node(self):
        self.stop.set()
        self.loop.join(timeout=1)
        self.bus.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = CanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
