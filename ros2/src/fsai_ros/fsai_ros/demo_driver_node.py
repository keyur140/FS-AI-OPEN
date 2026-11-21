# fsai-open: ROS 2 node - scripted Acceleration mission (plays both the marshal and the team software)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: a pretend marshal flips the car's switches, a pretend team program drives 75 m in a
straight line and stops. If everything works the run ends in AS_FINISHED with no faults.

Only the marshal part touches the emulator. The driving part uses exactly the topics and services
real team software would use (/fsai/cmd, /fsai/mission_finished, /fsai/speed, /fsai/state).
Writes a JSON verdict to the `result` parameter path and exits 0 on success, 1 on failure.
"""
import json
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class DemoDriver(Node):
    def __init__(self):
        super().__init__("fsai_demo_driver")
        self.distance_goal = self.declare_parameter("distance_m", 75.0).value
        self.speed_goal = self.declare_parameter("speed_mps", 10.0).value
        self.result_path = self.declare_parameter("result", "").value
        self.pub_cmd = self.create_publisher(AckermannDriveStamped, "/fsai/cmd", 10)
        self.pub_switch = self.create_publisher(String, "/fsai_emulator/inputs", 10)
        self.finish = self.create_client(Trigger, "/fsai/mission_finished")
        self.create_subscription(Float32, "/fsai/speed", self.on_speed, 10)
        self.create_subscription(String, "/fsai/state", self.on_state, 10)
        self.create_subscription(String, "/fsai/status", self.on_status, 10)
        self.state = "?"
        self.status = {}
        self.speed = 0.0
        self.distance = 0.0
        self.last = time.monotonic()
        self.t0 = time.monotonic()
        self.phase = "arm"
        self.finish_sent = False
        self.max_speed = 0.0
        self.timeline = []
        self.done = None
        self.create_timer(0.02, self.tick)

    def on_speed(self, msg):
        now = time.monotonic()
        self.distance += self.speed * (now - self.last)
        self.last = now
        self.speed = msg.data
        self.max_speed = max(self.max_speed, self.speed)

    def on_state(self, msg):
        if msg.data != self.state:
            self.timeline.append((round(time.monotonic() - self.t0, 2), msg.data))
            self.get_logger().info(f"car state -> {msg.data}")
        self.state = msg.data

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def switches(self, **kw):
        self.pub_switch.publish(String(data=json.dumps(kw)))

    def drive(self, speed, steer=0.0):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed = float(speed)
        m.drive.steering_angle = float(steer)
        self.pub_cmd.publish(m)

    def tick(self):
        el = time.monotonic() - self.t0
        if self.done is not None:
            return
        if el > 60:
            return self.end(False, "timeout")
        if self.status.get("faults"):
            return self.end(False, f"fault: {self.status['faults']}")
        if self.phase == "arm" and el > 2.0:
            self.switches(asms=True, tsms=True, ami=1, ebs_armed=True, sdc_closed=True, res_go=False)
            self.phase = "wait_ready"
        elif self.phase == "wait_ready" and self.state == "AS_READY":
            self.ready_at = el
            self.phase = "wait_5s"
        elif self.phase == "wait_5s" and el - self.ready_at > 5.5:
            self.switches(res_go=True)
            self.phase = "wait_driving"
        elif self.phase in ("wait_driving", "drive"):
            if self.state == "AS_DRIVING":
                self.phase = "drive"
            if self.phase == "drive" and self.distance >= self.distance_goal and not self.finish_sent:
                self.finish.call_async(Trigger.Request())
                self.finish_sent = True
                self.get_logger().info(f"{self.distance:.1f} m reached: mission finished, braking")
            self.drive(self.speed_goal)
            if self.state == "AS_FINISHED":
                return self.end(True, "AS_FINISHED with no faults")
        elif self.phase == "wait_ready":
            self.drive(0.0)

    def end(self, ok, why):
        self.done = ok
        verdict = {"ok": ok, "why": why, "distance_m": round(self.distance, 2),
                   "max_speed_mps": round(self.max_speed, 2), "timeline": self.timeline,
                   "final_status": self.status}
        (self.get_logger().info if ok else self.get_logger().error)(json.dumps(verdict)[:500])
        if self.result_path:
            with open(self.result_path, "w") as f:
                json.dump(verdict, f, indent=1, default=str)


def main():
    rclpy.init()
    node = DemoDriver()
    while rclpy.ok() and node.done is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    ok = bool(node.done)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)
