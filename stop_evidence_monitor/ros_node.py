"""ROS 2 observation-only adapter. Does not stop or command a robot."""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .core import StopEvidenceMonitor


class StopEvidenceNode(Node):
    def __init__(self):
        super().__init__("stop_evidence_monitor")
        self.monitor = StopEvidenceMonitor()
        self.last_status = None
        self.create_subscription(Bool, "/sem/stop_request", self.on_request, 10)
        self.create_subscription(Twist, "/sem/downstream_cmd", self.on_command, 10)
        self.create_subscription(Twist, "/sem/independent_motion", self.on_motion, 10)
        self.publisher = self.create_publisher(String, "/sem/status", 10)
        self.create_timer(0.1, self.on_tick)
        self.get_logger().info("Observation only. Do not wire synthetic test topics to motor control.")

    def on_request(self, msg):
        if msg.data:
            self.monitor.request_stop(time.monotonic())
            self.get_logger().info("New stop request observed (not issued by this node)")

    def on_command(self, msg):
        # Downstream of the ACTUAL command arbiter, not merely Nav2's intended cmd_vel.
        self.monitor.record_downstream_command(
            time.monotonic(), msg.linear.x, msg.angular.z)

    def on_motion(self, msg):
        # NOT /odom relabeled. Wire only after provenance and independence review.
        self.monitor.record_motion(
            time.monotonic(), msg.linear.x, msg.angular.z, "independent_motion")

    def on_tick(self):
        result = self.monitor.evaluate(time.monotonic())
        data = {"status": result.status.value, "reason": result.reason,
                "age_s": None if result.stop_age_s is None else round(result.stop_age_s, 3)}
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        self.publisher.publish(msg)
        if result.status.value != self.last_status:
            self.get_logger().info(msg.data)
            self.last_status = result.status.value


def main(args=None):
    rclpy.init(args=args)
    node = StopEvidenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
