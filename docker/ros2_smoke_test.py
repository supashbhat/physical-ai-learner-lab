"""Exercise the real ROS 2 node and host bridge inside a ROS container."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ros2_node import PhysicalAILabNode
from sensor_msgs.msg import LaserScan


class BridgeSmokeCheck(Node):
    def __init__(self):
        super().__init__("physical_ai_lab_smoke_check")
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.latest_x: float | None = None
        self.scan_count = 0
        self.create_timer(0.2, self._send_command)

    def _send_command(self) -> None:
        message = Twist()
        message.linear.x = 0.22
        self.command_pub.publish(message)

    def _on_odom(self, message: Odometry) -> None:
        self.latest_x = float(message.pose.pose.position.x)

    def _on_scan(self, message: LaserScan) -> None:
        if len(message.ranges) != 181:
            raise AssertionError(f"expected 181 scan rays, received {len(message.ranges)}")
        if any(not math.isfinite(value) for value in message.ranges):
            raise AssertionError("scan contained a non-finite range")
        self.scan_count += 1


def main() -> None:
    rclpy.init()
    bridge = PhysicalAILabNode()
    check = BridgeSmokeCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(check)
    deadline = time.monotonic() + 6.0
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.remove_node(check)
        executor.remove_node(bridge)
        check.destroy_node()
        bridge.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    if check.latest_x is None or check.latest_x < 0.05:
        raise SystemExit(f"wheel odometry did not respond to /cmd_vel: x={check.latest_x}")
    if check.scan_count < 5:
        raise SystemExit(f"expected multiple /scan messages; received {check.scan_count}")
    print(
        "ROS 2 bridge smoke check passed: "
        f"wheel odometry x={check.latest_x:.3f} m, {check.scan_count} scans received"
    )


if __name__ == "__main__":
    main()
