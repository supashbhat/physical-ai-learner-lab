"""Translate the lab's authenticated TCP stream into standard ROS 2 messages."""

from __future__ import annotations

import json
import math
import os
import socket
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PROTOCOL_VERSION = 1
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 2.0


class HostLink:
    """Reconnectable newline-delimited JSON client for the native simulator."""

    def __init__(self, host: str, port: int, token: str, logger: Any):
        self.host = host
        self.port = port
        self.token = token
        self.logger = logger
        self.sock: socket.socket | None = None
        self.buffer = bytearray()
        self.outgoing = bytearray()
        self.next_retry = 0.0
        self.warned = False

    def _connect(self) -> bool:
        now = time.monotonic()
        if now < self.next_retry:
            return False
        self.next_retry = now + 2.0
        try:
            sock = socket.create_connection((self.host, self.port), timeout=5.0)
            sock.settimeout(5.0)
            hello = {
                "type": "hello",
                "protocol_version": PROTOCOL_VERSION,
                "token": self.token,
            }
            sock.sendall((json.dumps(hello, separators=(",", ":")) + "\n").encode())
            response = bytearray()
            while b"\n" not in response:
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError("host closed during bridge authentication")
                response.extend(chunk)
                if len(response) > 16_384:
                    raise ConnectionError("authentication response is too large")
            line, _, remainder = response.partition(b"\n")
            reply = json.loads(line)
            if (
                reply.get("type") != "hello_ack"
                or reply.get("protocol_version") != PROTOCOL_VERSION
            ):
                raise ConnectionError("host rejected the bridge protocol")
            self.buffer.extend(remainder)
            sock.setblocking(False)
            self.sock = sock
            self.outgoing.clear()
            self.warned = False
            self.logger.info(f"Connected to MuJoCo host at {self.host}:{self.port}")
            return True
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.logger.warning(f"Waiting for MuJoCo host bridge: {exc}")
            return False

    def poll(self, linear: float, angular: float) -> dict[str, Any] | None:
        if self.sock is None and not self._connect():
            return None
        assert self.sock is not None
        command = {
            "type": "cmd_vel",
            "linear_x": linear,
            "angular_z": angular,
        }
        self.outgoing.extend(
            (json.dumps(command, separators=(",", ":"), allow_nan=False) + "\n").encode()
        )
        try:
            while self.outgoing:
                sent = self.sock.send(self.outgoing)
                del self.outgoing[:sent]
            while True:
                try:
                    chunk = self.sock.recv(65_536)
                except BlockingIOError:
                    break
                if not chunk:
                    raise ConnectionError("MuJoCo host bridge disconnected")
                self.buffer.extend(chunk)
            latest = None
            while b"\n" in self.buffer:
                line, _, remainder = self.buffer.partition(b"\n")
                self.buffer = bytearray(remainder)
                message = json.loads(line)
                if message.get("type") == "state":
                    latest = message
            return latest
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.logger.warning(f"MuJoCo bridge disconnected; reconnecting: {exc}")
            self.sock.close()
            self.sock = None
            self.buffer.clear()
            self.outgoing.clear()
            self.next_retry = time.monotonic() + 1.0
            return None

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None


class PhysicalAILabNode(Node):
    """Publish wheel odometry and simulated planar ranges; subscribe to cmd_vel."""

    def __init__(self):
        super().__init__("physical_ai_lab_bridge")
        host = os.environ.get("ROS_BRIDGE_HOST", "host.docker.internal")
        port = int(os.environ.get("ROS_BRIDGE_PORT", "8765"))
        token = os.environ.get("PHYSICAL_AI_BRIDGE_TOKEN", "")
        if len(token) < 32:
            raise RuntimeError("PHYSICAL_AI_BRIDGE_TOKEN must contain at least 32 characters")

        self.link = HostLink(host, port, token, self.get_logger())
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, 10)
        self.create_timer(0.05, self._publish_state)
        self.get_logger().info(
            "ROS 2 bridge ready: /cmd_vel -> MuJoCo, wheel odometry -> /odom, ray scan -> /scan"
        )

    def _on_command(self, message: Twist) -> None:
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().warning("Ignoring a non-finite velocity command")
            self.linear_command = 0.0
            self.angular_command = 0.0
            return
        self.linear_command = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, linear))
        self.angular_command = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular))

    def _publish_state(self) -> None:
        state = self.link.poll(self.linear_command, self.angular_command)
        if state is None:
            return
        now = self.get_clock().now().to_msg()
        odometry = state["odometry"]

        odom_message = Odometry()
        odom_message.header.stamp = now
        odom_message.header.frame_id = "odom"
        odom_message.child_frame_id = "base_link"
        odom_message.pose.pose.position.x = float(odometry["x"])
        odom_message.pose.pose.position.y = float(odometry["y"])
        odom_message.pose.pose.orientation.z = math.sin(float(odometry["yaw"]) / 2.0)
        odom_message.pose.pose.orientation.w = math.cos(float(odometry["yaw"]) / 2.0)
        odom_message.twist.twist.linear.x = float(odometry["linear_x"])
        odom_message.twist.twist.angular.z = float(odometry["angular_z"])
        odom_message.pose.covariance[0] = 0.01
        odom_message.pose.covariance[7] = 0.01
        odom_message.pose.covariance[35] = 0.03
        odom_message.twist.covariance[0] = 0.02
        odom_message.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom_message)

        scan_data = state["scan"]
        scan_message = LaserScan()
        scan_message.header.stamp = now
        scan_message.header.frame_id = "base_link"
        scan_message.angle_min = float(scan_data["angle_min"])
        scan_message.angle_max = float(scan_data["angle_max"])
        scan_message.angle_increment = float(scan_data["angle_increment"])
        scan_message.time_increment = 0.0
        scan_message.scan_time = 0.1
        scan_message.range_min = float(scan_data["range_min"])
        scan_message.range_max = float(scan_data["range_max"])
        scan_message.ranges = [float(value) for value in scan_data["ranges"]]
        self.scan_pub.publish(scan_message)

    def destroy_node(self) -> bool:
        self.link.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PhysicalAILabNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
