"""Run MuJoCo on the host and exchange bounded JSON messages with a ROS 2 client."""

from __future__ import annotations

import argparse
import hmac
import json
import math
import os
import socket
import time
from typing import Any

from .odometry import WheelOdometry
from .sensors import planar_scan
from .simulation import DifferentialDriveSimulation

PROTOCOL_VERSION = 1
MAX_MESSAGE_BYTES = 16_384
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 2.0
COMMAND_TIMEOUT_S = 0.5
SCAN_RAYS = 181
SCAN_FOV_RAD = math.pi
SCAN_MAX_RANGE_M = 4.0


class BridgeProtocolError(ValueError):
    """Raised when a peer sends an invalid or unsafe bridge message."""


def parse_command(message: dict[str, Any]) -> tuple[float, float]:
    """Validate one velocity command and return linear and angular speed."""
    if message.get("type") != "cmd_vel":
        raise BridgeProtocolError("expected a cmd_vel message")
    linear = message.get("linear_x")
    angular = message.get("angular_z")
    if isinstance(linear, bool) or isinstance(angular, bool):
        raise BridgeProtocolError("velocity values must be numbers")
    try:
        linear = float(linear)
        angular = float(angular)
    except (TypeError, ValueError) as exc:
        raise BridgeProtocolError("velocity values must be numbers") from exc
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise BridgeProtocolError("velocity values must be finite")
    if abs(linear) > MAX_LINEAR_SPEED or abs(angular) > MAX_ANGULAR_SPEED:
        raise BridgeProtocolError("velocity command exceeds the lab safety limits")
    return linear, angular


def encode_message(message: dict[str, Any]) -> bytes:
    """Encode one newline-delimited JSON protocol message."""
    return (json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n").encode()


def _wheel_angle(sim: DifferentialDriveSimulation, joint_id: int) -> float:
    address = int(sim.model.jnt_qposadr[joint_id])
    return float(sim.data.qpos[address])


def _wheel_speed(sim: DifferentialDriveSimulation, joint_id: int) -> float:
    address = int(sim.model.jnt_dofadr[joint_id])
    return float(sim.data.qvel[address])


def _state_message(
    sim: DifferentialDriveSimulation,
    odometry: WheelOdometry,
    linear_command: float,
    angular_command: float,
) -> dict[str, Any]:
    pose = sim.pose()
    wheel_linear = (
        odometry.geometry.wheel_radius
        * (_wheel_speed(sim, sim.left_joint) + _wheel_speed(sim, sim.right_joint))
        / 2.0
    )
    wheel_angular = (
        odometry.geometry.wheel_radius
        * (_wheel_speed(sim, sim.right_joint) - _wheel_speed(sim, sim.left_joint))
        / odometry.geometry.wheel_track
    )
    angles, ranges = planar_scan(
        sim.model,
        sim.data,
        sim.base_body,
        rays=SCAN_RAYS,
        field_of_view=SCAN_FOV_RAD,
        max_range=SCAN_MAX_RANGE_M,
    )
    return {
        "type": "state",
        "protocol_version": PROTOCOL_VERSION,
        "sim_time_s": sim.time,
        "odometry": {
            "x": odometry.x,
            "y": odometry.y,
            "yaw": odometry.yaw,
            "linear_x": wheel_linear,
            "angular_z": wheel_angular,
        },
        "ground_truth": {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
        "scan": {
            "angle_min": float(angles[0]),
            "angle_max": float(angles[-1]),
            "angle_increment": float(angles[1] - angles[0]),
            "range_min": 0.05,
            "range_max": SCAN_MAX_RANGE_M,
            "ranges": ranges.tolist(),
        },
        "command": {"linear_x": linear_command, "angular_z": angular_command},
    }


def serve(host: str, port: int, token: str, state_rate_hz: float = 10.0) -> None:
    """Accept authenticated ROS 2 clients and run the physics in real time."""
    if len(token) < 32:
        raise ValueError("bridge token must contain at least 32 characters")
    if state_rate_hz <= 0 or state_rate_hz > 30:
        raise ValueError("state rate must be in the interval (0, 30] Hz")

    sim = DifferentialDriveSimulation()
    odometry = WheelOdometry()
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, port))
    listener.listen(1)
    listener.settimeout(0.5)
    print(f"MuJoCo bridge listening on {host}:{port}", flush=True)

    try:
        while True:
            try:
                client, address = listener.accept()
            except TimeoutError:
                continue
            print(f"ROS 2 adapter connected from {address[0]}", flush=True)
            try:
                _serve_client(client, sim, odometry, token, state_rate_hz)
            except (ConnectionError, OSError, BridgeProtocolError, json.JSONDecodeError) as exc:
                print(f"Bridge client closed: {exc}", flush=True)
            finally:
                client.close()
    except KeyboardInterrupt:
        print("Stopping MuJoCo bridge.", flush=True)
    finally:
        listener.close()


def _serve_client(
    client: socket.socket,
    sim: DifferentialDriveSimulation,
    odometry: WheelOdometry,
    token: str,
    state_rate_hz: float,
) -> None:
    client.setblocking(False)
    incoming = bytearray()
    authenticated = False
    command = (0.0, 0.0)
    last_command_time = 0.0
    next_step = time.monotonic()
    next_publish = next_step
    step_period = sim.timestep
    publish_period = 1.0 / state_rate_hz

    while True:
        try:
            chunk = client.recv(4096)
            if chunk == b"":
                return
            incoming.extend(chunk)
        except BlockingIOError:
            pass

        if len(incoming) > MAX_MESSAGE_BYTES and b"\n" not in incoming:
            raise BridgeProtocolError("incoming message is too large")
        while b"\n" in incoming:
            raw_line, _, remainder = incoming.partition(b"\n")
            incoming = bytearray(remainder)
            if len(raw_line) > MAX_MESSAGE_BYTES:
                raise BridgeProtocolError("incoming message is too large")
            message = json.loads(raw_line)
            if not isinstance(message, dict):
                raise BridgeProtocolError("messages must be JSON objects")
            if not authenticated:
                supplied_token = message.get("token", "")
                valid_hello = (
                    message.get("type") == "hello"
                    and message.get("protocol_version") == PROTOCOL_VERSION
                    and isinstance(supplied_token, str)
                    and hmac.compare_digest(supplied_token, token)
                )
                if not valid_hello:
                    raise BridgeProtocolError("authentication or protocol version failed")
                client.sendall(
                    encode_message(
                        {
                            "type": "hello_ack",
                            "protocol_version": PROTOCOL_VERSION,
                        }
                    )
                )
                authenticated = True
            else:
                command = parse_command(message)
                last_command_time = time.monotonic()

        now = time.monotonic()
        if now - last_command_time > COMMAND_TIMEOUT_S:
            command = (0.0, 0.0)

        steps = 0
        while now >= next_step and steps < 50:
            sim.step(*command, dt=step_period)
            odometry.update(
                _wheel_angle(sim, sim.left_joint),
                _wheel_angle(sim, sim.right_joint),
            )
            next_step += step_period
            steps += 1
        if steps == 50 and now >= next_step:
            next_step = now + step_period

        if authenticated and now >= next_publish:
            client.sendall(encode_message(_state_message(sim, odometry, *command)))
            next_publish += publish_period
            if now - next_publish > publish_period:
                next_publish = now + publish_period
        time.sleep(min(0.001, max(0.0, next_step - time.monotonic())))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="listen interface")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--token",
        default=os.environ.get("PHYSICAL_AI_BRIDGE_TOKEN"),
        help="shared random token; defaults to PHYSICAL_AI_BRIDGE_TOKEN",
    )
    parser.add_argument("--state-rate", type=float, default=10.0, help="published state rate in Hz")
    args = parser.parse_args()
    if not args.token:
        parser.error("set PHYSICAL_AI_BRIDGE_TOKEN or pass --token")
    try:
        serve(args.host, args.port, args.token, args.state_rate)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
