import json

import pytest

from physical_ai_lab.bridge_host import (
    BridgeProtocolError,
    _state_message,
    encode_message,
    parse_command,
)
from physical_ai_lab.odometry import WheelOdometry
from physical_ai_lab.simulation import DifferentialDriveSimulation


def test_protocol_accepts_a_bounded_velocity_command() -> None:
    assert parse_command({"type": "cmd_vel", "linear_x": 0.25, "angular_z": -0.8}) == (
        0.25,
        -0.8,
    )


@pytest.mark.parametrize(
    "message",
    [
        {"type": "other", "linear_x": 0.0, "angular_z": 0.0},
        {"type": "cmd_vel", "linear_x": float("nan"), "angular_z": 0.0},
        {"type": "cmd_vel", "linear_x": 0.0, "angular_z": float("inf")},
        {"type": "cmd_vel", "linear_x": True, "angular_z": 0.0},
        {"type": "cmd_vel", "linear_x": 0.0},
        {"type": "cmd_vel", "linear_x": 0.6, "angular_z": 0.0},
        {"type": "cmd_vel", "linear_x": 0.0, "angular_z": 2.1},
    ],
)
def test_protocol_rejects_invalid_or_unbounded_commands(message: dict[str, object]) -> None:
    with pytest.raises(BridgeProtocolError):
        parse_command(message)


def test_messages_are_newline_delimited_json() -> None:
    encoded = encode_message({"type": "hello_ack", "protocol_version": 1})
    assert encoded.endswith(b"\n")
    assert json.loads(encoded) == {"type": "hello_ack", "protocol_version": 1}


def test_host_state_has_odometry_and_ros_scan_fields() -> None:
    sim = DifferentialDriveSimulation()
    state = _state_message(sim, WheelOdometry(), 0.0, 0.0)
    assert state["type"] == "state"
    assert len(state["scan"]["ranges"]) == 181
    assert state["scan"]["angle_min"] < 0 < state["scan"]["angle_max"]
    assert state["scan"]["range_max"] == 4.0
    assert state["odometry"]["x"] == 0.0
