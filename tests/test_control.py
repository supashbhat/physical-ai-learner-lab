from math import isclose

from physical_ai_lab.control import DriveGeometry, WheelVelocityController


def test_straight_line_maps_to_equal_wheel_speeds() -> None:
    controller = WheelVelocityController()
    left, right = controller.target_wheel_speeds(0.22, 0.0)
    assert isclose(left, right)
    assert left > 0


def test_positive_yaw_makes_left_wheel_slower() -> None:
    controller = WheelVelocityController()
    left, right = controller.target_wheel_speeds(0.2, 0.7)
    assert abs(left) < abs(right)


def test_wheel_torque_is_saturated() -> None:
    controller = WheelVelocityController(max_torque=0.4)
    left, right = controller.torques(1.0, 0.0, 0.0, 0.0)
    assert abs(left) <= 0.4
    assert abs(right) <= 0.4


def test_invalid_geometry_is_rejected() -> None:
    try:
        DriveGeometry(wheel_radius=0.0)
    except ValueError as exc:
        assert "positive" in str(exc)
    else:
        raise AssertionError("invalid wheel geometry should be rejected")
