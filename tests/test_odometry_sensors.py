import math

import mujoco

from physical_ai_lab.odometry import WheelOdometry
from physical_ai_lab.sensors import planar_scan
from physical_ai_lab.simulation import DifferentialDriveSimulation


def test_wheel_odometry_integrates_straight_motion() -> None:
    odom = WheelOdometry()
    x, y, yaw = odom.update(1.0, 1.0)
    assert math.isclose(x, 0.055, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


def test_wheel_odometry_turns_toward_faster_right_wheel() -> None:
    odom = WheelOdometry()
    _, _, yaw = odom.update(0.0, 1.0)
    assert yaw > 0


def test_planar_scan_has_expected_shape_and_detects_scene_props() -> None:
    sim = DifferentialDriveSimulation()
    mujoco.mj_forward(sim.model, sim.data)
    angles, ranges = planar_scan(sim.model, sim.data, sim.base_body)
    assert len(angles) == len(ranges) == 181
    assert ranges.min() < 2.0
    assert ranges.max() <= 4.0
