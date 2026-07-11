"""Differential-drive command conversion and bounded wheel-speed control."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DriveGeometry:
    wheel_radius: float = 0.055
    wheel_track: float = 0.216

    def __post_init__(self) -> None:
        if self.wheel_radius <= 0 or self.wheel_track <= 0:
            raise ValueError("wheel radius and track must be positive")


@dataclass
class WheelVelocityController:
    """Map body velocity (m/s, rad/s) to saturated wheel torques.

    Positive wheel-joint velocity rolls the wheel toward +x in this model, so
    the inverse-kinematics mapping matches the model's axis convention.
    """

    geometry: DriveGeometry = DriveGeometry()
    kp: float = 0.85
    max_torque: float = 1.8

    def target_wheel_speeds(self, linear: float, angular: float) -> tuple[float, float]:
        half_track = self.geometry.wheel_track / 2.0
        left_linear = linear - angular * half_track
        right_linear = linear + angular * half_track
        radius = self.geometry.wheel_radius
        return left_linear / radius, right_linear / radius

    def torques(
        self,
        linear: float,
        angular: float,
        measured_left: float,
        measured_right: float,
    ) -> tuple[float, float]:
        target_left, target_right = self.target_wheel_speeds(linear, angular)
        left = self.kp * (target_left - measured_left)
        right = self.kp * (target_right - measured_right)
        return (
            max(-self.max_torque, min(self.max_torque, left)),
            max(-self.max_torque, min(self.max_torque, right)),
        )
