"""Wheel-integrated differential-drive odometry, separate from simulator truth."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, sin

from .control import DriveGeometry


@dataclass
class WheelOdometry:
    geometry: DriveGeometry = field(default_factory=DriveGeometry)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    previous_left: float = 0.0
    previous_right: float = 0.0

    def update(self, left_angle: float, right_angle: float) -> tuple[float, float, float]:
        left_delta = left_angle - self.previous_left
        right_delta = right_angle - self.previous_right
        self.previous_left = left_angle
        self.previous_right = right_angle

        left_distance = self.geometry.wheel_radius * left_delta
        right_distance = self.geometry.wheel_radius * right_delta
        travel = (left_distance + right_distance) / 2.0
        turn = (right_distance - left_distance) / self.geometry.wheel_track
        midpoint_yaw = self.yaw + turn / 2.0
        self.x += travel * cos(midpoint_yaw)
        self.y += travel * sin(midpoint_yaw)
        self.yaw = atan2(sin(self.yaw + turn), cos(self.yaw + turn))
        return self.x, self.y, self.yaw
