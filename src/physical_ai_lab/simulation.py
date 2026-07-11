"""MuJoCo model loading and deterministic headless stepping."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .control import WheelVelocityController

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = PROJECT_ROOT / "robot" / "diffdrive.xml"


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


class DifferentialDriveSimulation:
    """A reproducible MuJoCo run with wheel-speed feedback control."""

    def __init__(self, model_path: Path | str = DEFAULT_MODEL):
        self.model_path = Path(model_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.controller = WheelVelocityController()
        self.left_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint"
        )
        self.right_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint"
        )
        self.base_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        if min(self.left_joint, self.right_joint, self.base_body) < 0:
            raise ValueError("model is missing a required base or wheel joint")
        self._left_qvel = self._joint_dof_address(self.left_joint)
        self._right_qvel = self._joint_dof_address(self.right_joint)
        self._left_ctrl = 0
        self._right_ctrl = 1
        mujoco.mj_forward(self.model, self.data)
        self.initial_pose = self.pose()

    def _joint_dof_address(self, joint_id: int) -> int:
        return int(self.model.jnt_dofadr[joint_id])

    @property
    def time(self) -> float:
        return float(self.data.time)

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def pose(self) -> Pose:
        position = self.data.xpos[self.base_body]
        quaternion = self.data.xquat[self.base_body]
        w, x, y, z = quaternion
        yaw = float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
        return Pose(float(position[0]), float(position[1]), yaw)

    def step(self, linear: float, angular: float, dt: float | None = None) -> Pose:
        end_time = self.time + (dt if dt is not None else self.timestep)
        while self.time + 1e-12 < end_time:
            left, right = self.controller.torques(
                linear,
                angular,
                float(self.data.qvel[self._left_qvel]),
                float(self.data.qvel[self._right_qvel]),
            )
            self.data.ctrl[self._left_ctrl] = left
            self.data.ctrl[self._right_ctrl] = right
            mujoco.mj_step(self.model, self.data)
        return self.pose()

    def run_command(
        self, linear: float, angular: float, duration: float, sample_period: float = 0.05
    ) -> list[tuple[float, Pose]]:
        if duration < 0 or sample_period <= 0:
            raise ValueError("duration must be non-negative and sample period positive")
        samples: list[tuple[float, Pose]] = [(self.time, self.pose())]
        next_sample = self.time + sample_period
        end_time = self.time + duration
        while self.time + 1e-12 < end_time:
            self.step(linear, angular)
            if self.time + 1e-12 >= next_sample or self.time + 1e-12 >= end_time:
                samples.append((self.time, self.pose()))
                next_sample += sample_period
        return samples

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
