"""Camera and planar range scan helpers for the MuJoCo model."""

from __future__ import annotations

import math

import mujoco
import numpy as np


def planar_scan(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base_body: int,
    *,
    rays: int = 181,
    field_of_view: float = math.pi,
    max_range: float = 4.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Cast a horizontal fan of rays and return relative angles and ranges.

    This is a simple, documented 2D scan approximation built from MuJoCo's
    geometry ray query. A no-hit return is max_range, as in clipped sensors.
    The robot body is excluded so the scanner does not hit its own chassis.
    """
    if rays < 2 or field_of_view <= 0 or max_range <= 0:
        raise ValueError("rays must be >= 2 and scan dimensions must be positive")

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar_origin")
    if site_id < 0:
        raise ValueError("model must define a site named 'lidar_origin'")
    origin = np.array(data.site_xpos[site_id], dtype=np.float64)
    quat = data.xquat[base_body]
    qw, qx, qy, qz = quat
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    angles = np.linspace(-field_of_view / 2, field_of_view / 2, rays)
    ranges = np.full(rays, max_range, dtype=np.float64)
    geomid = np.zeros(1, dtype=np.int32)
    for i, relative_angle in enumerate(angles):
        world_angle = yaw + relative_angle
        direction = np.array([math.cos(world_angle), math.sin(world_angle), 0.0])
        distance = mujoco.mj_ray(model, data, origin, direction, None, True, base_body, geomid)
        if distance >= 0:
            ranges[i] = min(float(distance), max_range)
    return angles, ranges
