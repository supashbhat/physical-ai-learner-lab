"""Render a reproducible camera, scan, and odometry demo from MuJoCo."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from pathlib import Path

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from matplotlib.patches import Circle, Rectangle
from PIL import Image

from .odometry import WheelOdometry
from .sensors import planar_scan
from .simulation import PROJECT_ROOT, DifferentialDriveSimulation

COLORS = {
    "bg": "#0b1016",
    "panel": "#111a23",
    "grid": "#263443",
    "text": "#e8edf2",
    "muted": "#9aa9b6",
    "teal": "#57e2c1",
    "amber": "#f3b35a",
    "coral": "#f0715b",
    "blue": "#79aaf5",
}


def command_at(time_s: float) -> tuple[float, float]:
    """Drive a gentle, obstacle-clear S-turn through the sensor scene."""
    if time_s < 2.4:
        return 0.24, 0.0
    if time_s < 3.4:
        return 0.0, -0.7
    if time_s < 5.8:
        return 0.24, 0.0
    if time_s < 6.8:
        return 0.0, 0.7
    if time_s < 8.0:
        return 0.20, 0.0
    return 0.0, 0.0


def _joint_angle(sim: DifferentialDriveSimulation, joint_id: int) -> float:
    return float(sim.data.qpos[sim.model.jnt_qposadr[joint_id]])


def _draw_frame(
    sim: DifferentialDriveSimulation,
    scene_renderer: mujoco.Renderer,
    sensor_renderer: mujoco.Renderer,
    overview_camera: mujoco.MjvCamera,
    *,
    truth_x: list[float],
    truth_y: list[float],
    odom_x: list[float],
    odom_y: list[float],
    scan_angles: np.ndarray,
    scan_ranges: np.ndarray,
    command: tuple[float, float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pose = sim.pose()
    overview_camera.lookat[:] = [
        pose.x + 0.55 * np.cos(pose.yaw),
        pose.y + 0.55 * np.sin(pose.yaw),
        0.10,
    ]
    scene_renderer.update_scene(sim.data, camera=overview_camera)
    scene_rgb = scene_renderer.render()
    sensor_renderer.update_scene(sim.data, camera="front_camera")
    sensor_rgb = sensor_renderer.render()

    fig = plt.figure(figsize=(12.8, 7.2), dpi=120, facecolor=COLORS["bg"])
    grid = fig.add_gridspec(2, 3, height_ratios=[1.1, 1.0], hspace=0.23, wspace=0.20)
    ax_camera = fig.add_subplot(grid[0, :2])
    ax_scan = fig.add_subplot(grid[0, 2], projection="polar")
    ax_map = fig.add_subplot(grid[1, :2])
    ax_info = fig.add_subplot(grid[1, 2])

    for ax in (ax_camera, ax_map, ax_info):
        ax.set_facecolor(COLORS["panel"])
    ax_camera.imshow(scene_rgb)
    ax_camera.set_title(
        "SIMULATION VIEW · ROBOT + ENVIRONMENT",
        color=COLORS["muted"],
        loc="left",
        fontsize=9,
        pad=8,
    )
    ax_camera.set_xticks([])
    ax_camera.set_yticks([])
    for spine in ax_camera.spines.values():
        spine.set_color(COLORS["grid"])

    ax_scan.set_facecolor(COLORS["panel"])
    world_angles = scan_angles + pose.yaw
    ax_scan.scatter(world_angles, scan_ranges, s=5, color=COLORS["amber"], alpha=0.9)
    ax_scan.set_ylim(0, 3.0)
    ax_scan.set_theta_zero_location("E")
    ax_scan.set_thetamin(-90)
    ax_scan.set_thetamax(90)
    ax_scan.grid(color=COLORS["grid"], linewidth=0.65)
    ax_scan.tick_params(colors=COLORS["muted"], labelsize=7)
    ax_scan.set_title("PLANAR RANGE SCAN", color=COLORS["muted"], fontsize=9, pad=9)

    ax_map.plot(truth_x, truth_y, color=COLORS["teal"], linewidth=2.2, label="simulated pose")
    ax_map.plot(
        odom_x, odom_y, color=COLORS["amber"], linewidth=1.5, linestyle="--", label="wheel odometry"
    )
    ax_map.add_patch(Rectangle((0.93, 0.40), 0.44, 0.16, color=COLORS["coral"], alpha=0.75))
    ax_map.add_patch(Circle((1.85, -0.42), 0.15, color=COLORS["blue"], alpha=0.8))
    ax_map.scatter(
        [pose.x],
        [pose.y],
        marker=(3, 0, np.degrees(pose.yaw) - 90),
        s=115,
        color=COLORS["teal"],
        edgecolor=COLORS["bg"],
        linewidth=0.7,
        zorder=5,
    )
    ax_map.set_xlim(-0.35, 2.35)
    ax_map.set_ylim(-0.95, 1.05)
    ax_map.set_aspect("equal", adjustable="box")
    ax_map.grid(color=COLORS["grid"], linewidth=0.6)
    ax_map.tick_params(colors=COLORS["muted"], labelsize=7)
    ax_map.set_xlabel("x · metres", color=COLORS["muted"], fontsize=8)
    ax_map.set_ylabel("y · metres", color=COLORS["muted"], fontsize=8)
    ax_map.set_title(
        "TOP-DOWN · TRUTH VS. ODOMETRY", color=COLORS["muted"], loc="left", fontsize=9, pad=8
    )
    ax_map.legend(loc="upper left", frameon=False, fontsize=7, labelcolor=COLORS["text"])
    for spine in ax_map.spines.values():
        spine.set_color(COLORS["grid"])

    ax_info.axis("off")
    ax_info.text(
        0.04,
        0.96,
        "MUJOCO RUN / 01",
        color=COLORS["teal"],
        fontsize=9,
        weight="bold",
        transform=ax_info.transAxes,
    )
    sensor_ax = ax_info.inset_axes([0.04, 0.51, 0.92, 0.39])
    sensor_ax.imshow(sensor_rgb)
    sensor_ax.set_xticks([])
    sensor_ax.set_yticks([])
    for spine in sensor_ax.spines.values():
        spine.set_color(COLORS["grid"])
    ax_info.text(
        0.04,
        0.45,
        "ONBOARD CAMERA · RGB",
        color=COLORS["muted"],
        fontsize=7,
        transform=ax_info.transAxes,
    )
    ax_info.text(
        0.04,
        0.34,
        f"{sim.time:05.2f} s",
        color=COLORS["text"],
        fontsize=10,
        family="monospace",
        transform=ax_info.transAxes,
    )
    ax_info.text(
        0.52,
        0.34,
        f"x {pose.x:+.2f} m",
        color=COLORS["text"],
        fontsize=9,
        family="monospace",
        transform=ax_info.transAxes,
    )
    ax_info.text(
        0.04,
        0.21,
        f"y {pose.y:+.2f} m",
        color=COLORS["text"],
        fontsize=9,
        family="monospace",
        transform=ax_info.transAxes,
    )
    ax_info.text(
        0.52,
        0.21,
        f"ψ {np.degrees(pose.yaw):+.0f}°",
        color=COLORS["text"],
        fontsize=9,
        family="monospace",
        transform=ax_info.transAxes,
    )
    ax_info.text(
        0.04,
        0.08,
        f"CMD  {command[0]:.2f} m/s   {command[1]:+.2f} rad/s",
        color=COLORS["amber"],
        fontsize=7.5,
        family="monospace",
        transform=ax_info.transAxes,
    )
    ax_info.text(
        0.04,
        0.015,
        f"MUJOCO {mujoco.__version__} · HEADLESS RENDER",
        color=COLORS["muted"],
        fontsize=6,
        transform=ax_info.transAxes,
    )

    fig.suptitle(
        "A ROBOT YOU CAN REPRODUCE",
        x=0.06,
        y=0.985,
        ha="left",
        color=COLORS["text"],
        fontsize=15,
        weight="bold",
    )
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())
    rgb = rgba[:, :, :3].copy()

    def crop_panel(axis: plt.Axes) -> np.ndarray:
        renderer = fig.canvas.get_renderer()
        box = axis.get_tightbbox(renderer).expanded(1.06, 1.08)
        height, width = rgb.shape[:2]
        left = max(0, int(box.x0))
        right = min(width, int(box.x1))
        top = max(0, height - int(box.y1))
        bottom = min(height, height - int(box.y0))
        return rgb[top:bottom, left:right].copy()

    lidar_panel = crop_panel(ax_scan)
    odometry_panel = crop_panel(ax_map)
    plt.close(fig)
    return rgb, lidar_panel, odometry_panel


def _fit_video_frame(frame: np.ndarray, size: tuple[int, int] = (960, 540)) -> np.ndarray:
    image = Image.fromarray(frame)
    image.thumbnail(size, Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", size, COLORS["bg"])
    canvas.paste(image, ((size[0] - image.width) // 2, (size[1] - image.height) // 2))
    return np.asarray(canvas)


def render_demo(output_dir: Path, fps: int = 15) -> tuple[Path, ...]:
    output_dir.mkdir(parents=True, exist_ok=True)
    video_path = output_dir / "demo_drive.mp4"
    lidar_video_path = output_dir / "demo_lidar.mp4"
    odometry_video_path = output_dir / "demo_odom.mp4"
    gif_path = output_dir / "demo.gif"
    contact_path = output_dir / "synthetic_frames.png"
    depth_contact_path = output_dir / "synthetic_depth_frames.png"
    dataset_path = output_dir / "synthetic_data.npz"
    sim = DifferentialDriveSimulation()
    odom = WheelOdometry()
    scene_renderer = mujoco.Renderer(sim.model, height=360, width=640)
    sensor_renderer = mujoco.Renderer(sim.model, height=180, width=320)
    overview_camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(overview_camera)
    overview_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    overview_camera.distance = 1.55
    overview_camera.azimuth = 135
    overview_camera.elevation = -23
    times = np.arange(0, 8.0, 1.0 / fps)
    truth_x: list[float] = []
    truth_y: list[float] = []
    odom_x: list[float] = []
    odom_y: list[float] = []
    scan_angles = np.linspace(-np.pi / 2, np.pi / 2, 181)
    scan_ranges = np.full(181, 4.0)
    gif_frames: list[Image.Image] = []
    contact_frames: list[tuple[float, np.ndarray, np.ndarray]] = []
    contact_indices = set(np.linspace(0, len(times) - 1, 20, dtype=int).tolist())

    with ExitStack() as stack:
        writers = [
            stack.enter_context(
                imageio.get_writer(
                    path,
                    fps=fps,
                    codec="libx264",
                    quality=8,
                    pixelformat="yuv420p",
                    macro_block_size=2,
                )
            )
            for path in (video_path, lidar_video_path, odometry_video_path)
        ]
        for frame_i, target_time in enumerate(times):
            command = command_at(float(target_time))
            while sim.time + 1e-12 < target_time:
                sim.step(*command)
            left_angle = _joint_angle(sim, sim.left_joint)
            right_angle = _joint_angle(sim, sim.right_joint)
            odom_pose = odom.update(left_angle, right_angle)
            pose = sim.pose()
            truth_x.append(pose.x)
            truth_y.append(pose.y)
            odom_x.append(odom_pose[0])
            odom_y.append(odom_pose[1])
            scan_angles, scan_ranges = planar_scan(sim.model, sim.data, sim.base_body)
            frame, lidar_panel, odometry_panel = _draw_frame(
                sim,
                scene_renderer,
                sensor_renderer,
                overview_camera,
                truth_x=truth_x,
                truth_y=truth_y,
                odom_x=odom_x,
                odom_y=odom_y,
                scan_angles=scan_angles,
                scan_ranges=scan_ranges,
                command=command,
            )
            writers[0].append_data(frame)
            writers[1].append_data(_fit_video_frame(lidar_panel))
            writers[2].append_data(_fit_video_frame(odometry_panel))
            if frame_i % max(1, fps // 6) == 0:
                small = Image.fromarray(frame).resize((960, 540), Image.Resampling.LANCZOS)
                gif_frames.append(small.quantize(colors=160, method=Image.Quantize.MEDIANCUT))
            if frame_i in contact_indices:
                sensor_renderer.update_scene(sim.data, camera="front_camera")
                rgb_image = sensor_renderer.render().copy()
                sensor_renderer.enable_depth_rendering()
                depth_image = sensor_renderer.render().copy()
                sensor_renderer.disable_depth_rendering()
                contact_frames.append((float(target_time), rgb_image, depth_image))

    gif_frames[0].save(
        gif_path,
        save_all=True,
        append_images=gif_frames[1:],
        duration=1000 / 6,
        loop=0,
        optimize=True,
    )

    fig, axes = plt.subplots(4, 5, figsize=(15, 8), facecolor=COLORS["bg"])
    for ax, (time_s, image, _) in zip(axes.flat, contact_frames):
        ax.imshow(image)
        ax.set_title(f"t = {time_s:.1f} s", color=COLORS["muted"], fontsize=8)
        ax.axis("off")
    fig.suptitle(
        "MUJOCO CAMERA FRAMES · DETERMINISTIC 8 s RUN",
        color=COLORS["text"],
        fontsize=15,
        weight="bold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(contact_path, dpi=160, facecolor=COLORS["bg"], bbox_inches="tight")
    plt.close(fig)

    depth_fig, depth_axes = plt.subplots(4, 5, figsize=(15, 8), facecolor=COLORS["bg"])
    for ax, (time_s, _, depth) in zip(depth_axes.flat, contact_frames):
        ax.imshow(depth, cmap="magma", vmin=0.02, vmax=3.0)
        ax.set_title(f"t = {time_s:.1f} s", color=COLORS["muted"], fontsize=8)
        ax.axis("off")
    depth_fig.suptitle(
        "MUJOCO CAMERA DEPTH · METRES · DISPLAY CLIPPED AT 3.0 m",
        color=COLORS["text"],
        fontsize=15,
        weight="bold",
    )
    depth_fig.tight_layout(rect=(0, 0, 1, 0.95))
    depth_fig.savefig(depth_contact_path, dpi=160, facecolor=COLORS["bg"], bbox_inches="tight")
    plt.close(depth_fig)

    np.savez_compressed(
        dataset_path,
        times_s=np.asarray([time_s for time_s, _, _ in contact_frames], dtype=np.float32),
        rgb=np.stack([rgb for _, rgb, _ in contact_frames]),
        depth_m=np.stack([depth for _, _, depth in contact_frames]).astype(np.float32),
    )
    scene_renderer.close()
    sensor_renderer.close()
    return (
        video_path,
        lidar_video_path,
        odometry_video_path,
        gif_path,
        contact_path,
        depth_contact_path,
        dataset_path,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "docs" / "assets")
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()
    if args.fps < 6 or args.fps > 30:
        parser.error("fps must be between 6 and 30")
    for path in render_demo(args.output_dir, fps=args.fps):
        print(path)


if __name__ == "__main__":
    main()
