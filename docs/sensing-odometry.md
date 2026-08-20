# Sensors and odometry

The demo places three views side by side: a MuJoCo camera render, a planar range scan, and a top-down comparison of simulator truth with wheel-integrated odometry.

## Planar scan

`src/physical_ai_lab/sensors.py` casts 181 horizontal rays over a 180° forward field of view using MuJoCo's `mj_ray` query. A ray with no hit is clipped to the configured maximum range. The scan excludes the robot body so rays do not immediately intersect the chassis.

This is a simple simulated range scan, not a model of a particular LiDAR's noise, reflectivity, timing, or hardware interface.

## Wheel odometry

`WheelOdometry` integrates changes in the two wheel joint positions. For wheel increments `ΔθL` and `ΔθR`, it computes

<p><code>Δs = r(ΔθL + ΔθR) / 2</code> &nbsp; and &nbsp; <code>Δψ = r(ΔθR − ΔθL) / b</code></p>

The simulator pose and odometry estimate have separate state. This makes small slip and integration errors visible in the trajectory panel.

## Camera output

The RGB contact sheet is accompanied by a matched depth contact sheet and a compressed archive with 20 timestamps and floating-point depth arrays. See the [synthetic-data guide](synthetic-data.md) for dimensions and generation steps.

The front camera is attached to the robot body and rendered through MuJoCo's offscreen renderer. The synthetic frame contact sheet in `assets/synthetic_frames.png` contains RGB camera frames from one deterministic run. It is not labeled training data and is not used to train a model.
