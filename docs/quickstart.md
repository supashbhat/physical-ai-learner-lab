# Quickstart

This lab is tested on Apple Silicon macOS. It uses native MuJoCo for simulation and a Python virtual environment for repeatable dependencies. ROS 2 Humble runs in a separate Docker container and connects to the host simulator over an authenticated local TCP bridge.

## Prerequisites

- macOS 14 or later (Apple Silicon tested first)
- Python 3.12–3.13; the recorded run used Python 3.13.2
- About 2 GB of free disk space for the environment, build tools, and generated media
- No RTX GPU or robot hardware required
- Docker Desktop is optional for the MuJoCo lessons and required only for the ROS 2 module

## Install and run

From a terminal:

```bash
git clone https://github.com/supashbhat/physical-ai-learner-lab.git
cd physical-ai-learner-lab
./scripts/setup_macos.sh
./scripts/run_lab.sh
```

The native MuJoCo viewer opens and drives the scripted demonstration. Close the viewer window to stop.

For a headless run that prints the measured start and end pose:

```bash
./scripts/run_lab.sh --headless
```

## Verify the setup

```bash
source .venv/bin/activate
pytest -q
ruff check src tests docker
ruff format --check src tests docker
```

The smoke test advances the physics for five simulated seconds and checks that the robot travels roughly one metre while remaining upright. It uses a range, not exact floating-point equality, so small machine-to-machine numerical differences do not make the test brittle.

## Generate the demo assets

```bash
.venv/bin/physical-ai-demo
```

This renders three MP4 views, a looping GIF, and synchronized RGB/depth camera data from the same scripted MuJoCo run. Demo generation takes longer than the headless smoke test because it renders camera images and dashboard frames.

The renderer also writes focused LiDAR and odometry videos plus matched RGB/depth arrays. See [Synthetic RGB and depth](synthetic-data.md).

## Optional ROS 2 module

Install and start Docker Desktop, then run:

~~~bash
./scripts/run_ros2.sh
~~~

The container publishes wheel-integrated /odom and a simulated /scan; send commands to /cmd_vel from a second terminal. Follow the [ROS 2 bridge guide](ros2-bridge.md) for topic checks and the container smoke test.

## Expected initial result

The recorded Apple Silicon run travels 1.0262 m in 5.0 simulated seconds with a final base height of 0.1296 m. These values are reference observations, not universal constants; the acceptance band is defined in `tests/test_smoke_sim.py`.

!!! tip "If the viewer does not open"
    Run the headless command first. If it passes, check that macOS has not blocked the MuJoCo viewer; the simulator itself does not require a display for the tests.
