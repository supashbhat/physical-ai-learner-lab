# Connect ROS 2

The lab keeps MuJoCo on macOS and runs ROS 2 Humble in a small Docker container. A newline-delimited TCP adapter connects the two processes; ROS 2 discovery stays inside the container, so Docker does not need host DDS multicast.

## Start both sides

Install and start Docker Desktop, then open a terminal in the repository:

~~~bash
./scripts/run_ros2.sh
~~~

The script creates a temporary random token, starts the native MuJoCo host bridge, builds the pinned ROS image once, and launches the ROS 2 node. The token is not written to the repository. Stop both processes with Ctrl-C in this terminal.

In a second terminal, send a forward command from inside the ROS container:

~~~bash
docker compose exec ros2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
~~~

Stop the publisher with Ctrl-C, then inspect the published state:

~~~bash
docker compose exec ros2 ros2 topic echo --once /odom
docker compose exec ros2 ros2 topic echo --once /scan
~~~

The simulator clips linear commands to ±0.5 m/s and yaw commands to ±2 rad/s. If the command stream stops, the host applies zero velocity after 0.5 seconds.

## Topics

| Topic | Direction | ROS message | Contents |
|---|---|---|---|
| /cmd_vel | ROS 2 → MuJoCo | geometry_msgs/msg/Twist | linear.x and angular.z are used |
| /odom | MuJoCo → ROS 2 | nav_msgs/msg/Odometry | Pose integrated from wheel-joint angles, plus wheel-derived velocity |
| /scan | MuJoCo → ROS 2 | sensor_msgs/msg/LaserScan | 181 simulated horizontal rays over a 180° field of view |

The /odom pose comes from wheel integration. Simulator ground truth is carried separately in the internal TCP state for comparison; it is not substituted for the odometry estimate.

The node attaches fixed illustrative covariance values to /odom; they have not been calibrated against a physical robot or a statistical noise model.

## Reproduce the container check

The smoke check runs a ROS publisher and subscribes to /odom and /scan inside ROS 2 Humble. It fails unless the command moves the simulated robot and multiple 181-ray scans arrive.

~~~bash
PHYSICAL_AI_BRIDGE_TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))')"
export PHYSICAL_AI_BRIDGE_TOKEN
.venv/bin/physical-ai-bridge-host --host 0.0.0.0 --port 8765 &
BRIDGE_PID=$!
docker compose --profile test run --rm ros2-smoke
kill "$BRIDGE_PID"
~~~

For normal use, prefer ./scripts/run_ros2.sh; it manages the token and cleans up the host process when stopped.

## Network and platform notes

- Docker Desktop provides host.docker.internal so the container can reach the host.
- The host listens on port 8765 and requires a fresh 256-bit random token for every launch.
- Keep the generated token private. The bridge binds to all host interfaces so Docker Desktop can reach it; the token is checked before any commands or sensor state are exchanged. The TCP transport is not encrypted, so use this development bridge only on a trusted machine/network and do not port-forward the port.
- The pinned official ROS image supports both linux/arm64 and linux/amd64; Docker selects the host architecture.
- This workflow does not launch RViz. ROS topics are visible through the ROS CLI inside the container; a desktop visualization is a future addition.

## If the container cannot connect

1. Confirm Docker Desktop is running with docker info.
2. Leave ./scripts/run_ros2.sh running while the container is active.
3. Check that port 8765 is not already in use.
4. Stop old project containers with docker compose down, then restart the script to create a new token.
