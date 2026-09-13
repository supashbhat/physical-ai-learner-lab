#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ ! -x .venv/bin/physical-ai-bridge-host ]]; then
  echo "Set up the Python lab first with ./scripts/setup_macos.sh." >&2
  exit 1
fi
if ! command -v docker >/dev/null 2>&1; then
  echo "Install and start Docker Desktop, then retry this command." >&2
  exit 1
fi
if ! docker info >/dev/null 2>&1; then
  echo "Docker is installed but its engine is not running. Start Docker Desktop and retry." >&2
  exit 1
fi

PHYSICAL_AI_BRIDGE_TOKEN="$("$PWD/.venv/bin/python" -c 'import secrets; print(secrets.token_urlsafe(32))')"
export PHYSICAL_AI_BRIDGE_TOKEN
BRIDGE_LOG="$(mktemp -t physical-ai-bridge)"
cleanup() {
  if [[ -n "${BRIDGE_PID:-}" ]]; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
  rm -f "$BRIDGE_LOG"
}
trap cleanup EXIT INT TERM

.venv/bin/physical-ai-bridge-host --host 0.0.0.0 --port 8765 >"$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

for _ in {1..50}; do
  if grep -q "listening on" "$BRIDGE_LOG"; then
    break
  fi
  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    cat "$BRIDGE_LOG" >&2
    exit 1
  fi
  sleep 0.1
done
if ! grep -q "listening on" "$BRIDGE_LOG"; then
  echo "The MuJoCo host bridge did not become ready." >&2
  cat "$BRIDGE_LOG" >&2
  exit 1
fi

echo "Starting the ROS 2 container. In another terminal, use:"
echo "  docker compose exec ros2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'"
echo "Press Ctrl-C here to stop the ROS 2 container and host simulator."
docker compose up --build --remove-orphans
