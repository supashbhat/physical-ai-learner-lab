#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ ! -x .venv/bin/physical-ai-viewer ]]; then
  echo "Environment not found. Run ./scripts/setup_macos.sh first." >&2
  exit 1
fi

if [[ "${1:-}" == "--headless" ]]; then
  shift
  .venv/bin/physical-ai-sim "$@"
else
  .venv/bin/physical-ai-viewer
fi
