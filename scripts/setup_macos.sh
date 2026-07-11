#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PYTHON_BIN="${PYTHON:-python3}"
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "Python 3.12–3.13 is required. Set PYTHON=/path/to/python3.13 and retry." >&2
  exit 1
fi

"$PYTHON_BIN" - <<'PY'
import sys
if not (3, 12) <= sys.version_info[:2] <= (3, 13):
    raise SystemExit(f"Need Python 3.12–3.13; found {sys.version.split()[0]}")
print(f"Setting up with Python {sys.version.split()[0]}")
PY

if [[ ! -d .venv ]]; then
  "$PYTHON_BIN" -m venv .venv
fi

.venv/bin/python -m pip install --require-hashes -r requirements.lock
.venv/bin/python -m pip install --no-deps --no-build-isolation -e .
echo "Setup complete. Run ./scripts/run_lab.sh to open the simulator."
