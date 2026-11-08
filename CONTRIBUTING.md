# Contributing

Thanks for taking a look. The lab is designed to be reproducible on macOS, and changes should keep the simulator, tutorial, and recorded results in agreement.

## Before opening a change

1. Set up the locked Python environment with ./scripts/setup_macos.sh.
2. Run pytest -q, ruff check src tests docker, ruff format --check src tests docker, and ./scripts/run_lab.sh --headless.
3. If you change model geometry or the renderer, regenerate the affected assets with .venv/bin/physical-ai-demo.
4. If you change the site, run mkdocs build --strict.
5. Update the relevant tutorial page and CHANGELOG.md.

For ROS 2 changes, run the container check in the ROS 2 bridge guide when Docker is available. Include platform, Python version, command, and relevant output when reporting a failure.

Do not include credentials, private machine paths, or personal data in issues or commits. Keep performance and compatibility claims tied to a recorded run or automated check.
