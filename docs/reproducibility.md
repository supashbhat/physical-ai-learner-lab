# Reproducibility record

## Verified local environment

| Item | Recorded value |
|---|---|
| Host | Apple M2 Pro, 16 GB RAM |
| Operating system | macOS 26.5.1, Darwin 25.5.0 |
| Architecture | Apple Silicon, `arm64` |
| Python | 3.13.2 |
| MuJoCo | 3.3.7 |
| NumPy | 2.5.4 |
| Pytest | 9.2.0 |
| Headless simulation | 5.0 s simulated time; 1.0262 m forward; base height 0.1296 m |

The completed suite currently contains 20 tests. All 20 pass both in the development environment and in a fresh temporary copy set up with the published installation script and hash-locked requirements. The five-second headless run traveled 1.0262 m in both environments.

## Repeat the checks

```bash
./scripts/setup_macos.sh
source .venv/bin/activate
pytest -q
ruff check src tests docker
ruff format --check src tests docker
./scripts/run_lab.sh --headless
```

## Scope of the current claim

- Apple Silicon macOS is the tested local platform. The hosted macOS Intel job also passed; local Intel hardware was not available.
- The core headless MuJoCo tests do not require a graphics window.
- Camera rendering was exercised locally on Apple Silicon. The ROS 2 host protocol passes a local TCP integration test, and the hosted Linux ROS 2 topic check passed. Docker Desktop was unavailable on the development Mac.
- The ROS 2 base image is pinned to a multi-platform image digest so arm64 and amd64 use the same release.

## Continuous integration

The workflow runs the headless model, controller, odometry, sensor, protocol, lint, and tutorial-build checks on Apple Silicon and Intel macOS. A Linux job builds the pinned ROS 2 image and checks the bridge topics. See the [GitHub Actions history](https://github.com/supashbhat/physical-ai-learner-lab/actions) for the latest results. GitHub lists `macos-latest` as arm64 and `macos-15-intel` as Intel in its [hosted-runner reference](https://docs.github.com/en/actions/reference/runners/github-hosted-runners).
