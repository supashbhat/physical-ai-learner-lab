"""Open a native MuJoCo window and drive a short repeatable maneuver."""

from __future__ import annotations

import time

import mujoco.viewer

from .demo import command_at
from .simulation import DifferentialDriveSimulation


def main() -> None:
    sim = DifferentialDriveSimulation()
    print("Physical AI Learner Lab is running. Close the MuJoCo window to stop.")
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running():
            cycle_time = sim.time % 8.0
            command = command_at(cycle_time)
            tick_start = time.monotonic()
            sim.step(*command, dt=0.01)
            viewer.sync()
            remaining = 0.01 - (time.monotonic() - tick_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
