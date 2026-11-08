"""Run a deterministic example trajectory and print its measured final pose."""

from __future__ import annotations

import argparse
import json

from .simulation import DifferentialDriveSimulation


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0, help="drive duration")
    parser.add_argument("--speed", type=float, default=0.28, help="forward speed in m/s")
    parser.add_argument("--turn-rate", type=float, default=0.0, help="yaw rate in rad/s")
    args = parser.parse_args()

    sim = DifferentialDriveSimulation()
    start = sim.pose()
    samples = sim.run_command(args.speed, args.turn_rate, args.seconds)
    end = sim.pose()
    print(
        json.dumps(
            {
                "model": sim.model_path.name,
                "sim_time_s": round(sim.time, 3),
                "start_pose_m_rad": {
                    "x": round(start.x, 4),
                    "y": round(start.y, 4),
                    "yaw": round(start.yaw, 4),
                },
                "final_pose_m_rad": {
                    "x": round(end.x, 4),
                    "y": round(end.y, 4),
                    "yaw": round(end.yaw, 4),
                },
                "sample_count": len(samples),
                "base_height_m": round(float(sim.data.xpos[sim.base_body, 2]), 4),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
