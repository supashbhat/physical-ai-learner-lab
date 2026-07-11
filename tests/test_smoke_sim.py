import math

from physical_ai_lab.simulation import DifferentialDriveSimulation


def test_simulation_loads_and_advances() -> None:
    sim = DifferentialDriveSimulation()
    start = sim.pose()
    samples = sim.run_command(linear=0.28, angular=0.0, duration=5.0)
    end = sim.pose()
    assert len(samples) >= 2
    assert sim.time >= 5.0 - 1e-9
    assert 0.9 < end.x - start.x < 1.15
    assert abs(end.y) < 0.08
    assert abs(end.yaw) < 0.08
    assert 0.08 < float(sim.data.xpos[sim.base_body, 2]) < 0.20


def test_robot_stays_near_level_during_wheel_drive() -> None:
    sim = DifferentialDriveSimulation()
    max_tilt = 0.0
    for _ in range(round(5.0 / sim.timestep)):
        sim.step(linear=0.28, angular=0.0)
        quaternion = sim.data.xquat[sim.base_body]
        body_up_world_z = 1.0 - 2.0 * (quaternion[1] ** 2 + quaternion[2] ** 2)
        tilt = math.acos(max(-1.0, min(1.0, float(body_up_world_z))))
        max_tilt = max(max_tilt, tilt)
    assert max_tilt < math.radians(12.0)
