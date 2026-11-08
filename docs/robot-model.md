# Build the robot model

The model lives in `robot/diffdrive.xml`. It is intentionally small enough to read in one sitting and uses only original geometry authored for this lab.

## Model parts

| Part | MuJoCo representation | What to inspect |
|---|---|---|
| Chassis | Free body with box geoms | Mass, shape, and initial height |
| Drive wheels | Two hinge joints with cylinder geoms | Joint axis, wheel radius, friction, and contact |
| Motors | Bounded motor actuators | Torque limits and their joint names |
| Front/rear skids | Low-friction sphere geoms | Rear glide and a lightly lifted front anti-tip support |
| Camera | Body-mounted camera | View direction and field of view |
| Scan origin | Named site | Location used by the ray-based planar scan |
| Room boundary | Four static box geoms | Contact surfaces and visual context in camera and scan output |
| Obstacles | Two independent rigid bodies | A box and cylinder used as visible landmarks |

The robot uses a free joint for its chassis and one hinge joint for each wheel. Both wheel axes are parallel to the world y axis. The joint sign convention is encoded in `WheelVelocityController.target_wheel_speeds` and covered by a test.

Four static wall segments bound the room. Colored markers sit flush on the inside faces so the front-camera sequence retains landmarks as the robot turns. They are part of the simulated scene and can appear in the ray scan. A rear skid supports the chassis; the front skid sits 1 cm above level ground and limits pitch during acceleration without carrying the normal load.

## Inspect and edit

```bash
open robot/diffdrive.xml
```

Try changing the wheel radius in the XML, then update `DriveGeometry` in `src/physical_ai_lab/control.py`. Keeping model and controller dimensions consistent is part of Exercise 1.

## Why this model is deliberately modest

The goal is to make the relationships between wheel motion, chassis motion, contact, and sensing easy to see. The rear sphere is a low-friction skid, not a realistic caster. The model is a teaching example, not a validated design for a physical robot.
