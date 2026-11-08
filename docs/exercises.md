# Exercises

Try each exercise before opening its answer. The quickstart and installed Python environment are assumed.

## 1. Predict a straight-line wheel speed

The default wheel radius is 0.055 m. For a forward command of 0.28 m/s and zero yaw rate, what target angular speed does each wheel receive?

<details><summary>Check your answer</summary>

Both wheels receive approximately `0.28 / 0.055 = 5.09 rad/s`. The sign convention is positive in this model. The command is a target; the motor torque limit and contact determine the actual trajectory.

</details>

## 2. Turn without translating ideally

Run `./scripts/run_lab.sh --headless --seconds 2 --speed 0 --turn-rate 0.8`. Which wheel target has the greater magnitude, and which direction should the body yaw?

<details><summary>Check your answer</summary>

The right wheel is faster. The body should turn counter-clockwise from above, so yaw is positive. Small deviations in the physical run are possible because the wheels and chassis interact through contact.

</details>

## 3. Find the obstacle in the scan

Generate the demo and inspect the scan panel. The rectangular obstacle is centered at `(1.15, 0.48)` m. At the initial pose, why does the nearest return appear above the centerline in the top-down view?

<details><summary>Check your answer</summary>

The obstacle is offset to positive y, so rays at positive relative angles reach it. The scan starts at the robot's named `lidar_origin` site and is expressed in the robot's heading frame.

</details>

## 4. Compare truth and odometry

In the generated demo, compare the teal and amber paths. What could make them diverge even if the odometry equations are correct?

<details><summary>Check your answer</summary>

Wheel slip, imperfect contact, actuator tracking error, and numerical integration all affect the simulated body motion. Odometry only observes the simulated wheel angles and does not receive the body's true world pose.

</details>

## 5. Change the track width

Increase `wheel_track` in `DriveGeometry`, rerun the in-place turn, and compare the yaw. What must also change in the XML for the model and controller to remain consistent?

<details><summary>Check your answer</summary>

The wheel bodies' y positions should be moved so their separation matches the new track width. Otherwise the controller's geometry model and the physical wheel layout describe different robots.

</details>
