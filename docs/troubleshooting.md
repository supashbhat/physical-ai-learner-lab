# Troubleshooting

This page starts with issues encountered while implementing and checking this repository. New items should include the symptom, reproduction command, root cause, and verified fix.

## The first model command drove backward

**Symptom:** the initial headless run moved slightly forward, then drove backward.

**Cause:** the wheel-joint sign in the inverse-kinematics mapping disagreed with the MuJoCo model's axis convention. A half-second smoke test passed because it only measured the startup transient.

**Fix:** changed the target wheel speed sign and extended the integration check to a five-second run with a one-metre travel band.

**Verify:**

```bash
./scripts/run_lab.sh --headless
pytest -q tests/test_control.py tests/test_smoke_sim.py
```

## Simulated time is a few ulps below the requested duration

**Symptom:** a test compared `sim.time >= 5.0` and failed at `4.999999999999671`.

**Cause:** binary floating-point representation of repeated 2 ms simulation steps.

**Fix:** compare with a small `1e-9` tolerance. The run duration is still checked to the nearest simulation step.

## The demo needs offscreen rendering

**Symptom:** a headless simulation passes, but generating the camera/video assets fails on systems without a working graphics context.

**Diagnosis:** use the simulation and unit tests separately from the optional renderer. Record the exact platform and renderer error before adding a workaround; do not claim the media pipeline is portable until tested on that machine.

## The depth contact sheet is dominated by nearby surfaces

**Symptom:** early frames showed the front bumper filling much of the depth image.

**Cause:** the camera origin was too close to the front bumper, so near-field geometry occupied most of the view.

**Fix:** moved the camera slightly forward and upward, regenerated the RGB/depth sequence, and inspected the complete contact sheet before publishing it.

## The depth renderer reports reduced precision

**Symptom:** MuJoCo reports that `ARB_clip_control` is unavailable and depth precision may be limited on the current macOS graphics backend.

**Response:** retain the raw renderer output for the tutorial, label the contact sheet's 3.0 m display clipping, and describe the archive as illustrative simulation data rather than sensor-grade measurements.

## Camera views swing down during acceleration

**Symptom:** parts of the RGB sequence showed mostly floor or chassis after a turn, even though the base-height smoke check passed.

**Cause:** the two driven wheels and rear skid allowed excessive pitch; checking chassis height alone did not detect the body rotation.

**Fix:** added a lightly lifted front skid as a pitch stop and a five-second tilt check. The scripted eight-second demo now stays within about 7.1° of level on the recorded Apple Silicon run.

## Reporting a new issue

Please include macOS version, chip architecture, Python version, command run, terminal output, and whether the headless test passes. Do not paste access tokens or personal filesystem contents into an issue.
