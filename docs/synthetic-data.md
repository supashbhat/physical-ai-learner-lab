# Synthetic RGB and depth

The demo renderer records 20 evenly spaced camera observations from the same deterministic eight-second MuJoCo trajectory. Each observation includes an RGB image and a depth image from the robot's front camera.

## Generate the data

~~~bash
.venv/bin/physical-ai-demo
~~~

The renderer writes these files to docs/assets:

| File | Contents |
|---|---|
| synthetic_frames.png | Contact sheet of 20 RGB camera frames |
| synthetic_depth_frames.png | Matching depth views, color-mapped and displayed with a 3.0 m limit |
| synthetic_data.npz | times_s, rgb, and metric depth_m arrays |
| demo_drive.mp4 | Full simulation dashboard |
| demo_lidar.mp4 | Cropped range-scan view |
| demo_odom.mp4 | Cropped truth-versus-wheel-odometry plot |
| demo.gif | Short looping dashboard for the project overview |

Load the NumPy archive:

~~~python
import numpy as np

data = np.load("docs/assets/synthetic_data.npz")
print(data["times_s"].shape)  # (20,)
print(data["rgb"].shape)      # (20, 180, 320, 3)
print(data["depth_m"].shape)  # (20, 180, 320)
~~~

The PNG uses a magma color map and clips display values above 3.0 m so nearby geometry and the scene remain distinguishable. The archive preserves the floating-point depth values from MuJoCo's camera renderer. These frames are unlabelled simulation output; the repository does not train a model or claim that the images represent real sensor data.

[Download the generated archive](assets/synthetic_data.npz) (about 2.1 MB).

!!! warning "Depth precision"
    The current macOS graphics backend reports that ARB_clip_control is unavailable and depth accuracy is limited. The files are useful for understanding the rendering pipeline, but should not be treated as sensor-grade measurements.

## Try a data exercise

Compare RGB and depth around an obstacle. Identify the pixels where the range changes sharply, then alter an obstacle's position in robot/diffdrive.xml and regenerate the archive. Check that the timestamps and array dimensions remain stable.

The depth arrays can be inspected without a display:

~~~bash
.venv/bin/python -c 'import numpy as np; d=np.load("docs/assets/synthetic_data.npz"); print(d["depth_m"].min(), d["depth_m"].max())'
~~~
