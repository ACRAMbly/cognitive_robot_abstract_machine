# Scripted Tetris observations

The `scripted_tetris` analysis engine produces deterministic CAS object hypotheses
for the two supplied Tetris block models. Its single annotator plays six scenes:
both blocks visible, translation, rotation, the first block absent, both blocks
absent, and both blocks reappearing.

## Run

From the repository root, using the current project environment:

```bash
source /opt/ros/jazzy/setup.bash
source .venv-foundation-pose/ros_workspace/install/setup.bash
.venv-foundation-pose/bin/python -m robokudo.scripts.main _ae scripted_tetris _no3d
```

Add `_headless` to run without the GUI. The default demonstration repeats with
one second between observations. Stop the runner with Ctrl+C.

## Observation data

The [scenario descriptor](src/robokudo/descriptors/scripted_tetris_observations.py)
contains the editable poses and visibility sequence. The positions are illustrative
synthetic data. Model 1 uses `child_cube_0_colored.ply`; model 2 uses
`child_cube_2_colored.ply`, both from the checkout's `robokudo_cad_data_new` directory.

Each completed observation has a fresh CAS, capture timestamp, camera frame, and
`CameraInfo`. Each visible detection has:

- `Classification` with the model identifier, mesh stem, and declared confidence.
- A plain camera-relative `PoseAnnotation` in the original CAD frame, with metre
  translations and an `xyzw` quaternion.
- `BoundingBox3DAnnotation` dimensions derived from the actual mesh. Its center
  accounts for the offset between the CAD origin and the enclosing box center.
- A rectangular image region obtained by projecting the mesh vertices, with a
  cropped binary foreground mask.

Mesh vertices are scaled from millimetres to metres once, without recentering.
The color image and GUI output show a diagnostic drawing of regions and labels.
The masks are rectangles rather than object silhouettes. The engine ends at CAS
hypotheses; belief association and world synchronization are separate work.

## Playback configuration

The [analysis engine](src/robokudo/descriptors/analysis_engines/scripted_tetris.py)
accepts these settings:

| Setting | Behavior |
| --- | --- |
| `scenario` | A `ScriptedScenario` containing ordered `ScriptedObservation` values. |
| `repeat` | Repeat the sequence when true; report `Status.FAILURE` at completion when false. |
| `timing.frame_interval` | Positive `datetime.timedelta` controlling playback speed and capture-time increments. |
| `timing.capture_start` | First capture time in nanoseconds since the epoch. Defaults to the current time. |
| `clock` | Elapsed-time callable for scheduling. Defaults to `time.monotonic`. |

Capture times describe the scripted timeline and continue increasing across repeats.
Waiting ticks return `Status.RUNNING` and leave the last completed CAS intact.
Consumers should process a new observation on `Status.SUCCESS`. The enclosing
runner owns process shutdown when finite playback reports completion.

## Verify

After sourcing the same ROS environment:

```bash
.venv-foundation-pose/bin/python -m pytest test/robokudo_test/test_scripted_observations.py
```

The tests use a controllable clock and the checked-in meshes. They require no
camera, inference models, downloaded datasets, or credentials and are discovered
by the existing RoboKudo CI suite.
