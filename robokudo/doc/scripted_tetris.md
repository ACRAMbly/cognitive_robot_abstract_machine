# Scripted Tetris observations

The `scripted_tetris` analysis engine supplies synthetic camera observations to
later RoboKudo annotators. It uses local CAD meshes and requires no camera,
inference model, credentials, or display. The default schedule captures thirty
seconds at ten observations per second, including both endpoints: 301 completed
observations.

## Setup and headless execution

Complete the workspace's [installation](../../README.md#installation), including
its ROS 2 workspace setup. In a new terminal, replace the two `/path/to` paths
below with your checkout and mesh directory:

```bash
cd /path/to/cognitive_robot_abstract_machine
source /opt/ros/jazzy/setup.bash
source "$HOME/ros_ws/install/setup.bash"
source "$HOME/.virtualenvs/cram-env/bin/activate"
uv sync --active
python scripts/regenerate_all_orm.py
export TETRIS_MESH_DIRECTORY=/path/to/tetris-meshes
python -m robokudo.scripts.main _ae scripted_tetris _headless _tickrate 20
```

The configured directory must contain `child_cube_0_colored.ply` and
`child_cube_2_colored.ply`. A missing resource reports its expected path. Their
model definitions declare millimetres and convert them to metres once, preserving
the original CAD origin.

The command runs headlessly and exits after the final observation. Its twenty
ticks per second accommodate the default ten captures per second. Set `_tickrate`
at least as high as your configured capture rate. Every scheduled observation is
preserved: if processing falls behind, overdue samples are delivered in order and
the actual run can take longer than the configured capture duration.

## Editing the scenario

The engine accepts an explicit mesh directory, scenario, and clock. The launcher
constructs the default `AnalysisEngine` and reads `TETRIS_MESH_DIRECTORY`. Camera,
block poses, and timing defaults live in
[`scenario.py`](../src/robokudo/scripted_observations/scenario.py); the public
descriptor is
[`scripted_tetris.py`](../src/robokudo/descriptors/analysis_engines/scripted_tetris.py).

`TetrisScenario.timing` is a `PlaybackTiming` descriptor with
`duration_nanoseconds` and `sample_period_nanoseconds`. Use a nonnegative duration
that is an exact multiple of a positive sample period. A zero duration emits one
observation. The configured sequence includes the initial capture at offset zero
and the final capture at the duration; each scheduled offset appears once.

Object translations are metres and quaternions are ordered `xyzw`. Camera-relative
poses locate the original CAD origin in the optical frame: X right, Y down,
Z forward. CAD geometry remains model-local; bounding-box dimensions and center
offsets come from the loaded mesh.

Each block also carries an image region and a cropped binary foreground mask
following its projected CAD outline. See the
[mask conventions and limitations](scripted_tetris_masks.md) for details.
Optional [native visualization](scripted_tetris_visualization.md) displays the
annotator's 2D and 3D outputs and retains the final scene for inspection.

## Consuming observations

The command supplies CAS observations in memory. Add later annotators to the
engine's pipeline to consume each observation through `BaseAnnotator.get_cas()`;
programmatic callers use `Pipeline.get_cas()`. The engine creates object
hypotheses without persistent beliefs or world updates. Model classifications
identify object kinds. Detection identifiers are deterministic and new for every
capture; they do not identify a physical object across captures.

Each completed sample replaces the pipeline's CAS, including an empty scene.
Camera calibration and frame remain available through `camera_info`,
`camera_intrinsic`, and `camera_frame`. `data_timestamp` is the scheduled Unix
capture time in integer nanoseconds, starting from the clock at the first tick;
the camera header stamp agrees with it. `timestamp` remains CAS construction time.
Tests inject a clock to advance playback without sleeping.

The source returns `RUNNING` while waiting for the next scheduled capture and
keeps the last completed CAS. A due observation returns `SUCCESS`, allowing later
annotators to process that capture. The first tick after the final completed
observation returns `FAILURE`, which signals completion to the headless runner.
Further ticks preserve the final CAS and cannot restart the sequence.
