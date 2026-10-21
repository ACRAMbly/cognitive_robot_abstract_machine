# Scripted Tetris observations and synchronized CAD display

## Static receiving-world demonstration (ticket 01)

This demonstration accepts **one initial observation** through
`SemanticDigitalTwinConnector`. Native `WorldSynchronizer` sends the accepted
world structure, geometry, and poses to an independent SDT World in a separate
receiving process. Only that process publishes CAD markers, collision boxes, and
TF. The source stays running after acceptance so its stream remains available.

Use the current project interpreter, `.venv-foundation-pose/bin/python` in this
checkout. Its name does not imply that FoundationPose runs: this demonstration
needs no inference service, camera, model weights, or credentials.

From the repository root, source both ROS overlays in **each terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source .venv-foundation-pose/ros_workspace/install/setup.bash
```

Start the receiver first and wait for `Receiver ready`:

```bash
.venv-foundation-pose/bin/python -m robokudo.scripts.scripted_tetris_sync receiver
```

Then start the source in another terminal:

```bash
.venv-foundation-pose/bin/python -m robokudo.scripts.scripted_tetris_sync source
```

The source reports two accepted CAD beliefs and its published stream position.
It checks for a peer native publisher and a matched receiving subscription before
creating the root, fixed camera connection, or object bodies. Its own subscription
does not satisfy this check. Without the receiver, startup fails with
`ReceiverUnavailableError` after five seconds and releases the source resources.
Use the dedicated synchronization topic only for this source/receiver pair. Both commands
accept `--topic /another/world_sync` and the source accepts
`--receiver-timeout SECONDS`. For parallel demonstrations, pass the same unique
`--namespace demo_name` to both commands and adjust the RViz marker and TF topics
to that namespace. The synchronization topic override must also be unique.

Start RViz in a third terminal:

```bash
rviz2 -d robokudo/scripted_tetris.rviz
```

The configuration uses Fixed Frame `map`, the receiving marker topics
`/scripted_tetris/visual` and `/scripted_tetris/collision`, reliable delivery, and
Transient Local durability. TF refreshes even while the accepted scene is static,
so RViz may start after the observation was accepted.

Check visually that both CAD blocks appear inside translucent collision
boxes, roughly 0.3 m apart. Their CAD origins are preserved; the boxes' centers
need not coincide with those origins. The source uses the identity transform from
`map` to `synthetic_camera`. Each fresh CAS carries that world frame and the fixed
camera-to-world transform. Model coordinates are converted from millimetres to
metres once, and each accepted CAD pose is transformed into `map` once. The mesh
and its offset collision box exist before body creation is recorded for replication.

Stop both applications with Ctrl+C. The source joins its connector worker and
closes synchronization. The receiver destroys its timer, removes visualization
and synchronization callbacks, and destroys their publishers and subscriptions.
Each application then releases its executor, shared-node registration, node,
and ROS context.

Late joins and receiver restarts are outside this contract: to restart, **stop both
applications**, restart an empty receiver, then restart the source. Starting a new
receiver beside a running source does not reconstruct previously published changes.

Run the automated geometry, readiness, shutdown, and separate-process checks with:

```bash
.venv-foundation-pose/bin/python -m pytest \
  test/robokudo_test/test_scripted_twin.py \
  test/robokudo_test/test_scripted_tetris_processes.py \
  test/robokudo_test/test_scripted_observations.py \
  test/robokudo_test/test_semantic_world_connector.py
```

These checks use isolated ROS streams and the supplied local meshes, and belong to
the existing RoboKudo CI suite. Receiving-world tests wait for the relevant native
source stream position before comparing geometry or poses. GUI inspection is a
separate manual check; passing pytest alone does not establish an RViz visual check.

## Observation-only engine

The original `scripted_tetris` analysis engine produces deterministic CAS object hypotheses
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
