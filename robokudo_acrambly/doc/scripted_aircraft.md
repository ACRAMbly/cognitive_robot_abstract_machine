# Scripted aircraft observations

The `scripted_aircraft` analysis engine emits camera-relative aircraft-part
observations and accepts them through `SemanticDigitalTwinConnector` into the
robot's world, which it fetches from a world server and keeps synchronized with
every other process holding it. Its default scenario
contains one part of every kind in
[`AircraftModel`](../src/robokudo_acrambly/aircraft/models.py). The catalog owns
the labels, semantic types, resource paths, metric scale, and source mesh origins;
the scenario does not maintain another inventory. The parts move independently
and need not form an assembled aircraft.

## Setup and headless execution

Complete the repository's [installation](../../README.md#installation), including
the ROS 2 overlay containing RoboKudo messages and Python dependencies. From a
new terminal, substitute your checkout and overlay paths:

```bash
cd /path/to/cognitive_robot_abstract_machine
source /opt/ros/jazzy/setup.bash
source "$HOME/ros_ws/install/setup.bash"
source "$HOME/.virtualenvs/cram-env/bin/activate"
uv sync --active
python scripts/regenerate_all_orm.py
export AIRCRAFT_FRAME_CONFIGURATION="$PWD/robokudo_acrambly/doc/aircraft_frames.json"
export AIRCRAFT_MESH_DIRECTORY="$PWD/meshes/plane"
python -m robokudo.scripts.main _ros_pkg robokudo_acrambly _ae scripted_aircraft _headless _tickrate 20
```

Start a world server first. Without Giskard, run the stand-in in its own terminal
with the same environment, and keep it running:

```bash
python acrambly_temp/stand_in_world_server.py
```

It serves a world whose only body is its root `map`, and keeps a synchronizer of
its own, so a process that fetches the world later also gets every part accepted
since. Never run the aircraft engine next to a real Giskard yet. When no world
server answers within ten seconds (`WorldServerSource.timeout_seconds`), the
engine raises `WorldServerUnavailable`, which names the stand-in script.

Use the configured Python 3.12 interpreter, here
`$HOME/.virtualenvs/cram-env/bin/python`; ROS modules such as `sensor_msgs` must
remain importable in that interpreter. Regenerate ORM interfaces for a fresh
checkout as above; they are generated, ignored files. The headless command
requires the configured aircraft meshes; it needs no display, camera topics,
inference service, or credentials.

`_ros_pkg robokudo_acrambly` selects the application's descriptors and
`_ae scripted_aircraft` selects the aircraft source and belief connector.
`_headless` disables native windows. `_tickrate 20` ticks the tree twenty times
per second, accommodating the
default ten captures per second. Keep the tick rate at least as high as the
configured capture cadence. Overdue captures are delivered in order, with their
scheduled times; slow processing can extend elapsed execution beyond thirty
seconds without discarding captures.

The sequence includes time zero and thirty seconds: **301 completed captures**.
After the final capture the source returns stable `FAILURE`, and the headless
runner exits. A due capture runs the source followed by the threaded belief
connector. The pipeline returns `RUNNING` while that update completes and `SUCCESS` after
acceptance. A source waiting for its next capture returns `RUNNING` and retains
the preceding CAS. Exhaustion retains the final CAS and never loops.

## Required frame configuration

`AIRCRAFT_FRAME_CONFIGURATION` names a JSON document with the shape in
[`aircraft_frames.json`](aircraft_frames.json). Copy that file for your deployment
and supply both fields:

| Field | Meaning |
| --- | --- |
| `camera_frame` | Optical camera-frame identifier, distinct from the world's root |
| `camera_pose` | Explicit camera pose relative to the root of the robot's world, with `translation` in metres and `rotation` as an xyzw quaternion |

The world frame is the root of the fetched world, whatever its name: `map` for
the stand-in and Giskard's generic world, `map2` for Tracy's Giskard. It is not
configured. The supplied example declares `aircraft_camera`, a camera
translation of `(10, 20, 30)` metres, and identity orientation. These are
illustrative values, not sensor calibration. Replace them with the known
camera-to-world calibration. Explicit identity calibration is allowed; missing
configuration never supplies identity implicitly.

The launcher adds no frame body to the world: accepted parts hang directly from
its root. Programmatic applications can instead pass
`AnalysisEngine(frames=AircraftFrames(camera_frame=..., camera_pose=...))` with
an explicit `HomogeneousTransformationMatrix` relative to the root, and
`world_source=ProvidedWorldSource(world=...)` to hand over a world they already
hold.

## Optional native mesh verification

Load the same ROS and Python environment, configure the frame document, then
explicitly select your mesh directory. The checkout's `meshes/plane/` directory is an
example; it is not bundled in the application wheel:

```bash
export AIRCRAFT_FRAME_CONFIGURATION="$PWD/robokudo_acrambly/doc/aircraft_frames.json"
export AIRCRAFT_MESH_DIRECTORY="$PWD/meshes/plane"
python -m robokudo.scripts.main _ros_pkg robokudo_acrambly _ae scripted_aircraft _tickrate 20
```

`AIRCRAFT_MESH_DIRECTORY` is required, including in headless mode. Missing or
empty configuration raises `MissingAircraftMeshConfiguration`; unavailable model
resources raise `MissingAircraftResource` before world initialization. The
catalog supplies both semantic types and visual/collision geometry to the
connector. The existing source preview shares that same catalog.

Programmatic callers can pass `preview=AircraftPreview(catalog)` to reuse an
already loaded catalog with replacement resources. This catalog takes precedence
over the environment's directory, which must still be configured. `_headless`
suppresses windows but does not cancel preview geometry generation.

The native Open3D view displays independent mesh copies at the exact
camera-relative mesh-origin poses emitted in the CAS. Meshes load once per
pipeline configuration, retaining catalog units and original offsets without
recentering. Display updates never change shared catalog meshes or CAS poses.
No image is fabricated; the OpenCV control window has no aircraft camera image.
Keep Open3D enabled: `_no3d` disables the mesh window.

Completed captures replace the displayed parts, clearing geometry during total
absence. Waiting and exhausted ticks retain the last completed output. After
finite playback finishes, the viewer continues processing events and retains
the final reappeared scene for inspection. In the focused OpenCV control window,
Escape exits; right arrow, `n`, or `k` selects the next annotator, and left arrow,
`p`, or `j` selects the preceding one. Closing the Open3D window also exits.
Space retains the existing pipeline-trigger behavior and does not restart or
pause the finite schedule.

## Editing poses, motion, visibility, and timing

The public descriptor is
[`scripted_aircraft.py`](../src/robokudo_acrambly/descriptors/analysis_engines/scripted_aircraft.py).
Pass `scenario=AircraftScenario(...)` and an optional injectable `clock` for
programmatic control. Structured scene defaults live in
[`scenario.py`](../src/robokudo_acrambly/aircraft_observations/scenario.py);
generic timing, motion, and visibility descriptors live in
[`playback/`](../src/robokudo_acrambly/playback).

`AircraftLayout` configures columns, horizontal/vertical spacing, first origin,
translation distance, rotation angle, and `partial_absence_models`. Call
`create_parts()` and pass the resulting parts to `AircraftScenario(parts=...)`.
The default absence membership is every other member in catalog order. Change
that configured set to select a different subset without copying model labels
or resource filenames.

Each `AircraftPartState` has a `model`, initial `pose`, `keyframes`, and
`absence_intervals`. Poses locate the original mesh origin in the optical camera
frame: X right, Y down, Z forward. Translations are metres; rotations are valid
finite xyzw quaternions. Each `PoseKeyframe` specifies an elapsed nanosecond offset
and pose. Translation interpolates linearly, rotation follows the shortest
quaternion path, and exact keyframe endpoints are retained. Equal successive
poses define holds; an empty keyframe list leaves the part stationary, and the
last keyframe pose holds after its endpoint.

Each `AbsenceInterval(start_nanoseconds, end_nanoseconds)` is half-open: absent
at its start and visible again at its end. An empty list keeps the part visible;
multiple intervals hide it during their union. Motion continues independently
of visibility. Holds and reappearance still produce fresh detection identifiers.

`AircraftScenario.timing` is a `PlaybackTiming` with
`duration_nanoseconds` and `sample_period_nanoseconds`. Use a nonnegative duration
that is an exact multiple of a positive sample period. Zero duration emits one
capture. Keyframe offsets are strictly increasing and positive; absence starts
are nonnegative and precede their ends. Endpoints within playback align with the
sample period. Later keyframes and absence endpoints may remain when shortening
the duration.

| Elapsed capture time | Default observations |
| --- | --- |
| 0–3 seconds | All parts hold their initial poses |
| 3–9 seconds | All parts translate along camera X |
| 9–12 seconds | All parts hold their translated poses |
| 12–18 seconds | All parts rotate about camera Z |
| 18–21 seconds | All parts hold their rotated poses |
| 21–24 seconds | Configured partial subset is absent |
| 24–27 seconds | Empty scene |
| 27–30 seconds, including the endpoint | All parts reappear at their last visible poses and hold |

Each visibility boundary emits its new state once. Scheduled capture time drives
motion and visibility even if delivery is late.

## CAS output and local belief updates

Consume observations in later pipeline annotators through `get_cas()`, or
programmatically through `Pipeline.get_cas()`. The engine runs
`AircraftObservationAnnotator` followed by the configured
`SemanticDigitalTwinConnector` in the same pipeline. The connector updates
`robokudo.world.world_instance()`; no cross-process synchronization is started.

Each visible part supplies one `ObjectHypothesis` with a `Classification` whose
`classname` is its catalog label and one plain camera-relative `PoseAnnotation`
at the source mesh origin. Deterministic identifiers have the form
`scripted_aircraft:<capture index>:<catalog label>` and change on every capture,
including holds and reappearance. Labels identify kinds and detection IDs
identify individual observations; neither is a persistent physical-object UUID.
The connector appends the accepted stamped world pose to each hypothesis.

Each completed capture replaces the CAS and writes these fields anew, including
when no parts are visible:

| CAS field | Value |
| --- | --- |
| `data_timestamp` | Scheduled Unix capture time as integer nanoseconds, starting at the first tick's clock time |
| `world_frame` | Name of the root of the robot's world |
| `camera_frame` | Configured optical camera-frame identifier |
| `camera_to_world_transform` | Explicit finite rigid camera-to-world transform with matching reference/child frames |

`CAS.timestamp` remains construction time, separate from capture time. No camera
calibration, camera intrinsics, `CameraInfo`, depth-to-color metadata, images,
depth data, point clouds, populated masks/2D regions, bounding boxes, or
per-detection mesh/shape annotations are supplied. The hypothesis's existing
empty ROI default remains. Preview meshes stay in native annotator output.

The observation source itself creates no beliefs or semantic annotations. The
engine installs the fetched world and passes the completed CAS to the connector. Camera poses convert exactly once as
`world_T_part = world_T_camera @ camera_T_part`.

The configured connector creates a typed belief and mesh-bearing body for each
accepted aircraft part. Movement updates that body's world pose. Partial or
total absence retains its identity, geometry, last accepted pose, and capture
time; reappearance uses the existing association rules. Different model kinds
cannot associate. The source preview continues to show visible detections in
camera coordinates, so it clears absent parts even while beliefs persist.
Use the RViz inspection command below to display the resulting beliefs.

## Sharing beliefs with other processes

The engine installs the fetched world as RoboKudo's world and, when RoboKudo's
ROS node is registered, attaches one `WorldSynchronizer` on
`/semantic_digital_twin/world_sync`. Every new part, with its UUID, mesh and
semantic annotation, and every accepted pose change reaches the other
synchronized worlds; a receiving world created with
`defer_incoming_updates=True` sees them only after `apply_missed_messages()`.

After each capture from which it accepted at least one part, the engine publishes
an `AcceptedCapture` on `/robokudo_acrambly/accepted_captures`, as
`std_msgs/String` holding JSON, with volatile delivery. It lists the capture time
in Unix nanoseconds, the belief UUIDs accepted from that capture, and the
synchronizer's stream position after their poses. `AcceptedCaptureReceiver`
keeps the newest capture of each part; `PartCapture.is_applied` tells whether a
receiving world already holds that pose. The capture time is never written into
the world.

The engine derives `semantic_types` and `model_geometry` from one loaded
`AircraftModelCatalog`, following [aircraft model setup](aircraft_models.md) and
the generic [observation contract](../../robokudo/docs/semantic_observations.md).
Source-only tests exercise `AircraftObservationAnnotator` independently; the
engine integration test runs the real threaded consumer through movement,
partial absence, total absence, and reappearance for all ten aircraft kinds.

## Inspecting annotators and the beliefs

With the same ROS overlay, Python environment, and aircraft configuration as
above, start the stand-in world server first, then run this command instead of
the ordinary headless runner:

```bash
export AIRCRAFT_FRAME_CONFIGURATION="$PWD/robokudo_acrambly/doc/aircraft_frames.json"
export AIRCRAFT_MESH_DIRECTORY="$PWD/meshes/plane"
python -m robokudo_acrambly.aircraft_observations.inspection
```

This opens RoboKudo's standard OpenCV annotator controls and Open3D viewer,
alongside the ROS visualizers. The controls and camera-relative source preview
work as described under [native mesh verification](#optional-native-mesh-verification).
The inspection command requires a desktop display and never starts a world
server itself. Use RViz to view the retained beliefs in world coordinates.

In another terminal, source the same ROS installation and overlay, then run
from the checkout root:

```bash
rviz2 -d "$PWD/robokudo_acrambly/doc/aircraft_beliefs.rviz"
```

If a terminal opened from Snap-packaged VS Code reports a `GLIBC_PRIVATE`
symbol lookup error from `/snap/core20/`, remove its inherited GTK plugin path
for the RViz command:

```bash
env -u GTK_PATH rviz2 -d "$PWD/robokudo_acrambly/doc/aircraft_beliefs.rviz"
```

This preserves the sourced ROS library paths and changes only RViz's environment.

The inspection command runs the existing aircraft source and connector at twenty
ticks per second. `VizMarkerPublisher` publishes the actual
`robokudo.world.world_instance()` on `/semworld/viz_marker`, with its existing
visual geometry and collision fallback. TF locates each body's frame in the
world. Poses are in world coordinates, including the configured camera-to-world
calibration. Run one aircraft runner at a time against one world server.

The preset enables **Local belief geometry** and **Belief frames**, with reliable,
transient-local delivery for markers. Its fixed frame is `map`, the stand-in's root, and
its view is centered near the supplied example camera translation `(10, 20, 30)`.
For different calibration, adjust the view's focal point. For another world frame,
set **Global Options → Fixed Frame** to the root printed by `VizMarkerPublisher`
at startup. Enable **Belief frames → Show Names** to inspect frame identifiers.
Use the same ROS domain in both terminals.

Visually check the schedule above: all parts should appear, translate, rotate,
then stay visible at their last accepted poses through partial and total absence.
Reappearance should reuse the beliefs. The native source preview clears missing
detections, whereas this view shows retained beliefs. Retained and currently
observed beliefs use the publisher's existing appearance.

After the final observation, playback stops advancing while the annotator windows,
ROS, and TF remain active. You can still switch annotators and inspect the final
scene. RViz can also be opened after playback has finished: geometry is latched
and transforms continue to receive fresh timestamps. As in the standard RoboKudo
runner, Escape in the OpenCV window or closing Open3D stops the inspection
process. Ctrl-C also stops it. Closing RViz leaves the inspection process running.
Restart the stand-in and then the inspection to replay the sequence from a fresh
world: the stand-in keeps the parts accepted by earlier runs.

## Independent Tetris engine

Run `scripted_tetris` separately with `TETRIS_MESH_DIRECTORY`. It keeps its own
scenario, images, masks, geometry annotations, and preview. Aircraft configuration
does not run or configure Tetris, and there is no combined mode. The original
[Tetris command and setup](scripted_tetris.md), [mask guide](scripted_tetris_masks.md),
and [visualization guide](scripted_tetris_visualization.md) remain available.
