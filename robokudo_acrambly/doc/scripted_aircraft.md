# Scripted aircraft observations

The `scripted_aircraft` analysis engine emits camera-relative aircraft-part
observations and accepts them into RoboKudo's local belief world through
`SemanticDigitalTwinConnector`. Its default scenario
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
export AIRCRAFT_MESH_DIRECTORY="$PWD/plane"
python -m robokudo.scripts.main _ros_pkg robokudo_acrambly _ae scripted_aircraft _headless _tickrate 20
```

Use the configured Python 3.12 interpreter, here
`$HOME/.virtualenvs/cram-env/bin/python`; ROS modules such as `sensor_msgs` must
remain importable in that interpreter. Regenerate ORM interfaces for a fresh
checkout as above; they are generated, ignored files. The headless command
requires the configured aircraft meshes; it needs no display, camera topics,
inference service, or credentials.

`_ros_pkg robokudo_acrambly` selects the application's descriptors and
`_ae scripted_aircraft` selects the aircraft source and local belief connector.
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
and supply all three fields:

| Field | Meaning |
| --- | --- |
| `world_frame` | Nonempty application world-frame identifier |
| `camera_frame` | Distinct optical camera-frame identifier |
| `camera_pose` | Explicit camera pose in the world, with `translation` in metres and `rotation` as an xyzw quaternion |

The supplied example declares `aircraft_world` and `aircraft_camera`, a camera
translation of `(10, 20, 30)` metres, and identity orientation. These are
illustrative values, not sensor calibration. Replace them with the known
camera-to-world calibration. Explicit identity calibration is allowed; missing
configuration never supplies identity implicitly.

The launcher creates detached calibration references and registers the declared
world-frame body in RoboKudo's world if absent. In a populated world the new
frame has a fixed identity connection to the existing root; an existing named
frame is reused. Programmatic applications can instead pass
`AnalysisEngine(frames=AircraftFrames(...))` with their own frame references and
explicit `HomogeneousTransformationMatrix`. The transform's reference frame
must match `world_frame`, and its child frame must match `camera_frame`.

## Optional native mesh verification

Load the same ROS and Python environment, configure the frame document, then
explicitly select your mesh directory. The checkout's `plane/` directory is an
example; it is not bundled in the application wheel:

```bash
export AIRCRAFT_FRAME_CONFIGURATION="$PWD/robokudo_acrambly/doc/aircraft_frames.json"
export AIRCRAFT_MESH_DIRECTORY="$PWD/plane"
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
| `world_frame` | Configured world-frame identifier |
| `camera_frame` | Configured optical camera-frame identifier |
| `camera_to_world_transform` | Explicit finite rigid camera-to-world transform with matching reference/child frames |

`CAS.timestamp` remains construction time, separate from capture time. No camera
calibration, camera intrinsics, `CameraInfo`, depth-to-color metadata, images,
depth data, point clouds, populated masks/2D regions, bounding boxes, or
per-detection mesh/shape annotations are supplied. The hypothesis's existing
empty ROI default remains. Preview meshes stay in native annotator output.

The observation source itself creates no beliefs or semantic annotations. The
engine initializes the declared world frame and passes the completed CAS to the
connector. Camera poses convert exactly once as
`world_T_part = world_T_camera @ camera_T_part`.

The configured connector creates a typed belief and mesh-bearing body for each
accepted aircraft part. Movement updates that body's world pose. Partial or
total absence retains its identity, geometry, last accepted pose, and capture
time; reappearance uses the existing association rules. Different model kinds
cannot associate. The source preview continues to show visible detections in
camera coordinates, so it clears absent parts even while beliefs persist.
Belief-world visualization remains separate work.

The engine derives `semantic_types` and `model_geometry` from one loaded
`AircraftModelCatalog`, following [aircraft model setup](aircraft_models.md) and
the generic [observation contract](../../robokudo/docs/semantic_observations.md).
Source-only tests exercise `AircraftObservationAnnotator` independently; the
engine integration test runs the real threaded consumer through movement,
partial absence, total absence, and reappearance for all twelve aircraft kinds.

## Independent Tetris engine

Run `scripted_tetris` separately with `TETRIS_MESH_DIRECTORY`. It keeps its own
scenario, images, masks, geometry annotations, and preview. Aircraft configuration
does not run or configure Tetris, and there is no combined mode. The original
[Tetris command and setup](scripted_tetris.md), [mask guide](scripted_tetris_masks.md),
and [visualization guide](scripted_tetris_visualization.md) remain available.
