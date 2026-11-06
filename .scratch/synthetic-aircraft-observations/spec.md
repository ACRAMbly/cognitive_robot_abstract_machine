# Synthetic aircraft-part CAS observations

Status: ready-for-agent

Implementation permission: not granted. The user approved this design and
authorized specifications only. Production code, tests, fixtures, and runtime
configuration require subsequent explicit implementation permission.

## Problem and outcome

The existing `scripted_tetris` engine supplies synthetic observations, while
[PR #13](https://github.com/ACRAMbly/cognitive_robot_abstract_machine/pull/13)
accepts aircraft-part observations into persistent local beliefs. Its aircraft
sequence is a test fixture, not a production observation source.

Add a public `scripted_aircraft` engine in `robokudo_acrambly` that produces fresh
CAS observations for all twelve separate aircraft-part kinds. Its output must
satisfy PR #13's configured aircraft input contract without generating image
evidence or persistent beliefs. Optional native 3D visualization provides mesh
verification independently of the CAS payload.

Keep `scripted_aircraft` and `scripted_tetris` separate. Each has its own engine
entry point, scenario configuration, observation source, output contract, and
preview path. Launching one does not run or configure the other. Leave existing
Tetris implementation, tests, assets, and configuration unchanged. Feature the
aircraft engine in the package's primary synthetic-observation example, with
separate usage instructions for each engine.

## Authoritative dependencies

Use these existing definitions rather than creating parallel inventories or
reimplementing acceptance rules:

- [AircraftModel and AircraftModelCatalog](../../robokudo_acrambly/src/robokudo_acrambly/aircraft/models.py):
  model labels, semantic types, resources, and metric scale.
- [Configured observation contract](../../robokudo/docs/semantic_observations.md):
  accepted annotations, capture metadata, frames, and pose conversion.
- [PR #13 specification](../local-world-belief-updates/spec.md): aircraft semantics
  and the six acceptance stages.
- [Existing playback decisions](../synthetic-tetris-observations/map.md): finite
  scheduling, motion interpolation, absence, and native runner lifecycle.
- [Spatial style guide](../../semantic_digital_twin/doc/style_guide.md): transform
  notation and frame composition.

The inspected PR #13 head and checkout are
`3b86514b78521115fc1ed427dcbb77b23c43d023`. Future implementation needs those
interfaces or their merged successors; this specification does not assume they
already exist on `main`.

## Ownership and reuse

| Owner | Responsibility |
| --- | --- |
| Aircraft observation source | Scenario poses, visibility, scheduled capture times, detection IDs, and CAS metadata |
| Existing aircraft inventory | Labels, semantic kinds, mesh resources, units, and mesh origins |
| Hosting application or pipeline setup | Frame initialization and the configured camera-to-world transform |
| Optional native 3D output | Display copies of catalog meshes at observed poses |
| PR #13 connector, when separately configured | Validation, world-pose conversion, association, persistent identities, typed beliefs, and belief geometry |

Use existing generic clock, spatial, and native visualization interfaces where
their contracts fit. Sharing generic helpers does not combine the two observation
sources. Aircraft generation must not depend on Tetris-specific models, scenarios,
annotators, or renderers. Do not refactor Tetris into a combined source or add an
aircraft/Tetris mode switch. The earlier playback decisions specify observable
behavior to preserve in the aircraft source, not a requirement to merge their
implementations.

Follow the repository's code-reuse rules within this separation. Keep
aircraft-specific imports in `robokudo_acrambly`; generic RoboKudo and semantic
digital twin packages must not import application aircraft types.

## Observation payload

Every visible part produces one public `ObjectHypothesis` with the following
data:

| Data | Contract |
| --- | --- |
| Classification | `Classification.classname` equals the part's existing aircraft model label; labels are unambiguous and configured |
| Pose | One plain camera-relative `PoseAnnotation` locating the original mesh origin |
| Translation | Finite coordinates in metres |
| Orientation | Finite, valid xyzw quaternion, using existing pose conventions |
| Detection identifier | Deterministic and fresh for each detection in each completed capture |

Model labels identify kinds. Detection IDs identify observations. The source
does not assign persistent physical-object UUIDs or instantiate aircraft semantic
annotations attached to world bodies.

The default scenario contains one part of every `AircraftModel` kind. An extra
same-kind duplicate-copy scenario is not required by this feature.

Do not populate masks, 2D image regions, bounding-box annotations, per-detection
mesh/shape annotations, synthetic camera images, depth images, or point clouds.
An `ObjectHypothesis` may retain its existing empty ROI default; omitting image
evidence does not require changing RoboKudo's hypothesis class.

Camera calibration, `CameraInfo`, image intrinsics, and depth-to-color metadata
are unnecessary for this source. Capture time comes from `CAS.data_timestamp`.
The existing Tetris image and geometry contract remains intact.

## Capture and frame metadata

Write these fields into every completed CAS, including an empty scene:

| CAS field | Contract |
| --- | --- |
| `data_timestamp` | Scheduled capture time as integer nanoseconds since the Unix epoch |
| `world_frame` | Configured world frame identifier |
| `camera_frame` | Configured camera frame identifier, distinct from the world frame |
| `camera_to_world_transform` | Explicit finite rigid transform whose reference frame matches `world_frame` and child frame matches `camera_frame` |

Use `world_T_part = world_T_camera @ camera_T_part` as the consumer's conversion
contract. The source emits camera-relative part poses and does not append derived
world poses. Preserve nanosecond precision and keep capture time separate from
CAS construction time.

Pipeline CAS replacement discards prior views. Frame configuration therefore
lives outside a particular CAS and is written into each new capture; setting
metadata only once on the initial pipeline CAS is insufficient.

The application supplies frame configuration and its known transform. The source
must not infer the transform from part poses or silently substitute an identity
transform for missing configuration. Explicit identity calibration remains valid.
At connector consumption time, the declared world frame must already resolve to
an existing frame body as required by PR #13.

Frame initialization belongs to the hosting application or pipeline setup. Source
construction, playback, and preview leave world contents, connections, beliefs,
and semantic annotations unchanged. A standalone source can emit configured
frame metadata without registering bodies in a belief world. Documentation must
distinguish that observation-only run from a consumer whose frame setup is ready
for PR #13 validation.

## Scenario and playback

Keep poses, keyframes, absence intervals, frame configuration, and timing editable
through structured descriptors. Preserve mesh-origin poses and optical camera
axes: X right, Y down, Z forward. The parts are independent objects and need not
form an assembled aircraft.

Carry over the existing default duration and cadence: thirty seconds at ten
captures per second, including both endpoints, for 301 completed observations.
Adapt PR #13's all-present, translation, rotation, partial-absence, empty, and
reappearance stages to the existing continuous playback schedule:

| Elapsed capture time | Aircraft observations |
| --- | --- |
| 0–3 seconds | All parts hold their initial poses |
| 3–9 seconds | Parts translate between configured endpoints |
| 9–12 seconds | All parts hold the translated poses |
| 12–18 seconds | Parts rotate between configured orientations |
| 18–21 seconds | All parts hold the rotated poses |
| 21–24 seconds | A deterministic nonempty proper subset is absent |
| 24–27 seconds | No part hypotheses |
| 27–30 seconds, including the final sample | All parts reappear at their last visible poses and hold |

Use linear translation and shortest-path quaternion interpolation, retaining exact
keyframe endpoints. Holds still generate fresh detections with advancing capture
times. Visibility intervals are half-open and boundary states occur once. Motion
continues according to configured keyframes independently of visibility.

The pose descriptor defines the aircraft-specific initial layout and endpoints
once. Reuse the established illustrative translation and rotation conventions
where suitable; do not copy PR #13 fixture spacing as a verified display layout.
Select the partial-absence subset deterministically from the configured inventory
and expose it through scenario configuration. Tests derive expected poses and
subset membership from the definitions the source consumed.

Preserve every scheduled capture when delivery is late. Emit overdue captures in
order with their scheduled times; waiting ticks retain the preceding CAS. Use the
existing injectable clock so tests advance the sequence without sleeping.

Follow the existing runner statuses: `SUCCESS` for a completed capture, `RUNNING`
while waiting, and stable `FAILURE` after exhaustion. Preserve the last completed
CAS and do not restart or loop after the final capture. A zero-duration configured
sequence emits one observation; retain existing timing and boundary validation.

## Optional 3D verification

Use RoboKudo's native 3D annotator output to show the catalog meshes at the same
camera-relative mesh-origin poses emitted in the CAS. Load preview resources
through `AircraftModelCatalog` from an explicitly configured directory, such as
the checkout's `plane/` directory. Preserve catalog units and source offsets.

Meshes are display data, not mandatory hypothesis annotations. Transform display
copies so preview never changes shared catalog geometry or CAS poses. Load models
once per preview configuration, using the catalog's existing resource errors.
Headless observation generation does not need to load meshes solely to populate
the observation payload.

Keep separate parts readable in the preview. With no pixel projection or masks,
the old image-frustum and silhouette non-overlap constraints are not CAS
requirements. This does not introduce sensor occlusion simulation.

Each completed capture replaces displayed part geometry, including clearing it
for an empty scene. Waiting and exhausted ticks retain the last completed output.
The optional viewer stays responsive on the final scene until the user exits;
headless playback exits after exhaustion. Do not fabricate a CAS image to enable
mesh viewing, and do not retain the Tetris visualizer's box or mask prerequisites
in the aircraft preview path.

## Acceptance and verification

After explicit implementation permission, work test-first through the public
analysis engine and its completed CAS observations. Reuse existing ROS, CAS,
blackboard, world, clock, and native-output fixtures. New application tests belong
in `test/robokudo_acrambly_test`.

Required coverage:

- Every configured aircraft kind appears with its catalog label and configured
  mesh-origin pose; detection IDs change even during holds and after reappearance.
- All 301 captures follow the timeline, keyframes, absence subset, endpoint,
  waiting, late-delivery, and exhaustion contracts without stale detections.
- Every fresh CAS, including empty captures, carries exact capture and frame
  metadata; camera-to-world transforms preserve their configured meaning.
- Observations satisfy PR #13's public semantic acceptance interfaces with a
  preinitialized fixture world and a nonidentity transform containing rotation
  and translation. Compare prepared world poses with independently calculated
  expectations. These checks validate source compatibility without running
  persistent belief updates or duplicating PR #13's belief acceptance suite.
- The source and preview leave an isolated baseline world and belief state
  unchanged. Frame setup is fixture/application-owned and occurs before the
  baseline snapshot.
- Aircraft CAS output carries no populated masks/regions, mesh/shape or box
  annotations, camera calibration, synthetic images, depth data, or point clouds.
- Optional preview matches catalog geometry and observed poses, preserves shared
  model data, clears absent parts, and retains the final scene. Mock desktop
  boundaries so CI requires no display, hardware, network, or credentials.
- Existing Tetris source, timing, masks, geometry, resource configuration, and
  visualization remain covered by their unchanged regression tests.

Derive expectations from definitions and consumed fixtures; assert exact values
where determined. Preserve existing tests. Record each new behavior's failing
test before implementation, then its successful focused run. Use deterministic
local assets for preview tests and actual plane-resource checks when available,
paired with coverage that always runs in CI.

Run bounded pytest batches with the configured workspace interpreter and ROS
environment, `--orm-build never`, a free-memory check, and a per-run memory cap.
Use `scripts/regenerate_all_orm.py` only when regeneration is required. Format
modified Python files with `scripts/format_docstrings.py`. Exact implementation
checks and limitations belong in ticket completion records, not this approved
specification's status.

## Scope exclusions

This feature does not create/update persistent beliefs, run the connector in the
source-only engine, initialize belief-world frames, change association policies,
assign persistent UUIDs, simplify meshes, assemble aircraft, define grasps, model
sensor occlusion, add inference or live sensors, synchronize worlds, add
recording/looping controls, or modify the Tetris implementation. PR #13 remains
responsible for belief behavior.

## Delivery

The [feature map](map.md) indexes five dependency-ordered implementation tickets.
Each requires subsequent explicit user permission. Keep current changes limited
to these specification documents. Future implementation updates setup/run guides,
the primary package example, and precise verification records while preserving
the existing Tetris documentation and command.

## Approval record

The user approved separate parts, observation-only scope, a separate aircraft
engine retaining Tetris, inherited playback/lifecycle, omission of 3D boxes, and
the revised minimal PR #13 payload with optional catalog-based mesh verification.
The revised payload supersedes the earlier proposal to retain masks, image
regions, and per-observation meshes. The user subsequently authorized creating
specifications while explicitly withholding implementation permission.
The user clarified that aircraft and Tetris must remain separate; preserving
Tetris means leaving it unchanged, not combining the sources.
