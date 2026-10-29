# Accept aircraft-part observations into RoboKudo's local belief world

Status: implementation authorized for tickets 01–06 on 2026-10-28.

For configuration and current behavior, read
[aircraft models](aircraft_models.md), the generic
[observation contract](../../robokudo/docs/semantic_observations.md), and
[verification evidence](aircraft_belief_verification.md).

The [feature specification](../../.scratch/local-world-belief-updates/spec.md)
contains the observation contract, acceptance matrix, and verification criteria.
The [feature map](../../.scratch/local-world-belief-updates/map.md) links the
dependency-ordered implementation tickets.

## Goal and boundary

Make `SemanticDigitalTwinConnector` accept CAS observations of aircraft parts and
create or update persistent object beliefs in RoboKudo's local world:

`CAS object hypotheses → SemanticDigitalTwinConnector → local belief world`

Each physical part has its own belief, body, persistent UUID, semantic annotation,
and pose. Preserve its identity and geometry through movement, temporary absence,
and reappearance. Persistence here means retention across observations in the
running local world.

`WorldSynchronizer` transfers accepted changes to another process's world. That
transfer remains a separate responsibility.

The acceptance scenario uses the 12 named STL parts in `plane/`. Tetris belongs
to the previous task. This brief supersedes the earlier Tetris connector brief.

## Application-owned semantics and model resources

Define aircraft-part semantic annotations in `robokudo_acrambly`, under a common
`AircraftPart` base, with distinct types corresponding to these mesh parts:

| Mesh part | Current model resource |
| --- | --- |
| BottomWing | `plane/BottomWing.stl` |
| Chassis | `plane/Chassis.stl` |
| FrontWheel | `plane/FrontWheel.stl` |
| MotorGrill | `plane/MotorGrill.stl` |
| Propeller | `plane/Propeller.stl` |
| RearWheelLeft | `plane/RearWheelLeft.stl` |
| RearWheelRight | `plane/RearWheelRight.stl` |
| RearWing | `plane/RearWing.stl` |
| TopWing | `plane/TopWing.stl` |
| UnderBody | `plane/UnderBody.stl` |
| UpperBody | `plane/UpperBody.stl` |
| Window | `plane/Window.stl` |

The generic connector accepts a configurable mapping from classification labels
to semantic annotation types. `robokudo_acrambly` supplies that mapping and the
model resource configuration. Preserve the existing dependency direction:
application code imports generic RoboKudo and semantic digital twin interfaces.

A plane hypothesis supplies its model label and observed pose; each CAS supplies
capture and coordinate-frame metadata. Mesh geometry need not be repeated in
observations. Load each configured model once and reuse its geometry when
creating independent bodies. Retain the classification label on the accepted
semantic annotation using existing semantic interfaces.

Use the current original STL meshes for both visual and collision geometry.
Later, switch the configured resources to simplified meshes for both roles.
Preserve metric scale and each source mesh's coordinate frame. Mesh
simplification is a separate task. This choice supersedes the earlier plane
collision plan's use of detailed visual meshes for this connector scenario.

The semantic scope is types, labels, bodies, geometry, and poses. Grasp
definitions and assembly relationships belong to later work.

See the [glossary](../../GLOSSARY.md) and
[architecture decision](../../docs/adr/0003-keep-aircraft-semantics-in-acrambly.md).

## Verified starting point

The interview inspected `main` at `e09436a30`. The connector, belief registry,
world helpers, and connector tests already exist. The checkout also includes the
previous scripted Tetris source; its presence is not a prerequisite for plane
acceptance.

Existing association performs one-to-one assignment over image regions, stamped
world poses, and bounding-box dimensions. Successful assignments update beliefs;
low-similarity assignments and excess hypotheses create beliefs. Absent beliefs
remain. At that starting point, classification labels did not constrain
association.

At that starting point, body updates derived their pose from a bounding box,
replaced visual geometry with a box, and left collision geometry unset. The
implemented configuration adds model compatibility, typed beliefs, and
geometry-preserving pose updates.

Reference files:

- `robokudo/src/robokudo/annotators/semantic_world_connector.py`
- `robokudo/src/robokudo/world.py`
- `robokudo/src/robokudo/types/belief_state.py`
- `robokudo/src/robokudo/types/annotation.py`
- `robokudo/src/robokudo/world_descriptor.py`
- `robokudo/src/robokudo/utils/hypothesis_comparators.py`
- `robokudo/src/robokudo/utils/annotator_helper.py`
- `semantic_digital_twin/src/semantic_digital_twin/semantic_annotations/mixins.py`
- `semantic_digital_twin/src/semantic_digital_twin/semantic_annotations/semantic_annotations.py`
- `test/robokudo_test/test_semantic_world_connector.py`
- `test/robokudo_test/test_full_ae_execution.py`
- `test/robokudo_test/conftest.py` and `test/robokudo_acrambly_test/conftest.py`

## Required behavior

### Association and identity

Reuse the existing comparator, assignment, belief registry, and world-update
machinery. Association similarity and classification confidence remain distinct.
Do not introduce new acceptance thresholds or an uncertainty model.

Known distinct model kinds are incompatible for association: a `TopWing`
observation cannot update a `BottomWing` belief. A model identifier describes a
kind of part; neither it nor a per-frame detection identifier is the persistent
identity of a physical part.

Create a belief and body for an unmatched accepted hypothesis. A successful match
updates the existing belief and body while retaining their UUID and semantic
annotation. Keep low-similarity and unmatched-hypothesis behavior covered by
regression tests. Plane-specific configuration must preserve working behavior of
the generic connector when that configuration is absent.

### Poses, frames, and capture time

An observation pose locates the mesh origin, rather than its bounding-box center.
Interpret a plain `PoseAnnotation` in the CAS camera frame. Accept stamped poses
in the declared camera or world frame. Reject unsupported or ambiguous frames.

Convert camera poses to the declared world frame exactly once. A pose already in
world coordinates stays in world coordinates. Carry the observation's capture
time and frame into stamped world annotations, and retain original observation
metadata. Reprocessing annotations must not compound camera transforms.

### Geometry and accepted updates

Initialize a body's visual and collision collections from the configured model
before adding it to the world. Use the same model geometry for both collections,
with the body frame at the mesh origin and local shape offsets retained.

Subsequent accepted pose updates translate and rotate that existing body without
replacing its geometry or accumulating shapes or semantic annotations. Keep
geometry initialization separate from pose updates. Preserve the existing generic
bounding-box behavior for observations without configured model geometry.

### Absence and invalid observations

Absent parts retain their beliefs, UUIDs, semantic annotations, geometry, last
accepted poses, and last accepted capture times. An empty observation makes no
object-state changes. Reappearance updates the corresponding existing beliefs.

In the configured plane pipeline, missing, unknown, or conflicting model labels
are invalid. Invalid poses and missing required capture/frame/transform metadata
are invalid too. Validate the entire observation before changing persistent
beliefs, so an invalid hypothesis does not leave earlier hypotheses partially
applied. Raise a meaningful exception and never fabricate a world pose.

## Integration and scope

Demonstrate acceptance in a local pipeline containing the existing connector,
with deterministic plane observations built by test fixtures through public CAS
annotation types. Initialize world and camera frames through existing APIs, using
a nonidentity camera transform with both translation and rotation.

Keep generic connector regression tests in `test/robokudo_test`. Put tests that
import aircraft types and configuration in `test/robokudo_acrambly_test`, which
already has a CI entry. Reuse existing world, CAS, ROS, and blackboard fixtures.

This change does not add a production synthetic source, plane detector, external
inference worker, receiving process, synchronization startup, tracking service,
RViz configuration, Coraplex plan, or Giskard control. Retain the existing absence
policy; new stale/lost/removal policies and unrelated world-state refactors are
outside the scope. Assembly poses, insertion paths, and mesh simplification
remain separate work.

## Tests and acceptance criteria

Prove every missing behavior with a focused failing pytest test before changing
implementation. Preserve existing failing tests. Keep code snippets and fixture
data in separate files of the appropriate type, and isolate local world and
blackboard state between tests.

Cover:

- Each configured model label resolves to its expected aircraft semantic type
  and model resource; all types are aircraft parts.
- Creation with no prior beliefs installs both mesh collections and one semantic
  annotation per accepted part.
- Successful association preserves the belief, body, UUID, geometry, and semantic
  annotation; incompatible model kinds cannot match.
- Unmatched and low-similarity observations retain established behavior.
- Plain camera poses and stamped camera/world poses place the mesh origin
  correctly; a nonidentity camera transform is applied exactly once.
- Capture times and frames are preserved, including when annotations are
  processed again.
- Pose-only updates preserve mesh geometry and local offsets without accumulating
  shapes or annotations.
- Existing generic bounding-box behavior remains supported.
- Invalid labels, poses, frames, capture metadata, and required transforms raise
  meaningful exceptions before persistent state changes. Include an invalid
  hypothesis after a valid one to expose partial application.
- Partial absence, empty observations, and reappearance retain or update exactly
  the expected beliefs and their accepted state.

Construct a six-stage pipeline sequence using all 12 configured part kinds:
all present, translation, rotation, some absent, all absent, and all reappearing.
Use deterministic test poses; an assembled-plane layout is unnecessary. Assert
exact belief counts, UUIDs, semantic types, accepted poses, model geometry, and
unchanged absent-part state. Derive expected values from fixtures and model
definitions. A separate case with two physical copies of one model is not
required.

The checked-in STL resources are available to CI. Use small deterministic local
fixtures for focused generic cases where appropriate, and exercise the configured
plane resources in application coverage. Tests must require no live external
services or credentials.

Run focused package batches through pytest with the configured project
interpreter, existing ROS setup, and `--orm-build never`. Check available memory
and cap each run's memory before starting. Run the existing connector tests,
plane-specific acceptance tests, and relevant full-engine regressions. Regenerate
ORM interfaces explicitly only if needed, and never read or edit generated
interfaces to debug them. Run `scripts/format_docstrings.py` on modified Python
files. Record exact results and remaining environment limitations.

## Implementation workflow and deliverable

After explicit user permission to start implementation, work on a branch from
`main`, such as `feature/local-world-belief-updates`. Preserve unrelated work and keep the
change independent of other feature branches. Follow applicable `AGENTS.md`,
`docs/agents/`, and `semantic_digital_twin/doc/style_guide.md` guidance.

Provide the focused implementation, regression coverage, and application
configuration documentation, including model-resource replacement, coordinate
frames, validation, identity, geometry, and absence behavior.

Suggested PR title:
`Create and update local aircraft-part beliefs from CAS observations`.

Push only to the explicitly authorized
[ACRAMbly fork](https://github.com/ACRAMbly/cognitive_robot_abstract_machine.git).
Read the complete `git remote -v` output before pushing. Open the PR in
`ACRAMbly/cognitive_robot_abstract_machine`, targeting that fork's `main`, with
both head and base belonging to the fork. Include exact test results and report
existing satisfied behavior. Never push to or modify an upstream
`cram2/cognitive_robot_abstract_machine` PR.
