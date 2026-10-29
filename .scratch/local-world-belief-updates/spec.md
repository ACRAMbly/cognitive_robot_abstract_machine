# Persistent aircraft-part beliefs from CAS observations

Status: resolved

Implementation permission: granted. The user explicitly requested implementation
of tickets 04–06 on 2026-10-28, following the committed implementation of 01–03.
This supersedes the earlier specification-only restriction.

## Problem

RoboKudo already associates observations with local object beliefs, but its body
updates use bounding boxes, replace visual geometry, and leave collision geometry
unset. It does not resolve aircraft classifications to semantic annotations.
Aircraft parts need persistent typed beliefs whose meshes and identities survive
movement, absence, and reappearance.

## Outcome and boundary

Accept public CAS object hypotheses through `SemanticDigitalTwinConnector` into
RoboKudo's local belief world. Each accepted physical aircraft part has one
belief, body, UUID, semantic annotation, and accepted world pose. Persistence
means retention across observations in the running process.

Use the 12 part kinds and current STL resources listed in the
[connector brief](../../robokudo_acrambly/doc/semantic_digital_twin_connector.md#application-owned-semantics-and-model-resources).
That table is the inventory reference. The
[glossary](../../GLOSSARY.md) defines the domain language, and
[ADR 0003](../../docs/adr/0003-keep-aircraft-semantics-in-acrambly.md) records
application ownership of aircraft semantics.

## Ownership and configuration

| Owner | Responsibility |
| --- | --- |
| `robokudo_acrambly` | `AircraftPart`, its 12 concrete semantic types, model definitions, and label-to-type mapping |
| Generic RoboKudo connector | Configurable semantic acceptance, validation, frame conversion, association, and local belief updates |
| Existing semantic digital twin interfaces | Bodies, semantic annotations, geometry, connections, and recorded world modifications |
| Test fixtures | Deterministic plane observations, camera transform, capture metadata, and expected states |

Generic RoboKudo and semantic digital twin modules must not import
`robokudo_acrambly`. The application supplies configuration through the generic
connector's extension point. Preserve existing generic behavior when aircraft
configuration is absent.

Aircraft semantic types expose their body and retain the accepted classification
label through existing semantic interfaces. They support queries for all
aircraft parts and for an individual part type. They do not introduce grasp or
assembly behavior.

Model definitions associate a classification label, semantic type, mesh resource,
and explicit metric scale. Use structured identifiers and dataclasses under
repository conventions. The configured mesh directory must not depend on a
developer's absolute path. Resource names and model labels have one definition.

Load each configured model once per configuration lifecycle. Reuse model data
without sharing mutable body state. Each body owns its shape collections and
frame references. Initially use the original STL geometry for both visual and
collision collections. Later resource replacement supplies simplified geometry
to both collections without changing observation labels or the CAS contract.

## Observation contract

| Input | Contract |
| --- | --- |
| `ObjectHypothesis` | One detection in the current observation; its detection identifier is not a persistent UUID |
| `Classification` | An unambiguous model label resolving through the configured mapping |
| Plain `PoseAnnotation` | Mesh-origin pose in the CAS camera frame |
| `StampedPoseAnnotation` | Mesh-origin pose in the declared camera or world frame, carrying capture time |
| CAS metadata | Declared world frame and capture time; camera frame and camera-to-world transform when a camera-relative pose requires them |
| Optional image/bounding-box evidence | Existing association evidence; a bounding box or per-observation mesh is not required to place a configured aircraft model |

Translations use metres and quaternions follow the existing xyzw convention.
Preserve capture-time precision using existing CAS and ROS timestamp interfaces.
Retain consistent observation metadata rather than substituting processing time.
Contradictory required metadata is invalid. A world-frame stamped pose does not
require a camera transform solely for accepting that pose.

Missing, unknown, or conflicting model labels are invalid in the configured
aircraft pipeline. Missing or invalid poses, unsupported or ambiguous frames,
and missing required capture or transform metadata are also invalid. Follow the
existing annotation contract when selecting the newest supported pose; preserve
support for a plain observation together with its derived world annotation.

## Acceptance behavior

### Validate before changing beliefs

Validate the entire observation and prepare required model geometry and world
poses before mutating persistent beliefs. A valid hypothesis followed by an
invalid one must leave all persistent object state unchanged. Apply this to
initial creation and observations of existing beliefs.

Raise meaningful existing or custom dataclass exceptions under the repository's
exception rules. Do not turn absent metadata into an identity transform, default
object pose, current timestamp, or untyped aircraft belief. This is validation
before mutation, not a new shared-world transaction or rollback subsystem.

### Convert poses once

For camera observations, use `world_T_part = world_T_camera @ camera_T_part`.
For world observations, retain `world_T_part` directly. The body frame stays at
the source mesh origin; mesh offsets remain local to that body.

Stamped world annotations retain the observation's capture time and declared
world frame. Processing annotations again must not compound the camera transform
or overwrite capture metadata. Verify with a camera transform containing both
translation and rotation, and geometry with a nonzero local center offset.

### Associate and accept

Reuse existing comparators and one-to-one assignment. Known different model
kinds are incompatible even when their positions, regions, or dimensions are
similar. A `TopWing` observation cannot update a `BottomWing` belief.

Association similarity and classification confidence remain separate. Preserve
established unmatched and low-similarity behavior without inventing thresholds.
Neither model labels nor detection identifiers become physical-object UUIDs.

For an accepted unmatched hypothesis, create a body with visual and collision
mesh collections present before world insertion, its belief, and one semantic
annotation. Use existing world and geometry APIs.

For an accepted match, retain the belief, body, UUID, semantic annotation, and
geometry. Update the existing body's world pose and accepted observation state.
Pose-only updates must translate and rotate the model without replacing meshes
or adding more shapes or semantic annotations.

### Retain absent parts

An absent part retains its belief, body, UUID, semantic annotation, geometry,
last accepted pose, and last accepted capture time. An empty observation changes
none of that object state. Reappearance reassociates the scenario's known part
kinds and updates their existing beliefs.

## Acceptance matrix

Let `N` be the number of configured part kinds, which must be 12 for this
scenario. Let `A` be a deterministic nonempty proper subset selected by the
fixture for partial absence. Derive expected values from model and fixture
definitions, not copies of production literals.

| Stage | Observed hypotheses | Persistent beliefs | Required evidence |
| --- | --- | --- | --- |
| Initial creation | `N` | `N` | All configured types, model geometry in both collections, distinct UUIDs |
| Translation | `N` | `N` | Accepted fixture translations; retained identities and geometry |
| Rotation | `N` | `N` | Accepted fixture orientations and correctly rotated local offsets |
| Partial absence | `N - len(A)` | `N` | Absent parts retain all accepted state; visible parts accept current observations |
| Total absence | `0` | `N` | Every part retains its last accepted state and time |
| Reappearance | `N` | `N` | Existing UUIDs reassociated, accepted poses updated, no duplicate beliefs or annotations |

Exercise the sequence through the existing connector in a local pipeline using
fresh CAS observations and changing detection identifiers. Test poses are
deterministic and need not form an assembled aircraft. A separate case with two
physical copies of the same model is not required.

Additional focused coverage must prove:

- Mapping and inheritance for each configured aircraft part, with model-resource
  failures reported before belief creation.
- Plain camera poses, stamped camera poses, and stamped world poses; correct
  capture metadata; world poses and repeated annotations not transformed twice.
- Rejection of a different model kind as an identity match, including when
  geometric evidence would otherwise favor the wrong belief.
- Existing unmatched, low-similarity, and generic bounding-box behavior.
- Each invalid-label, pose, frame, capture, and required-transform case leaves
  persistent state unchanged, including an invalid hypothesis after a valid one.
- Creation has geometry at insertion; accepted pose updates preserve geometry,
  shape counts, UUIDs, and semantic annotation identity.

## Verification and package boundaries

Generic connector tests belong in `test/robokudo_test` and must not need aircraft
types. Tests importing aircraft configuration belong in
`test/robokudo_acrambly_test`, which already has a CI matrix entry. Use existing
CAS, world, ROS, and blackboard fixtures. Keep new test data or mimic classes in
the appropriate test dataset files.

Use checked-in plane resources for application model coverage and small local
mesh fixtures for focused generic geometry cases. Tests require no live APIs,
credentials, hardware, real-time waits, or receiving process.

After implementation is authorized, prove missing behavior with failing pytest
tests before changing production code. Run existing connector tests, the new
generic tests, application acceptance tests, and relevant full-engine regressions
in separate focused batches. Use the configured interpreter and ROS environment,
`--orm-build never`, a checked memory budget, and a per-run memory cap. Regenerate
ORM interfaces explicitly only when needed. Format modified Python files with
`scripts/format_docstrings.py`. Record exact test results and limitations.

## Scope exclusions

This work does not implement mesh simplification, grasp definitions, assembly
relationships or poses, insertion paths, a plane detector, a production synthetic
source, synchronization startup or transport, tracking services, new stale/lost/
removal policies, durable storage, Coraplex plans, Giskard control, or unrelated
world-state refactors. Existing Tetris behavior is not the acceptance scenario.

## Delivery

Implementation is authorized by the user. For delivery,
follow the repository's branch and fork workflow described in the
[connector brief](../../robokudo_acrambly/doc/semantic_digital_twin_connector.md#implementation-workflow-and-deliverable).
Deliver focused code, tests, application configuration documentation, and exact
verification results. Publish only through the authorized ACRAMbly fork.

The [feature map](map.md) lists the dependency-ordered implementation tickets.

## Comments

- 2026-10-28: The user accepted the design decisions and authorized proceeding to
  specifications, explicitly withholding implementation permission.

- 2026-10-28: Tickets 04–06 implemented, verified and reviewed on the integration
  branch. All six local tickets are resolved; verification limitations are recorded
  in the application verification document. No push or PR was published.
