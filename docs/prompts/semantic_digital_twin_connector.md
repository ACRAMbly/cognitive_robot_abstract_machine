# Prompt: accept observation hypotheses into RoboKudo's local belief world

Work on a new branch created from `main`, suggested name
`feature/local-world-belief-updates`. If the user has already created that branch
from `main`, use it. Prepare a reviewable change for a PR titled
`Create and update local object beliefs from observation hypotheses`.

Push the implementation branch to the explicitly authorized
[ACRAMbly fork](https://github.com/ACRAMbly/cognitive_robot_abstract_machine.git)
and open the PR in `ACRAMbly/cognitive_robot_abstract_machine`, targeting that
fork's `main` branch. Both the head and base branches must belong to that fork.

## Goal

Make `SemanticDigitalTwinConnector` accept CAS observation hypotheses and create
or update persistent object beliefs in RoboKudo's local world. Reuse existing
association and world-update machinery, preserve object identity and CAD geometry,
and prove the behavior with deterministic observations of the two Tetris blocks.

The boundary of this change is:

`CAS object hypotheses → SemanticDigitalTwinConnector → RoboKudo local belief world`

`WorldSynchronizer` transfers accepted changes to another process's world. That
transfer is outside this PR and is not the connector's responsibility.

## Establish the starting point

- Read applicable `AGENTS.md` instructions and
  `semantic_digital_twin/doc/style_guide.md`. Read `GLOSSARY.md` and `docs/agents/`
  guidance if present; they are absent from the inspected `main` and are not
  prerequisites supplied by this prompt. If using a skill, follow its instructions
  and any available repository prerequisites.
- Inspect the branch and working tree and confirm the branch starts from `main`.
  Preserve unrelated work and use an isolated worktree when needed.
- The connector, belief type, world helpers, and connector tests already exist on
  `main`. Inspect their actual behavior, retain working functionality, and extend
  missing behavior with tests. CAD-preserving updates from other feature branches
  must not be assumed to exist.
- Keep this PR independent of the synthetic-data PR. Do not stack it, merge that
  branch, or import its proposed modules. Use local test fixtures to construct CAS
  hypotheses through the public annotation types available on `main`.

## Existing reference and implementation files on main

- `robokudo/src/robokudo/annotators/semantic_world_connector.py`
- `robokudo/src/robokudo/world.py`
- `robokudo/src/robokudo/types/belief_state.py`
- `robokudo/src/robokudo/types/annotation.py`
- `robokudo/src/robokudo/world_descriptor.py` and its `ObjectSpec` mesh support
- `robokudo/src/robokudo/utils/hypothesis_comparators.py`
- `robokudo/src/robokudo/utils/annotator_helper.py`
- `test/robokudo_test/test_semantic_world_connector.py`
- `test/robokudo_test/test_full_ae_execution.py`
- `test/robokudo_test/conftest.py` and `test/conftest.py`

## New test coverage and documentation

Create a focused test module such as
`test/robokudo_test/test_local_belief_updates.py` and any required fixture data.
This is a proposed new file, not an existing module. Add documentation of local
belief acceptance in a new file such as `robokudo/local_belief_updates.md`.

Do not expect `scripted_observations.py`, `scripted_tetris.py`,
`test_scripted_observations.py`, `test_scripted_twin.py`, or
`robokudo/scripted_tetris.md` on `main`. They are not prerequisites for this PR.

## Required behavior

1. Consume the current CAS hypotheses and associate them with persistent
   `ObjectBeliefState` instances using the existing comparator and assignment
   machinery. Reuse configuration where sufficient; change the connector only
   where tests demonstrate missing behavior. Keep association similarity distinct
   from classification confidence.
2. Create a belief and body for an unmatched accepted hypothesis. A successful
   match updates the existing belief and body while preserving their persistent
   UUID. Keep low-similarity and unmatched-hypothesis behavior explicit and covered
   by regression tests. A model identifier names an object kind, not a physical
   instance's persistent identity.
3. For the two distinct supplied models, translation, rotation, temporary absence,
   and reappearance must preserve identity without duplicate beliefs. Configure
   the existing association rules appropriately; do not equate a per-frame
   detection identifier with a persistent UUID.
4. Convert camera-relative poses into the declared world frame exactly once.
   Carry the observation's capture timestamp and frame into stamped annotations.
   Prove this with a nonidentity camera transform containing translation and
   rotation. Avoid repeatedly transforming an annotation already in world space.
5. For hypotheses carrying CAD geometry and a valid plain pose, initialize the
   body with its visual mesh and enclosing collision box using existing geometry
   support. Keep the body frame at the CAD origin and retain the box's local
   center offset. Geometry must be present when the body is added to the world.
6. Accepted pose updates must move and rotate the existing body without replacing
   its CAD visual geometry or accumulating collision shapes. Keep geometry
   initialization separate from subsequent pose updates. Preserve the existing
   bounding-box behavior for hypotheses without CAD geometry.
7. With one or both objects absent, retain the corresponding beliefs, UUIDs,
   geometry, and last accepted poses. Do not synthesize a new last-seen timestamp
   for an absent observation. Reappearance of the scripted objects must update
   their existing beliefs.
8. Missing pose or transform data must not produce a fabricated world pose or
   silently mix camera and world coordinates. Inspect the existing annotation
   contract and preserve compatible supported inputs. Add focused tests for
   unsupported pose updates and use a meaningful existing or custom exception
   where the input is invalid. Do not invent acceptance thresholds or a new
   uncertainty model without a requirement.

## Integration and scope

Demonstrate local acceptance in a pipeline containing the existing connector.
Supply fresh CAS observations from test fixtures using public annotation types;
the production synthetic source and its analysis engine belong to the other PR.
Initialize the local world root and fixed camera transform through existing APIs,
and ensure each CAS carries the required frame metadata. Reuse the connector
tests' existing pipeline wiring. Do not add a second production synthetic source.

Do not add native synchronization startup, ROS receiving processes, stream-position
barriers, RViz configuration, Coraplex plans, Giskard control, external inference
workers, tracking services, or new stale/lost/removal policies. Do not expand the
shared world API or refactor unrelated global state merely to support this PR.

## Tests and acceptance criteria

Use test-driven development for every missing behavior. Add focused failing pytest
tests before implementation, reuse existing world/CAS fixtures, and do not weaken
or modify existing failing tests. New test classes and mimics must be named for
their behavior. Keep tests isolated so local world and blackboard state cannot
leak between cases.

Cover creation with no prior beliefs; successful association; unmatched and
low-similarity hypotheses; nonidentity camera-to-world conversion; timestamp and
frame preservation; CAD mesh and offset collision-box creation; pose-only updates
preserving geometry and UUIDs; bounding-box fallback; missing pose/transform data;
one object absent; an empty observation; and reassociation on reappearance.

Construct a six-stage observation sequence in test fixtures: both objects visible,
translation, rotation, the first object absent, both objects absent, and both
objects reappearing. Exercise it through the local connector pipeline. Assert exact
belief counts, retained UUIDs, accepted poses, geometry, and unchanged absent-object
state. Derive expected values from the fixtures and model geometry. Use deterministic
local mesh fixtures so this test runs on `main` without the other PR, a developer's
untracked CAD assets, a receiving process, or synchronization transport. Additional
checks using the supplied Tetris meshes may run when those assets are available;
equivalent fixture-based coverage must run in CI.

Run tests through pytest using the configured project interpreter. Determine ROS
overlays from the environment and existing setup instructions rather than assuming
a feature branch's virtual environment or documentation exists. Run the existing
connector tests, the new local acceptance tests, and relevant existing full-engine
regression checks. All new tests must be included in CI and require no live APIs
or credentials. Run `scripts/format_docstrings.py` on modified Python files.

## Deliverable

Provide the branch, focused implementation, regression coverage, and documentation
of the local acceptance boundary, coordinate frames, geometry, and absence
behavior. Include a PR-ready title and description with exact test results and
confirm the PR targets the fork's `main` independently of the synthetic-data PR.
Report existing satisfied behavior rather
than manufacturing changes. Do not publish, comment on, or modify an upstream
`cram2/cognitive_robot_abstract_machine` PR. Open a PR only in an explicitly
authorized fork: `ACRAMbly/cognitive_robot_abstract_machine`.
