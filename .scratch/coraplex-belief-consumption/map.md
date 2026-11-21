# Coraplex belief consumption

Status: ready-for-agent

Implementation permission: not granted (2026-11-21: specification and tickets
only, no code).

## Notes

- Design: [coraplex_belief_consumption.md](../../robokudo_acrambly/doc/coraplex_belief_consumption.md).
  Discussion record: [coraplex_belief_consumption_discussion.md](../../robokudo_acrambly/doc/coraplex_belief_consumption_discussion.md).
- Specification for part 1: [spec.md](spec.md). Part 2 and the real-camera work
  get their own specification once the Coraplex action plan exists.
- Order of pull requests: PR 13 is merged, then the Tetris PR
  ([prompt](../../robokudo_acrambly/doc/tetris_beliefs_prompt.md)) is merged,
  then this work branches from `main`.
- These documents were written on PR 13's branch. The user saves them and removes
  the planning commits from PR 13 before merging it.
- File and line references were read before the Tetris PR and must be checked
  again.
- The planning session ran on macOS, which lacks the project's environment;
  implementation runs on a Linux machine. Paths are relative to the repository
  root.

## Decisions-so-far

- RoboKudo accepts beliefs directly into the robot's shared world
  ([ADR 0001](../../docs/adr/0001-accept-beliefs-into-the-shared-robot-world.md)).
- No local mode for the aircraft engine, to keep the code base clean
  (ADR 0001). The Tetris engine moves to the shared world in a later PR.
- Coraplex applies held-back updates only between plans, at perception points
  ([ADR 0002](../../docs/adr/0002-apply-belief-updates-only-between-plans.md)).
- Aircraft semantics stay in `robokudo_acrambly`; only the Coraplex plan script
  imports them ([ADR 0003](../../docs/adr/0003-keep-aircraft-semantics-in-acrambly.md)).
- Capture times travel in an accepted capture message beside the world, with the
  stream position of their poses
  ([ADR 0004](../../docs/adr/0004-publish-capture-times-beside-the-world.md)).
  An annotation field was rejected because every capture would be a model change,
  and Giskard aborts a running motion on a waiting model change.
- `std_msgs/String` with `DataclassJSONSerializer` JSON; publisher and receiver in
  `robokudo_acrambly`; the receiver keeps the newest capture per part; volatile
  QoS.
- The engine depends on an abstract world source; existing tests switch through
  one shared fixture.
- The world frame is the fetched world's root. Tracy's Giskard root is `map2`,
  the stand-in's is `map`.
- The stand-in world server lives in `acrambly_temp/`.
- Inspection without a world server raises a custom exception naming the
  stand-in script.
- Rear wheels leave the perception catalog and its tests only; the four current
  docs go from 12 parts to 10.
- Glossary: *accepted capture*, *capture time*, *perception point*, *settle time*.

## Fog

- Whether the Tetris PR reshaped the aircraft catalog, which changes where ticket
  01 removes the rear wheels.
- How the RoboKudo pipeline hands its ROS node to the publishing annotator.
  Settle this while implementing ticket 04, without changing `robokudo/`.
- Part 2: where the perception points are, which parts each plan needs, the
  timeout, when to skip the wait, belief vs the robot's own action, and two
  writers on one part.
- A part seen for the first time during a Giskard motion adds a body, which
  aborts the motion. A gated pipeline avoids this; it is a part 2 and
  real-camera concern.
- `meshes/plane/` replaced `plane/` in the user's working tree;
  `test/robokudo_acrambly_test/test_aircraft_models.py` still names `"plane"`.
  This is expected to be settled before this work starts.
