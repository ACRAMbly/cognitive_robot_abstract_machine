# How Coraplex uses RoboKudo's aircraft-part beliefs: shared world with held-back updates

Status: chosen approach (E3b from
[the discussion record](coraplex_belief_consumption_discussion.md)). Nothing has
been implemented yet. Every step below comes from reading the code, and none
has been run.

Any change to a file outside `robokudo_acrambly/` or `acrambly/` ust be mentioned explicitly
and approved by the user.

## Starting point

[PR 13](https://github.com/ACRAMbly/cognitive_robot_abstract_machine/pull/13)
(`feature/local-world-belief-updates`) accepts CAS object hypotheses of the 12
aircraft parts into RoboKudo's local belief world. Each part has one persistent
belief, body, UUID, semantic annotation, mesh geometry, and accepted pose. These
survive movement, absence, and reappearance.

Goal: Coraplex uses those beliefs while it executes a plan.

Terms: no new glossary term. This document writes out what happens, for example
"Coraplex's world, which `WorldSynchronizer` keeps identical with Giskard's".
The existing terms local belief world, acceptance, and synchronization stay as
they are.

## How the worlds relate today

| Process | World it holds | Relation |
|---|---|---|
| Giskard | the robot's world | the original copy (serves `fetch_world`, `giskardpy/src/giskardpy/middleware/ros2/giskard.py:210`) |
| Coraplex | the robot's world | a copy of Giskard's, kept identical by `WorldSynchronizer` (`coraplex/src/coraplex/demonstrations.py:275-276`) |
| RoboKudo | the local belief world | separate: its own root, frames, and UUIDs, not synchronized with anything (`robokudo/src/robokudo/world.py:47`) |

In simulation (not `ExecutionType.REAL`), the Coraplex demo builds its own world
instead of fetching Giskard's (`coraplex/src/coraplex/demonstrations.py:263-273`).

## The approach

RoboKudo stops keeping a separate belief world and accepts beliefs directly
into the world Giskard and Coraplex share. Coraplex holds incoming updates back
and applies them at moments it chooses.

- At startup, RoboKudo fetches Giskard's world with `fetch_world_from_service`.
- It makes that world its own, with `set_world` and
  `init_world_entity_tracker_from_world`.
- It attaches a `WorldSynchronizer` on the normal
  `/semantic_digital_twin/world_sync` topic.
- PR 13's connector then accepts beliefs directly into that world. Each new
  part (UUID, mesh, annotation) and each accepted pose change is published to
  Giskard and Coraplex.
- The Coraplex demo script creates its `WorldSynchronizer` with
  `defer_incoming_updates=True` and applies the held-back updates
  (`apply_missed_messages()`) only between plans.
- The demo script splits a task into plans that run one after another, so
  that every perception point falls between two plans. There the script
  triggers one capture, waits for a fresh observation, applies the updates and
  builds the next plan (see [Perception points between plans](#perception-points-between-plans)).

### Connector and synchronizer

- `SemanticDigitalTwinConnector`: **used**, in RoboKudo, unchanged from PR 13.
  It writes into the shared world instead of a private one.
- `WorldSynchronizer`: **used.**
  - RoboKudo attaches one on the normal topic.
  - Coraplex's own synchronizer is created with `defer_incoming_updates=True`,
    so received changes wait until the demo applies them.
  - The existing Coraplex ↔ Giskard synchronization is otherwise unchanged.

### Why RoboKudo's world can't be synced as it is

- Sync messages identify bodies and connections by UUID (`original_parent_id`,
  `semantic_digital_twin/src/semantic_digital_twin/world_description/world_modification.py:204`).
  Coraplex can only apply RoboKudo's changes if both worlds start from the same
  world and the same root UUIDs.
- RoboKudo starts from an empty `World()` (`robokudo/src/robokudo/world.py:48`).
- The query path (`RoboKudoPerception` → `Detection.apply_to`) doesn't help:
  - `GenerateQueryResult` (`robokudo/src/robokudo/annotators/query.py:283`)
    answers from the current frame's object hypotheses, not from beliefs, and
    its `uid` is not the belief UUID.
  - `apply_to` needs Coraplex to have preloaded the part and moves the body
    itself, so both processes would write the same pose.
  - Queries take about 10 s each (`acrambly/sub_parts/real/cube_perception.py:85-87`).

### What already exists

- `fetch_world_from_service`, `FetchWorldServer`, and `WorldSynchronizer` in
  `semantic_digital_twin`. Coraplex demos already use the fetch-then-sync pair.
- `WorldSynchronizer.defer_incoming_updates`, `apply_missed_messages()`
  (`world_synchronizer.py:665`) and `resume()` (`world_synchronizer.py:724`).
- Giskard already holds back updates the same way
  (`giskardpy/src/giskardpy/middleware/ros2/giskard.py:208`,
  `giskardpy/src/giskardpy/middleware/ros2/world_updates.py:62-70`), so the
  motion controller is safe.
- `robokudo.world.set_world()` and `init_world_entity_tracker_from_world()`,
  used by `robokudo/src/robokudo/io/semdt_raytracer_camera_interface.py:145`
  and `robokudo/src/robokudo/annotators/world_descriptor_bootstrap.py:105`.
- The demo scripts already create their own synchronizer, e.g.
  `acrambly/tracy_real.py`.
- A sync can carry the aircraft parts: the JSON serializer stores the full
  class name and imports it on the receiving side
  (`krrood/src/krrood/adapters/json_serializer.py:150`), and `Mesh.to_json`
  includes the vertices. `robokudo_acrambly/aircraft/types.py` imports only
  `semantic_digital_twin`.

- Gates that hold a RoboKudo pipeline until it is triggered: `QueryAnnotator`
  waits for a query on RoboKudo's query action server
  (`robokudo/src/robokudo/annotators/query.py:128-155`), and `PipelineTrigger`
  waits for a blackboard flag inside the RoboKudo process
  (`robokudo/src/robokudo/annotators/pipeline_trigger.py`).

Not there yet: no RoboKudo code does the fetch and sync, no demo holds back
updates, and the capture time of an accepted observation stays on the RoboKudo
side. It is on the CAS and the `StampedPose`
(`robokudo/src/robokudo/annotators/semantic_world_connector.py:237-243`);
the aircraft annotations in `robokudo_acrambly/aircraft/types.py` have no
timestamp, so Coraplex cannot tell how old a pose in the shared world is.

## Steps

1. **Write the failing test first.** In `test/robokudo_acrambly_test`: two
   worlds fetched from one source, a synchronizer on each, one accepted
   aircraft observation in the RoboKudo world. Assert that the receiving world
   has the same UUIDs, types, and poses. A second test holds back updates on
   the receiving side and asserts the world is unchanged until
   `apply_missed_messages()` runs. A third test asserts that the receiving
   world can read the capture time of the accepted observation (step 6).
2. **RoboKudo starts from the shared world.** Where the aircraft engine starts
   up (for example `AircraftBeliefInspection.__enter__` in
   `aircraft_observations/inspection.py`), before the pipeline runs:

   ```python
   set_world(fetch_world_from_service(node=node))
   init_world_entity_tracker_from_world(world_instance())
   WorldSynchronizer(_world=world_instance(), node=node)
   ```

   - `set_world` alone leaves `world_entity_tracker` tracking the old empty
     world, which `robokudo/src/robokudo/io/storage.py:260` and
     `robokudo/src/robokudo/io/cas_view_codecs.py:526` read.
   - Never call a clearing function once the synchronizer is attached (see
     risks).
3. **Point the aircraft world frame at the shared root.** In
   `aircraft_frames.json`, change `world_frame` from `aircraft_world` to the
   shared root's name, e.g. `map`.
   - The aircraft engine
     (`robokudo_acrambly/src/robokudo_acrambly/descriptors/analysis_engines/scripted_aircraft.py:99-107`)
     adds a missing world frame under the existing root with a
     `FixedConnection`, so `aircraft_world` would not become a second root.
     Using `map` keeps an extra frame from being published to Giskard.
   - The synthetic source keeps `aircraft_camera`; its scripted calibration is
     independent of the robot's camera link.
4. **Serve a world for the synthetic target.** Coraplex only fetches and syncs
   in REAL mode. For simulation, a process has to serve the world
   (`FetchWorldServer`): Giskard or a stand-in, and Coraplex fetches from it
   even though no real robot is involved.
5. **Coraplex holds back updates.** The demo script creates its
   `WorldSynchronizer` with `defer_incoming_updates=True` and calls
   `apply_missed_messages()` between plans.
6. **The capture time reaches Coraplex.** Each accepted aircraft observation
   carries its capture time into the shared world, for example as a field on
   the aircraft annotation, so Coraplex can tell a fresh pose from an old one.
   Not yet checked: whether `WorldSynchronizer` publishes a changed annotation
   field. If it does not, RoboKudo publishes a small "capture at T accepted,
   parts seen: …" message instead.
7. **Split each task into plans at its perception points.** The demo script
   runs the plans one after another and, between two of them, does what
   [Perception points between plans](#perception-points-between-plans)
   describes. Today `acrambly/tracy_real.py:130-138` builds one plan per task
   and calls `perform()` once.
8. **Real camera: gate the pipeline.** The real aircraft analysis engine starts
   with a gate, so NOCTIS and FoundationPose run once per trigger rather than
   on every frame (see [GPU load with a real camera](#gpu-load-with-a-real-camera)).
   `QueryAnnotator` can be triggered from the Coraplex process over the
   existing query action. Its reply is ignored, since the result arrives
   through synchronization.
9. **Plans read parts from the world, not from RoboKudo queries.** For example
   `world.get_semantic_annotations_by_type(TopWing)[0].root`.
   - In REAL mode, `DetectAction` and `DetectBeforeGrasp` always use
     `RoboKudoPerception` (hard-wired at
     `coraplex/src/coraplex/robot_plans/motions/misc.py:67`), so aircraft plans
     leave both out. This removes the double write through `apply_to`.
10. **Make the aircraft types importable in the Coraplex process.** Install
   `robokudo_acrambly` in Coraplex's environment so incoming annotations can be
   deserialized. Only the plan script imports
   `robokudo_acrambly.aircraft.types`; the `coraplex` package stays independent
   of it.

## Order of work

The work is split in two. Part 1 needs no Coraplex plan and can be built and
tested now. Part 2 depends on decisions only the Coraplex action plan makes:
where the perception points are, which parts each plan needs, and how retries
work.

### Part 1: now (RoboKudo side)

| Step | What |
|---|---|
| 1 | tests: two synchronized worlds, held-back updates, capture time readable on the receiving side |
| 2 | RoboKudo fetches the shared world, sets up the tracker, attaches the synchronizer |
| 3 | `world_frame` points at the shared root |
| 4 | world server for the synthetic target |
| 6 | capture time reaches the receiving world, including the open check whether `WorldSynchronizer` publishes a changed annotation field |

Done when a second process with `defer_incoming_updates=True` receives the 12
parts with the right UUIDs, types, poses and capture times, and sees them only
after `apply_missed_messages()`.

### Part 2: after the Coraplex action plan exists

| Step | What |
|---|---|
| 5 | the demo script holds back updates |
| 7 | tasks split into plans at perception points; trigger, wait and timeout |
| 9 | plans read parts from the world |
| 10 | aircraft types installed in the Coraplex process |

### Real-camera work, later

- Step 8, the gate on the pipeline: needed only once NOCTIS and FoundationPose
  are wired in.
- The camera link's second parent: a `robokudo/` change (**outside**).

### Rules for the split

- Part 1 makes RoboKudo a writer in the robot's world. Until part 2 and the
  real-camera risks are resolved, run it only against the synthetic target or
  a stand-in world server, never against a real Giskard.
- The capture-time format is where part 1 meets part 2. Keep it simple and
  general so the Coraplex plan does not force a rewrite.
- These open questions stay open until part 2: belief vs the robot's own
  action, two writers on one part, and when to skip the wait.
- The two rear wheels question is a catalog change inside
  `robokudo_acrambly`. Decide it before part 1's tests are written, so they do
  not have to be rewritten.
- With `/add-plan-item`, this becomes two items, part 2 depending on part 1.

## Changes

| Directory | Change | Size |
|---|---|---|
| `robokudo_acrambly/` | fetch, switch world, tracker, synchronizer; `world_frame`; capture time on accepted observations; gate on the real analysis engine | small-medium |
| `acrambly/` (**outside**) | demo, world server for simulation, deferral flag and apply calls; tasks split into plans at perception points; trigger, wait and timeout between plans | small-medium |
| `test/robokudo_acrambly_test/` (**outside**) | synchronization and capture-time tests | small |
| `coraplex/` | none, as long as every perception point falls between plans | none |
| `robokudo/` | none for the synthetic target | none |

Changes only some situations need:

- **A perception point that cannot become a plan boundary**, i.e. one that has
  to sit in the middle of one plan's motion: a `coraplex/` plan node that
  waits and applies updates (**outside**, small). For sequential assembly
  steps this is not expected; it is the fallback, not the plan.
- **Real camera:** `setup_world_for_camera_frame`
  (`robokudo/src/robokudo/world.py:188`) would give the robot's camera link a
  second parent; a small `robokudo/` change (**outside**).

The scope of PR 13's spec lists "synchronization startup or transport" and
"Coraplex plans" as out of scope, so this is new work. `/add-plan-item` can
decide whether it becomes its own item.

## Pros

- One world, one identity (the belief UUID), one geometry.
- Giskard gets collision geometry for every part RoboKudo has accepted.
- Parts that are out of view still exist, and several copies of a kind work.
- Little new code; almost every piece already exists.
- The world no longer changes under a running Coraplex plan.
- At a perception point the plan gets a pose captured after the robot
  settled, and with a gated pipeline the GPU runs only then.
- It fits PR 13's brief: "`WorldSynchronizer` transfers accepted changes to
  another process's world".

## Cons

- Coraplex sees updates only between plans. A plan acts on the pose from the
  capture taken before it started, and nothing newer.
- Each perception point costs one full NOCTIS + FoundationPose pass of waiting.
- Tasks have to be split into plans at their perception points, and retries
  (a timed-out observation, a failed grasp that needs another look) move from
  the plan into the demo script.
- A perception point in the middle of one plan's motion needs a `coraplex/`
  change.

Holding back updates only changes *when* Coraplex applies what RoboKudo
publishes. RoboKudo still publishes every change at once, and the held-back
changes still land in full. So these remain:

1. **Real camera: the camera link gets a second parent.** A real camera stores
   its transform through `setup_world_for_camera_frame`
   (`robokudo/src/robokudo/world.py:188`, called from
   `robokudo/src/robokudo/io/camera_interface.py:108`). In the shared world
   the camera link already has a parent from the robot model; the function
   finds no `{camera}_T_{world}` connection and adds a second one, which
   reaches Giskard at once. Not yet checked against the real robot model.
2. **Two writers on one part.** After Coraplex attaches a held part to the
   gripper, RoboKudo's next accepted pose update is only delayed; it still
   overwrites the pose when Coraplex applies it (open question below).
3. **RoboKudo becomes a writer** in the robot's world. A perception bug
   reaches motion planning, one apply step later.

## Perception points between plans

A perception point is where the next motion needs a part's current pose, for
example before a grasp. Applying whatever updates have arrived is not enough
there: they may come from a capture taken before the robot last moved the
part, or while the arm or camera was still moving. So the demo script waits
for a fresh observation.

The demo script runs a task as plans, one after another:

```
plan A (e.g. move to the viewing pose)  → perform()
trigger capture → wait for capture_time > end of A → apply_missed_messages()
build plan B (e.g. grasp the TopWing)   → perform()
...
```

1. A plan ends when the robot reaches the viewing pose. The moment `perform()`
   returns is the settle time: the motion is done.
2. The script triggers one capture on the gated pipeline.
3. It waits until an update arrives, for each part the next plan needs, with a
   capture time after the settle time.
4. It calls `apply_missed_messages()`.
5. It builds the next plan from the world *after* applying. A plan built
   earlier may have read the old pose.

Rules:

- **Timeout.** An occluded part never produces a fresh observation. After a
  configured number of captures the script fails explicitly or falls back to
  the last belief under the rule in
  [Belief vs the robot's own action](#belief-vs-the-robots-own-action). It
  never hangs.
- **Skip the wait** when nothing relevant changed since the last accepted
  capture: the robot did not touch the part and the camera did not move.
- **Wait only for the parts the next plan needs**, not all 12.
- Splitting plans does not solve the two-writers problem: a part held across a
  plan boundary is still overwritten by RoboKudo's next pose when the updates
  are applied.

Depends on step 6 (capture time reaches Coraplex) and, with a real camera,
step 8 (a gate on the pipeline).

## GPU load with a real camera

With a real camera, a pipeline tick runs camera frame → NOCTIS segmentation
(GPU) → FoundationPose 6D pose (GPU) → `SemanticDigitalTwinConnector`
acceptance (CPU) → `WorldSynchronizer` publish. Acceptance starts no GPU work.
How busy the GPU is depends only on how often the pipeline ticks:

| Pipeline | GPU |
|---|---|
| Continuous, like `scripted_aircraft.py` today | busy all the time; NOCTIS and FoundationPose run back to back on every frame |
| Gated (`QueryAnnotator`, `PipelineTrigger`) | idle between triggers; the models stay loaded in GPU memory |

Holding back updates in Coraplex does not reduce GPU load: RoboKudo still
computes and publishes every tick, and most results are overwritten before
Coraplex applies them. With a continuous pipeline a fresh observation also
takes up to about two cycles (the frame in flight was captured before the
settle time), against exactly one with a gate.

Recommendation: gate the real aircraft pipeline (step 8). The GPU then runs one
NOCTIS + FoundationPose pass per perception point.

Later option: FoundationPose tracking mode (one full detection, then cheap
per-frame tracking) would shorten each pass. It needs earlier poses linked to
persistent hypotheses, which does not exist yet
(see [synchronization research sources](synchronization_research_sources.md)).

## Risk

**Clearing would wipe the robot's world.** `unsafe_clear_world`
(`robokudo/src/robokudo/world.py:153`) empties the world. With a synchronizer
attached, the removal is published, and Coraplex applies it at its next chosen
moment.

- `init_world_with_entity_tracker` and `clear_world` call it; only tests call
  those today.
- `world_instance` calls it only when the world is `None`, which does not
  happen in normal use.
- Step 2 installs the fetched world with `set_world` and
  `init_world_entity_tracker_from_world` and never takes that path.

## The two rear wheels

The aircraft has two identical rear wheels. The catalog has two kinds,
`RearWheelLeft` and `RearWheelRight`, whose meshes differ by 3 mm in width
(26 mm vs 29 mm, measured with trimesh), which a camera can't reliably tell
apart. PR 13 never lets one kind match a belief of another kind, so a single
mislabel produces a wrong or duplicate belief.

In this approach each wheel is its own body with its own belief UUID, and
RoboKudo's one-to-one matching keeps them apart by position. The plan picks a
wheel from `get_semantic_annotations_by_type(RearWheel)`; with identical wheels
either one fits.

Still a risk: if the robot moves one wheel far, its match falls below
`confidence_threshold` (default `0.05`) and RoboKudo creates a third wheel
belief. The fix must use the robot's own record of where it put the wheel.

## Open questions

### When does Coraplex apply held-back updates?

Recommendation: only between plans, and the task is split so that every
perception point falls between two plans
(see [Perception points between plans](#perception-points-between-plans)).
This needs no `coraplex/` change.

Fallback: a perception point that has to stay inside one plan
(`DetectAction`, `DetectBeforeGrasp`) needs a `coraplex/` plan node that does
the same trigger, wait and apply. `DetectingMotion` is built so that "a motion
planned after it binds the corrected pose".

### Belief vs the robot's own action

Example: the robot moves the TopWing while the camera can't see it. PR 13 keeps
the last accepted pose, which is now wrong.

Recommendation: the robot's own action wins over an older observation. A belief
is used only if its last accepted capture time is after the robot last moved
that part. RoboKudo should not write the pose of a part the robot is holding.

Edge case: a part moved far may fall below `confidence_threshold`, so RoboKudo
creates a second belief of the same kind. Possible fix inside
`robokudo_acrambly`: aircraft configuration of the threshold. Not decided.

### First target

Recommendation: the synthetic target first (scripted aircraft source and
connector in a RoboKudo process, Coraplex in simulation in a second process
over ROS), then the real robot with Giskard. The real robot depends on the
camera-link con and the clearing risk being resolved.

### The two rear wheels: one kind or two?

- (a) One kind, `RearWheel`, with two physical copies. "Left" and "right"
  become where each wheel goes in the assembly, which the plan decides later.
- (b) Keep `RearWheelLeft` and `RearWheelRight`, and trust perception to tell
  them apart.

Recommendation: (a). It changes PR 13's catalog and types in
`robokudo_acrambly/aircraft/` (inside) and brings back the two-copies test
that PR 13's spec left out.

### Permission to write the plan outside `robokudo_acrambly`

Proposal: `.scratch/coraplex-belief-consumption/` with `spec.md`, `map.md`, and
tickets; `GLOSSARY.md` or `docs/adr/0004-…` only if a decision really needs it.
Documents only. Not yet granted.
