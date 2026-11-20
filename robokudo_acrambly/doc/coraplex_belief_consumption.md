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
  (`apply_missed_messages()`) at moments it chooses, for example between plans.

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

Not there yet: no RoboKudo code does the fetch and sync, and no demo holds back
updates.

## Steps

1. **Write the failing test first.** In `test/robokudo_acrambly_test`: two
   worlds fetched from one source, a synchronizer on each, one accepted
   aircraft observation in the RoboKudo world. Assert that the receiving world
   has the same UUIDs, types, and poses. A second test holds back updates on
   the receiving side and asserts the world is unchanged until
   `apply_missed_messages()` runs.
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
6. **Plans read parts from the world, not from RoboKudo queries.** For example
   `world.get_semantic_annotations_by_type(TopWing)[0].root`.
   - In REAL mode, `DetectAction` and `DetectBeforeGrasp` always use
     `RoboKudoPerception` (hard-wired at
     `coraplex/src/coraplex/robot_plans/motions/misc.py:67`), so aircraft plans
     leave both out. This removes the double write through `apply_to`.
7. **Make the aircraft types importable in the Coraplex process.** Install
   `robokudo_acrambly` in Coraplex's environment so incoming annotations can be
   deserialized. Only the plan script imports
   `robokudo_acrambly.aircraft.types`; the `coraplex` package stays independent
   of it.

## Changes

| Directory | Change | Size |
|---|---|---|
| `robokudo_acrambly/` | fetch, switch world, tracker, synchronizer; `world_frame` | small-medium |
| `acrambly/` (**outside**) | demo, world server for simulation, deferral flag and apply calls | small |
| `test/robokudo_acrambly_test/` (**outside**) | synchronization tests | small |
| `coraplex/` | none, as long as updates are applied between plans | none |
| `robokudo/` | none for the synthetic target | none |

Changes only some situations need:

- **Applying updates inside a running plan**, e.g. at a `DetectAction`: a
  `coraplex/` plan node (**outside**, small).
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
- It fits PR 13's brief: "`WorldSynchronizer` transfers accepted changes to
  another process's world".

## Cons

- Coraplex sees updates only at chosen moments, so a plan may act on a pose one
  perception cycle old.
- Fine-grained moments (inside a plan) need a `coraplex/` change.

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

- Between plans: no `coraplex/` change.
- At perception points inside a plan (`DetectAction`, `DetectBeforeGrasp`):
  needs a `coraplex/` plan node. `DetectingMotion` is built so that "a motion
  planned after it binds the corrected pose".

Starting point: between plans.

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
