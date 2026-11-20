# How Coraplex uses RoboKudo's aircraft-part beliefs: discussion record

Status: design discussion in progress. No implementation is requested.

The user's goal is to choose among approaches and write down a plan, not to
implement. Any change to a file outside `robokudo_acrambly/` must be mentioned
explicitly and approved by the user, even when it is the best approach.

## Starting point

[PR 13](https://github.com/ACRAMbly/cognitive_robot_abstract_machine/pull/13)
(`feature/local-world-belief-updates`) accepts CAS object hypotheses of the 12
aircraft parts into RoboKudo's local belief world. Each part has one persistent
belief, body, UUID, semantic annotation, mesh geometry, and accepted pose. These
survive movement, absence, and reappearance.

Question: what are the best approaches going forward, so that Coraplex can use
those beliefs while it executes a plan?

Context read:

- `.scratch/local-world-belief-updates/spec.md`
- `.scratch/local-world-belief-updates/map.md`
- [connector brief](semantic_digital_twin_connector.md)
- `docs/adr/0003-keep-aircraft-semantics-in-acrambly.md`
- `GLOSSARY.md`

## What the code says today

### How the worlds relate

| Process | World it holds | Relation |
|---|---|---|
| Giskard | the robot's world | the original copy (serves `fetch_world`, `giskardpy/src/giskardpy/middleware/ros2/giskard.py:210`) |
| Coraplex | the robot's world | a copy of Giskard's, kept identical by `WorldSynchronizer` (`coraplex/src/coraplex/demonstrations.py:275-276`) |
| RoboKudo | the local belief world | separate: its own root, frames, and UUIDs, not synchronized with anything (`robokudo/src/robokudo/world.py:47`) |

So there are two worlds, not three. Coraplex and Giskard hold copies of one
world in two processes, and a plan runs against exactly one world (the one in
its `Context`). PR 13 left the transfer to other processes out on purpose
(connector brief, "Goal and boundary").

In simulation (not `ExecutionType.REAL`), the Coraplex demo builds its own world
instead of fetching Giskard's.

### How Coraplex perceives

- Call chain: `DetectAction` → `DetectingMotion` → `PerceptionTask` →
  `PerceptionInterface.for_execution_type`.
- The source is hard-wired and can't be replaced:
  `coraplex/src/coraplex/robot_plans/motions/misc.py:67`.
  - Real run: `RoboKudoPerception`, which calls the `robokudo/query` action.
  - Any other run: `WorldPerception`, which reads Coraplex's own world.
- `Detection.apply_to` (`coraplex/src/coraplex/perception.py`) only moves a body
  that already exists.
  - It raises `PerceivedObjectNotInWorld` if there is no body of that type.
  - It raises `AmbiguousDetection` if there is more than one body of that type.
- Matching compares `type.lower()` with the class name in lower case. The
  aircraft labels already equal the class names (`TopWing` becomes `topwing`),
  so the names line up.
- `DetectBeforeGrasp` (`coraplex/src/coraplex/robot_plans/plan_transformations.py`)
  inserts a `LookAtAction` and a `DetectAction` before the final approach of a
  reach.

### What RoboKudo returns to a query

- `GenerateQueryResult` (`robokudo/src/robokudo/annotators/query.py:283`)
  answers from the current frame's object hypotheses, not from beliefs.
- The `uid` it reports is not the belief UUID.
- Result: a query can't see a part that is currently out of view, even though
  PR 13 keeps its belief.
- Queries are slow: `acrambly/sub_parts/real/cube_perception.py:85-87` notes
  about 10 s per query.

### What a world sync can carry

- The JSON serializer stores the full class name and imports the class on the
  receiving side (`krrood/src/krrood/adapters/json_serializer.py:150`).
- `Mesh.to_json` includes the vertices.
- So a receiving process only needs `robokudo_acrambly` installed.
  `robokudo_acrambly/aircraft/types.py` imports only `semantic_digital_twin`.

### Pieces that already exist

- `fetch_world_from_service` and `WorldSynchronizer` in `semantic_digital_twin`.
  Coraplex demos already use them together.
- `FetchWorldServer`, with a configurable service name.
- `WorldSynchronizer` with a configurable `topic_name`, and
  `defer_incoming_updates`, which holds incoming updates back until the owner
  applies them.
- `robokudo.world.set_world()` and `init_world_entity_tracker_from_world()`,
  used by `robokudo/src/robokudo/io/semdt_raytracer_camera_interface.py:145` and
  `robokudo/src/robokudo/annotators/world_descriptor_bootstrap.py:105`.
- `World.merge_world` and `World.merge_world_at_pose`
  (`semantic_digital_twin/src/semantic_digital_twin/world.py:1775`). Both copy a
  whole world, not one part.
- `spawn_body` in `acrambly/sub_parts/shared/utils.py:27`.
- The connector's `confidence_threshold` (default `0.05`) is a descriptor
  parameter. Low-similarity matches create new beliefs.

## Decisions so far

### Q1: terms (settled)

Question: PR 13 *accepts* object hypotheses into the *local belief world*, while
"synchronization" means copying a world to another process. Should a new term,
"execution world", name the world a plan runs against?

The user asked whether "execution world" is needed at all, since a plan may
involve several worlds (RoboKudo, Coraplex, Giskard). The answer was that
Coraplex and Giskard hold copies of one world, so "execution world" would be
misleading. The user then chose to describe what happens instead of inventing a
term.

**Decision:** no new glossary term. The plan writes things out, for example
"Coraplex's world, which `WorldSynchronizer` keeps identical with Giskard's" and
"RoboKudo's local belief world, which nothing synchronizes". The existing terms
local belief world, acceptance, and synchronization stay as they are.

## Q2: who decides a part exists in Coraplex's world? (open)

The user added option E3, "RoboKudo starts from the shared world", and asked
for all approaches in detail. Each approach lists how it works, what already
exists, the changes and their directories, and pros and cons.

> Edit freely: delete the options you rule out.

### E1: the plan decides (create the parts from the catalog; perception only corrects poses)

**How it works.**

- Before planning, the demo creates all 12 parts in Coraplex's world from
  `AircraftModelCatalog` in `robokudo_acrambly`.
- The existing sync forwards them to Giskard.
- During the plan, `DetectAction` → `RoboKudoPerception` asks `robokudo/query`.
- `Detection.apply_to` moves the matching part.
- RoboKudo's local belief world stays private.

**Already exists:** almost all of it. The Coraplex query path, `apply_to`, the
type matching, and the spawn helpers are all there. The cube and Tetris tasks
in `acrambly/sub_parts/real/` use this pattern today.

**Changes:**

- `robokudo_acrambly/`: a query-answering step at the end of the aircraft
  pipeline. It should answer from beliefs (belief UUID, last accepted pose)
  rather than from the current frame. *Medium-small.*
- `acrambly/sub_parts/`: an aircraft task that creates the parts and builds the
  plan. **Outside `robokudo_acrambly`.** *Small.*
- `coraplex/`: none.

**Pros:**

- Least risk, and no Coraplex change.
- The plan controls when perception happens.
- The robot's world never changes behind the plan's back.

**Cons:**

- Two independent records of each part: the belief (UUID A) and Coraplex's part
  (UUID B). Nothing links them.
- The plan has to know in advance which parts are present.
- Only poses cross the process boundary.
- Several copies of one kind fail with `AmbiguousDetection`.
- Queries are slow (about 10 s each).

### E2: RoboKudo decides in its own world, and the plan copies parts over at perception points

**How it works.**

- RoboKudo keeps its local belief world and shares it under its own name:
  - continuously, with a `WorldSynchronizer` on a separate topic, or
  - on demand, with a `FetchWorldServer` on a separate service name.
- Coraplex holds that belief world as a second `World`.
- At each `DetectAction`, a new perception source finds the queried part there
  and copies it into Coraplex's world:
  - if the part is missing, it creates it with the same UUID, mesh, and annotation;
  - otherwise it only moves it.
- The existing sync forwards the change to Giskard.

**Already exists:** the transport and the merge code.

**Missing:**

- a pluggable perception source in Coraplex (it's hard-wired at
  `coraplex/src/coraplex/robot_plans/motions/misc.py:67`);
- the step that copies one part over (`merge_world` copies a whole world).

**Changes:**

- `robokudo_acrambly/`: start the belief-world publisher or fetch server.
  *Small.*
- `coraplex/src/coraplex/perception.py` and
  `coraplex/src/coraplex/robot_plans/motions/misc.py`: make the perception
  source pluggable, and add the belief-world source with the copy step.
  **Outside.** *Medium-large.*
  - Putting this source in `robokudo_acrambly` instead would make
    `robokudo_acrambly` depend on `coraplex`. That is a new dependency and needs
    approval.
- `acrambly/`: the demo. **Outside.** *Small.*

**Pros:**

- One identity (the belief UUID) and one geometry.
- The plan controls timing.
- Parts that are out of view still exist.
- Several copies of a kind work.
- Giskard gets collision geometry when a part is copied over.

**Cons:**

- The most new code, much of it in `coraplex`.
- Two worlds per Coraplex process.
- The copy step has to handle parts the robot has moved since (Q4).

### E3: RoboKudo starts from the shared world (user's proposal)

**How it works.**

- At startup, RoboKudo fetches Giskard's world with `fetch_world_from_service`.
- It makes that world its own, with `set_world` and
  `init_world_entity_tracker_from_world`.
- It attaches a `WorldSynchronizer` on the normal topic.
- PR 13's connector then accepts beliefs directly into that world.
- Each new part (UUID, mesh, annotation) and each accepted pose change reaches
  Giskard and Coraplex automatically.
- RoboKudo's separate belief world goes away; the beliefs live in the one world
  everyone shares.

**Already exists:** nearly all the pieces.

- The fetch, the world switch, the synchronizer, and Giskard's fetch server are
  all there.
- Coraplex already does the same fetch-then-sync.
- In simulation, `WorldPerception` reads Coraplex's own world, so the synced
  parts are visible with no Coraplex change.
- In a real run, the existing `robokudo/query` path would also move the parts.
  The detailed steps below avoid that by not using it for aircraft parts.
- Not there yet: no RoboKudo code does the fetch and sync.

**Changes:**

- `robokudo_acrambly/`: at startup, fetch the world, switch to it, and attach
  the synchronizer, in the aircraft analysis engine or the inspection launcher.
  *Small to medium.*
- Possibly `robokudo/src/robokudo/world.py` or
  `robokudo/src/robokudo/io/camera_interface.py`, depending on the risks below.
  **Outside.** *Small, but in generic code.*
- `acrambly/`: the demo. **Outside.** *Small.*
- `coraplex/`: none to start with. A later change would be needed only if
  updates must wait for perception points.

**Pros:**

- One world, one identity, one geometry.
- Giskard can avoid collisions with every part RoboKudo has seen.
- The least new code that still uses PR 13's beliefs fully.
- Parts that are out of view still exist, and several copies of a kind work.
- It fits PR 13's brief: "`WorldSynchronizer` transfers accepted changes to
  another process's world".

**Cons and risks:**

1. **Clearing would wipe the robot's world.** `unsafe_clear_world`
   (`robokudo/src/robokudo/world.py:150`) clears the world, and both
   `init_world_with_entity_tracker` and `world_instance` call it. With a
   synchronizer attached, that removal is published to Giskard. Startup must
   never take that path.
2. **Frames.** `setup_world_for_camera_frame`
   (`robokudo/src/robokudo/io/camera_interface.py:108`) and the aircraft engine
   add world and camera frame bodies if they are missing.
   - In the shared world, those names must match Giskard's frames. Otherwise
     RoboKudo publishes stray frames.
   - The scripted source's explicit camera calibration could also disagree with
     the robot model's camera link.
3. **Two writers on one part.** After Coraplex attaches a held part to the
   gripper, RoboKudo's next accepted pose update would still write that part's
   pose (see Q4).
4. **Changes arrive during motions.** Every accepted observation changes the
   world mid-motion (see Q3). `WorldSynchronizer.defer_incoming_updates` exists
   for this, but Coraplex doesn't use it.
5. **RoboKudo becomes a writer** in the robot's world. A perception bug now
   affects motion planning too.

### E3 in detail: minimal steps (proposal added by the user)

The proposal below was added by the user. It works out E3 step by step. A
review against the code follows it.

The simplest route is to share one world through the `WorldSynchronizer` that
already exists, rather than going through RoboKudo's query action. Coraplex
already does its half of this in REAL mode
(`coraplex/src/coraplex/demonstrations.py:275-276`). RoboKudo needs the
matching half.

**Why the RoboKudo world can't just be synced as it is.**

- Sync messages identify bodies and connections by UUID (`original_parent_id`
  and similar in `world_modification.py`). Coraplex can only apply RoboKudo's
  changes if both worlds start from the same world and the same root UUIDs.
- Right now RoboKudo starts from an empty `World()` and creates its own
  `aircraft_world` root (`robokudo/world.py:48`, `:188`). Syncing that into
  Coraplex would try to add a second, unconnected root.
- The query action path (`RoboKudoPerception` → `Detection.apply_to`) doesn't
  help either. RoboKudo answers queries from the current frame's object
  hypotheses, not from the stored beliefs. `apply_to` also needs Coraplex to
  have preloaded the part and then moves the body itself, so both processes
  would be writing the same pose.

**Minimal steps.**

1. **RoboKudo starts from the shared world.** Where the aircraft engine starts
   up (for example `AircraftBeliefInspection.__enter__` in
   `aircraft_observations/inspection.py`), do this before the pipeline runs:

   ```python
   set_world(fetch_world_from_service(node=node))
   WorldSynchronizer(_world=world_instance(), node=node)
   ```

   This is the same pair Coraplex calls, and both processes fetch from the same
   world service.
2. **Point the aircraft world frame at the shared root.** In
   `aircraft_frames.json`, change `world_frame` from `aircraft_world` to the
   shared world's root name, e.g. `map`. `setup_world_for_camera_frame` then
   reuses that body instead of creating a new root.
   - For the synthetic source, keep `aircraft_camera` as a separate frame; it
     syncs along with everything else.
   - For a real camera that is a robot link, that function must not add a
     second parent connection to the link. Check this before going there.
3. **Coraplex runs in REAL mode as it already does** (fetch the world, attach a
   `WorldSynchronizer`). Every accepted belief then appears in Coraplex's world
   as its own body, UUID, and `AircraftPart` annotation, and its pose keeps
   updating.
4. **Plans read the parts from the world instead of querying RoboKudo.** For
   example `world.get_semantic_annotations_by_type(TopWing)[0].root`, or
   `WorldPerception`. Don't use `RoboKudoPerception` / `Detection.apply_to` for
   aircraft parts, because RoboKudo now owns those poses.
5. **Make the aircraft types importable in the Coraplex process.** Install
   `robokudo_acrambly` in Coraplex's environment so incoming annotations can be
   deserialized. Only the plan script imports `robokudo_acrambly.aircraft.types`;
   the `coraplex` package itself stays independent of it.
6. **Write the failing test first.** It goes in `test/robokudo_acrambly_test`:
   two worlds fetched from one source, a synchronizer on each, one accepted
   aircraft observation in the RoboKudo world. Then assert that the receiving
   world has the same UUIDs, types, and poses.

**Things to watch.**

- Updates arrive while a plan is running. If a plan step needs the world to
  hold still, `WorldSynchronizer` has `defer_incoming_updates` and `resume()`.
- The spec lists "synchronization startup or transport" and "Coraplex plans" as
  out of scope, so this is new work rather than part of PR 13. `/add-plan-item`
  can decide whether it becomes its own item.

None of these steps has been run; they come from reading the code.

#### Review of the steps against the code

The proposal matches the code in all but one place (point 2). Its steps also
leave out three things (points 3, 4, and 6).

1. **Confirmed.** `robokudo/src/robokudo/world.py:48` starts from an empty
   `World()`. `setup_world_for_camera_frame` (`world.py:188`) adds a missing
   world-frame body with `add_body`, which creates a second root.
   `original_parent_id` is in
   `semantic_digital_twin/src/semantic_digital_twin/world_description/world_modification.py:204`.
   `apply_missed_messages()` (`world_synchronizer.py:665`) and `resume()`
   (`world_synchronizer.py:724`) exist.
2. **Correction (step 2).** The aircraft code never calls
   `setup_world_for_camera_frame`. The aircraft engine
   (`robokudo_acrambly/src/robokudo_acrambly/descriptors/analysis_engines/scripted_aircraft.py:99-107`)
   adds a missing world frame *under the existing root* with a
   `FixedConnection`. With a fetched world, `aircraft_world` would therefore
   hang under `map` rather than become a second root. Setting `world_frame` to
   `map` is still cleaner, because no extra frame gets published to Giskard.
   - The aircraft camera frame looks like CAS metadata with an explicit
     camera-to-world transform, not a body. This still needs to be confirmed
     before the real-camera case.
3. **Missing from step 1: the entity tracker.** `set_world` replaces the world
   but not `world_entity_tracker`, which keeps tracking the old empty world.
   `robokudo/src/robokudo/io/storage.py:260` and
   `robokudo/src/robokudo/io/cas_view_codecs.py:526` read the tracker. Step 1
   also needs `init_world_entity_tracker_from_world(world_instance())`. This is
   an existing function, so the call stays inside `robokudo_acrambly`.
4. **Missing from steps 3 and 4, and related to Q6.**
   - Coraplex only fetches and syncs in REAL mode. In simulation it builds its
     own world (`coraplex/src/coraplex/demonstrations.py:263-273`).
   - The synthetic first target therefore needs a process that serves the
     world (`FetchWorldServer`): Giskard, or a stand-in.
   - Coraplex has to fetch from it even though no real robot is involved.
5. **Consequence of step 4.** In REAL mode, `DetectAction` and the
   `DetectBeforeGrasp` transformation always use `RoboKudoPerception` (it's
   hard-wired at `coraplex/src/coraplex/robot_plans/motions/misc.py:67`).
   - Aircraft plans must therefore leave both out. Otherwise a `coraplex/`
     change is needed.
   - This removes the double write through `apply_to`.
   - It does not remove the conflict once Coraplex attaches a held part to the
     gripper (E3 risk 3, Q4).
6. **Still applies, not mentioned in the steps:** the clearing risk (E3 risk 1).
   `unsafe_clear_world` must never run once the synchronizer is attached.
7. **Files touched outside `robokudo_acrambly/` by these steps:**
   - `test/robokudo_acrambly_test/` (step 6): it is at the repository root,
     not inside `robokudo_acrambly/`, so it needs approval.
   - The Coraplex plan script, e.g. in `acrambly/`.
   - Installing `robokudo_acrambly` into Coraplex's environment (step 5).

### E4: same process

**How it works.** The RoboKudo pipeline runs inside the Coraplex process, with
`robokudo.world.set_world(coraplex_world)`. In effect it's E3 without the
transfer between processes.

**Already exists:** `set_world`. The PR 13 tests already run the pipeline
in-process.

**Changes:**

- `acrambly/`: the demo. **Outside.**
- `robokudo_acrambly/`: an embeddable launcher.
- *Small.*

**Pros:**

- Quickest to show working in simulation.
- Easy to test.

**Cons:**

- It doesn't carry over to the real robot, where RoboKudo is its own ROS process.
- RoboKudo's heavy dependencies (open3d, py_trees) and its threads move into the
  plan's process.
- Same risks as E3 (clearing, frames, writers), minus the transport.

### Summary

| | Who creates the part | Identity | Geometry reaches Giskard | Already exists | New code | Main code dirs outside `robokudo_acrambly` |
|---|---|---|---|---|---|---|
| E1 | Plan, from the catalog | split | yes, from the catalog | most | small | `acrambly/` |
| E2 | RoboKudo, copied over by the plan | belief UUID | yes, when copied over | transport only | medium-large | `coraplex/`, `acrambly/` |
| E3 | RoboKudo, in the shared world | belief UUID | yes, as soon as accepted | nearly all pieces | small-medium | `acrambly/`, maybe `robokudo/` |
| E4 | RoboKudo, same `World` | belief UUID | yes | `set_world` | small | `acrambly/` |

### Recommendation for Q2

**E3: RoboKudo starts from the shared world.**

- It uses PR 13's beliefs fully with the least new code, and almost everything
  it needs already exists.
- Its risks are specific and can be tested.
- Risks 1 and 2 (clearing, frames) become requirements for the first ticket.
- Risks 3 and 4 (two writers, changes during motions) are decided next, as Q3
  and Q4.
- Fallback: E2, since the transport it needs already exists. E2 puts most of its
  new code in `coraplex`.

## Open questions

### Q3: when do beliefs reach Coraplex's world?

- (a) Continuously: every accepted observation changes the world, including
  during motions.
- (b) Only at perception points in the plan (`DetectAction`, `DetectBeforeGrasp`).

Recommendation from the first round: (b). `DetectingMotion` is built so that
"a motion planned after it binds the corrected pose". A world that changes
during a motion can move a held part under the gripper.

Under E3, (b) would mean holding back incoming updates
(`defer_incoming_updates`) until perception points, which requires a
`coraplex/` change. Choosing (a) with E3 needs no Coraplex change, but has to
solve risk 4.

### Q4: belief vs the robot's own action

Example: the robot moves the TopWing while the camera can't see it. PR 13 keeps
the last accepted pose, which is now wrong.

Recommendation: the robot's own action wins over an older observation. A belief
is used only if its last accepted capture time is after the robot last moved
that part.

Related (E3 risk 3): RoboKudo should not write the pose of a part the robot is
holding.

Related edge case (for a later round): a part moved far by the robot may fall
below `confidence_threshold`. RoboKudo then creates a *second* belief of the
same kind, and Coraplex sees two TopWings. Possible fix inside
`robokudo_acrambly`: aircraft configuration of the threshold. Not decided yet.

### Q5: identity

Recommendation: Coraplex finds parts by belief UUID, not by type, so that
several copies of one kind (`Bolt.stl`, `Nut.stl` in `acrambly/assets/battat/`)
work later. E2, E3, and E4 already settle this. It is only open if E1 is chosen.

### Q6: first target

- (a) Synthetic: the scripted aircraft source and connector in a RoboKudo
  process, with Coraplex executing in simulation in a second process over ROS.
- (b) The real robot with Giskard.
- (c) Both at once.

Recommendation: (a), then (b).

### Q7: permission to write the plan outside `robokudo_acrambly`

Proposal:

- `.scratch/coraplex-belief-consumption/` with `spec.md`, `map.md`, and tickets
- `GLOSSARY.md` or `docs/adr/0004-…`, only if a decision really needs it

These are documents only, no code. Not yet granted. This discussion file is in
`robokudo_acrambly/doc/` for that reason.
