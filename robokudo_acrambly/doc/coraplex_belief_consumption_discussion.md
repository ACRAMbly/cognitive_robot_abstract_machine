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

**Connector and synchronizer:**

- `SemanticDigitalTwinConnector`: **used**, in RoboKudo. It keeps building
  RoboKudo's private local belief world, and the query answer reads from it.
- `WorldSynchronizer`: **not used by RoboKudo.**
  - Only the existing Coraplex ↔ Giskard synchronizer runs, unchanged.
  - Beliefs reach Coraplex as poses in `robokudo/query` answers, never as
    world changes.

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

**Connector and synchronizer:**

- `SemanticDigitalTwinConnector`: **used**, in RoboKudo, filling its own local
  belief world.
- `WorldSynchronizer`: **used twice.**
  - A *second* synchronizer on its own topic publishes RoboKudo's belief world
    into the copy Coraplex holds.
  - The existing Coraplex ↔ Giskard synchronizer forwards each part copied over.
  - In the on-demand variant, a `FetchWorldServer` on its own service name
    replaces the second synchronizer.

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

**Connector and synchronizer:**

- `SemanticDigitalTwinConnector`: **used**, in RoboKudo. It accepts beliefs
  directly into the shared world instead of a private one.
- `WorldSynchronizer`: **used.**
  - RoboKudo attaches one on the normal `/semantic_digital_twin/world_sync`
    topic, the same one Coraplex and Giskard already use.
  - Every accepted belief change is published to both.

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
  `robokudo/src/robokudo/io/camera_interface.py`, depending on the cons and risk below.
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

**Cons** (these happen however carefully E3 is used; each needs code, a
design decision, or has to be accepted):

1. **Real camera: the camera link gets a second parent.** A real camera stores
   its transform through `setup_world_for_camera_frame`
   (`robokudo/src/robokudo/world.py:188`, called from
   `robokudo/src/robokudo/io/camera_interface.py:108`).
   - In the shared world, the camera link already has a parent from the robot
     model.
   - The function finds no `{camera}_T_{world}` connection, so it adds a second
     one to the link and publishes it to Giskard.
   - The fix is a small `robokudo/` change. Not yet checked against the real
     robot model.
2. **Two writers on one part.** After Coraplex attaches a held part to the
   gripper, RoboKudo's next accepted pose update would still write that part's
   pose (see Q4).
3. **Changes arrive during motions.** Every accepted observation changes the
   world mid-motion (see Q3). `WorldSynchronizer.defer_incoming_updates` exists
   for this, but Coraplex doesn't use it.
4. **RoboKudo becomes a writer** in the robot's world. A perception bug now
   affects motion planning too.

**Risk** (nothing goes wrong as long as the code is careful):

- **Clearing would wipe the robot's world.** `unsafe_clear_world`
  (`robokudo/src/robokudo/world.py:153`) empties the world. With a
  synchronizer attached, that removal is published to Giskard and Coraplex.
  - `init_world_with_entity_tracker` and `clear_world` call it. Only tests
    call those today.
  - `world_instance` calls it only when the world is `None`, which does not
    happen in normal use.
  - Startup must install the fetched world with `set_world` and
    `init_world_entity_tracker_from_world`, and never call a clear function
    once the synchronizer is attached.

**Keep in mind (not a risk):** frame names in the synthetic setup.

- Set `world_frame` to the shared root, e.g. `map` (step 2 of the detailed
  proposal), so the aircraft engine adds no stray world frame.
- The synthetic source's scripted camera calibration is independent of the
  robot's camera link.

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

**Con.** Updates arrive while a plan is running. If a plan step needs the
world to hold still, `WorldSynchronizer` has `defer_incoming_updates` and
`resume()`, which Coraplex would have to call (E3 con 3).

**Risk.** Clearing the world once the synchronizer is attached (E3 risk). The
steps above avoid it by installing the fetched world with `set_world`.

**Scope.** The spec lists "synchronization startup or transport" and "Coraplex plans" as
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
     gripper (E3 con 2, Q4).
6. **Still applies:** the clearing risk (E3 risk). `unsafe_clear_world` must
   never run once the synchronizer is attached.
7. **Files touched outside `robokudo_acrambly/` by these steps:**
   - `test/robokudo_acrambly_test/` (step 6): it is at the repository root,
     not inside `robokudo_acrambly/`, so it needs approval.
   - The Coraplex plan script, e.g. in `acrambly/`.
   - Installing `robokudo_acrambly` into Coraplex's environment (step 5).

### E4: same process

**How it works.** The RoboKudo pipeline runs inside the Coraplex process, with
`robokudo.world.set_world(coraplex_world)`. In effect it's E3 without the
transfer between processes.

**Connector and synchronizer:**

- `SemanticDigitalTwinConnector`: **used**, in the Coraplex process. It writes
  straight into Coraplex's `World` object.
- `WorldSynchronizer`: **not needed between RoboKudo and Coraplex**, because
  they share one object.
  - The existing Coraplex ↔ Giskard synchronizer still forwards the parts to
    Giskard in a REAL run.
  - In simulation there is no synchronizer at all.

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

### Connector and synchronizer use

| | `SemanticDigitalTwinConnector` | `WorldSynchronizer` between RoboKudo and Coraplex | Existing Coraplex ↔ Giskard synchronizer |
|---|---|---|---|
| E1 | used, private belief world | not used (poses travel in `robokudo/query` answers) | used, unchanged |
| E2 | used, private belief world | used, on a separate topic (or a `FetchWorldServer` instead) | used, forwards each part copied over |
| E3 | used, writes into the shared world | used, on the normal topic | used, unchanged |
| E3b | used, writes into the shared world | used, on the normal topic; Coraplex holds updates back | used, unchanged |
| E4 | used, writes into Coraplex's `World` object | not needed (same process, same object) | used in REAL runs only |

In every approach, the connector stays as PR 13 built it. The approaches
differ only in which world it writes into and how that world reaches Coraplex.

### The two rear wheels

The user stated that the aircraft has two identical rear wheels, and that this
must be handled.

What the code and resources say:

- The catalog has two kinds, `RearWheelLeft` and `RearWheelRight`, with
  different STL files (`plane/RearWheelLeft.stl`, `plane/RearWheelRight.stl`).
- The two meshes differ only slightly:

  | Mesh | Width | Height × depth | Volume | Faces |
  |---|---|---|---|---|
  | `RearWheelLeft.stl` | 26 mm | 40 × 40 mm | 7.55 cm³ | 2266 |
  | `RearWheelRight.stl` | 29 mm | 40 × 40 mm | 6.93 cm³ | 2146 |

  - Measured with trimesh on the checked-in files.
  - A camera cannot reliably tell a 3 mm width difference apart.
- PR 13 treats different kinds as incompatible for matching. If perception
  mislabels left as right, the right belief can never be corrected. Instead, a
  wrong or duplicate belief is created.
- PR 13's spec explicitly left out the case of two physical copies of one model.

So "left" and "right" probably describe where a wheel goes in the assembly, not
a kind of part perception can see. The wheels need one of these:

- one kind, `RearWheel`, with two physical copies, each with its own belief and
  UUID; or
- two kinds that perception is trusted to tell apart.

This is open as Q8.

What the wheels mean for each approach:

- **E1:** Coraplex finds parts by type. Two `RearWheel` bodies raise
  `AmbiguousDetection`, and `accept_first_if_multiple` may move the wrong wheel.
  `Detection` has no UUID field, so a fix needs a `coraplex/` change.
- **E2, E3, E3b, E4:** each wheel is its own body with its own belief UUID.
  RoboKudo's one-to-one matching keeps them apart by position. The plan picks a
  wheel from `get_semantic_annotations_by_type(RearWheel)`; with identical
  wheels, either one fits.
- **Still a risk for all of them:** the robot moves one wheel far.
  - Its match falls below `confidence_threshold`, so RoboKudo creates a third
    wheel belief (Q4 edge case).
  - With two identical copies, "always match the only belief of this kind"
    can't fix this. The fix must use the robot's own record of where it put
    the wheel.

### E3b: shared world, with Coraplex holding back updates

This is E3 plus one change: the Coraplex demo script creates its
`WorldSynchronizer` with `defer_incoming_updates=True`. It applies the held-back
updates (`apply_missed_messages()`) at moments it chooses, for example between
plan steps.

**Connector and synchronizer:**

- `SemanticDigitalTwinConnector`: **used**, in RoboKudo, as in E3.
- `WorldSynchronizer`: **used**, as in E3.
  - The difference is Coraplex's own synchronizer, created with
    `defer_incoming_updates=True`, so received changes wait until the demo
    applies them.

**Already exists:**

- Giskard already does exactly this: `giskardpy/src/giskardpy/middleware/ros2/giskard.py:208`
  and `giskardpy/src/giskardpy/middleware/ros2/world_updates.py:62-70`.
- So the motion controller is safe in E3 and E3b alike.
- The demo scripts already create their own synchronizer, e.g.
  `acrambly/tracy_real.py`.

**Changes:** E3's changes, plus a flag and the calls to apply updates in the
demo script (`acrambly/`, **outside**).

- Applying updates *inside* a running plan, at a `DetectAction`, would also
  need a `coraplex/` plan node (**outside**).
- Applying them between plans needs no `coraplex/` change.

**Pros:**

- Everything E3 has.
- The world no longer changes under a running Coraplex plan.

**Cons:**

- Updates arrive only at chosen moments, so the plan may act on a pose one
  perception cycle old.
- Fine-grained moments need a `coraplex/` change.

**Cons and risk kept from E3:**

Holding back updates only changes *when* Coraplex applies what RoboKudo
publishes. RoboKudo still publishes every change at once, and the held-back
changes still land in full. So these stay:

1. **Clearing would wipe the robot's world** (E3 risk). A clear is still
   published, and Coraplex applies the removal at its next chosen moment.
   Startup must still never take that path.
2. **Real camera: the camera link gets a second parent** (E3 con 1). The
   extra connection reaches Giskard as soon as RoboKudo publishes it.
3. **Two writers on one part** (E3 con 2). RoboKudo's pose update for a held
   part is only delayed. It still overwrites the pose when Coraplex applies it
   (see Q4).
4. **RoboKudo becomes a writer** in the robot's world (E3 con 4). A perception
   bug still reaches motion planning, one apply step later.

E3 con 3, changes arriving during motions, is the one E3b removes.

### Scores and ranking

Each criterion is scored 1 (poor) to 5 (good) and multiplied by its weight.
The maximum is 90.

| Criterion | Weight | E1 | E2 | E3 | E3b | E4 |
|---|---|---|---|---|---|---|
| Two identical rear wheels (several copies of a kind) | 3 | 1 | 4 | 4 | 4 | 4 |
| Uses PR 13's beliefs (identity, geometry, absence) | 2 | 2 | 5 | 5 | 5 | 5 |
| Object tracking reaches Coraplex (one identity over time, pose updates) | 2 | 2 | 3 | 5 | 4 | 5 |
| Little new code / already exists | 2 | 4 | 2 | 4 | 4 | 5 |
| Few changes outside `robokudo_acrambly` and `acrambly` | 2 | 5 | 1 | 4 | 4 | 5 |
| Plan safety (no changes mid-motion, no two writers) | 3 | 4 | 5 | 2 | 4 | 2 |
| Path to the real robot | 2 | 4 | 4 | 3 | 3 | 1 |
| Giskard gets collision geometry | 1 | 3 | 4 | 5 | 5 | 3 |
| Testable without hardware | 1 | 3 | 3 | 4 | 4 | 5 |
| **Total** |  | **55** | **64** | **69** | **73** | **68** |

Ranking:

1. **E3b, shared world with Coraplex holding back updates (73).** It has E3's
   strengths, and the plan-safety problem is solved by an option Giskard
   already uses. Tracked poses arrive whenever the demo applies them.
2. **E3, shared world (69).** The least work and full tracking, but the world
   changes under a running plan.
3. **E4, same process (68).** The quickest demo in simulation, with full
   tracking, but no path to the real robot.
4. **E2, belief mirror copied over at perception points (64).** The safest and
   cleanest identity, but tracked poses arrive only at perception points, and
   most of its new code sits in `coraplex/`.
5. **E1, the plan decides (55).** It can't handle the two identical wheels
   without a `coraplex/` change, and it loses the tracked identity.

Notes on the scores:

- Object tracking: RoboKudo tracks every part the same way in all approaches.
  PR 13's connector matches each detection to an existing belief, so a part
  keeps one belief UUID across perception cycles. The criterion scores how much
  of that tracking reaches Coraplex.
  - E3 and E4: every accepted pose update reaches Coraplex as it happens, under
    the belief UUID (5).
  - E3b: the same, but Coraplex sees the tracked state only when the demo
    applies the held-back updates (4).
  - E2: Coraplex sees the belief UUID, but poses only at perception points (3).
  - E1: Coraplex gets a pose per query and matches it by type, so the tracked
    identity is lost and two copies of a kind can be swapped (2).
  - Not covered by any approach: a part the robot moves or holds (Q4). Its
    tracking can break, and RoboKudo may create a second belief for it.
- Changes in `acrambly/` count as inside, like changes in `robokudo_acrambly/`.
  - E1 and E4 change only those two directories, so they score 5.
  - E3 and E3b may still touch generic `robokudo/` code (clearing, real camera).
  - E3b's 4 assumes it applies updates between plans. Applying them inside a
    running plan needs a `coraplex/` plan node and lowers it to 3 (total 71).
  - E2's new code is mostly in `coraplex/`, so it stays at 1.

- E2 and E3 are close. If changes inside `coraplex/` become acceptable, E2
  rises to about E3b's total.
- The "path to the real robot" scores for E3/E3b depend on two things still
  unchecked:
  - the real camera link's second parent connection (step 2 of the detailed
    proposal);
  - the clearing risk.
- The weights are a proposal; change them, and the totals follow.

### Amount of change per approach

Not part of the ranking; it shows where each approach's code goes and how much.
Sizes come from each approach's **Changes** list and are scored none 0,
small 1, small-medium 2, medium 3, medium-large 4, large 5. Lower totals mean
less change.

| Directory | E1 | E2 | E3 | E3b | E4 |
|---|---|---|---|---|---|
| `robokudo_acrambly/` | 2 (query answer from beliefs) | 1 (publisher or fetch server) | 2 (fetch, switch world, tracker, synchronizer) | 2 (as E3) | 1 (embeddable launcher) |
| `acrambly/` | 1 (aircraft task) | 1 (demo) | 1 (demo, world server for simulation) | 1 (as E3, plus the deferral flag and apply calls) | 1 (demo) |
| `coraplex/` | 0 | 4 (pluggable perception source, copy step) | 0 | 0 | 0 |
| `robokudo/` | 0 | 0 | 0 | 0 | 0 |
| **Total, synthetic target** | **3** | **6** | **3** | **3** | **2** |

Changes that only some situations need:

- **E1, two identical rear wheels:** `Detection` needs a UUID field, so a
  `coraplex/` change.
- **E3 and E3b, real camera:** `setup_world_for_camera_frame`
  (`robokudo/src/robokudo/world.py:188`) would give the robot's camera link a
  second parent, so a small `robokudo/` change (+1).
- **E3b, updates inside a running plan:** a `coraplex/` plan node that applies
  held-back updates at a `DetectAction`, about small (+1).
- **E4, real robot:** no path, so no amount can be given.

`robokudo/src/robokudo/io/camera_interface.py` needs no change in any approach,
and `world.py` needs none for the synthetic target: `set_world` and
`init_world_entity_tracker_from_world` already exist, and nothing on the
aircraft path calls the clearing functions.

### Recommendation for Q2

> Updated after scoring: the recommendation is now **E3b**. It is E3 below,
> with the Coraplex demo holding back incoming updates as described above.


**E3: RoboKudo starts from the shared world.**

- It uses PR 13's beliefs fully with the least new code, and almost everything
  it needs already exists.
- Its risks are specific and can be tested.
- Risks 1 and 2 (clearing, real camera) become requirements for the first ticket.
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
solve E3 con 3.

### Q4: belief vs the robot's own action

Example: the robot moves the TopWing while the camera can't see it. PR 13 keeps
the last accepted pose, which is now wrong.

Recommendation: the robot's own action wins over an older observation. A belief
is used only if its last accepted capture time is after the robot last moved
that part.

Related (E3 con 2): RoboKudo should not write the pose of a part the robot is
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

### Q8: the two rear wheels, one kind or two?

The two rear-wheel meshes differ by only 3 mm in width (see "The two rear
wheels" above), and the user describes the wheels as identical.

- (a) One kind, `RearWheel`, with two physical copies. "Left" and "right" become
  where each wheel goes in the assembly, which the plan decides later.
- (b) Keep `RearWheelLeft` and `RearWheelRight`, and trust perception to tell
  them apart.

Recommendation: (a). Perception can't reliably tell the two apart, and PR 13
never lets one kind match a belief of the other kind. So with (b), a single
mislabel produces a wrong or duplicate belief.

(a) changes PR 13's catalog and types in `robokudo_acrambly/aircraft/`
(inside). It also brings back the two-copies test that PR 13's spec left out.
