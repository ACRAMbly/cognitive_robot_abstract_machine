# Aircraft beliefs in the shared robot world (part 1)

Status: part 1 done

Implementation permission: granted 2026-11-25 for part 1. The user first asked on 2026-11-21 for the
specification and tickets only, with no code.

## Problem

After PR 13, RoboKudo accepts the aircraft parts into its own local belief world,
which no other process can see. Coraplex has to use those beliefs while it
executes a plan, and Giskard needs their collision geometry. The two worlds
cannot be synchronized as they are, because sync messages identify entities by
UUID and the worlds share no root
([ADR 0001](../../docs/adr/0001-accept-beliefs-into-the-shared-robot-world.md)).

## Outcome and boundary

The aircraft engine fetches the robot's world from a world server and accepts
beliefs directly into it. A `WorldSynchronizer` publishes every new part and
every accepted pose change. After each accepted capture, RoboKudo publishes an
accepted capture message beside the world
([ADR 0004](../../docs/adr/0004-publish-capture-times-beside-the-world.md)).

Done when a receiving world with its own `WorldSynchronizer` and
`defer_incoming_updates=True`, in the same pytest process, receives the 10
aircraft parts with the right UUIDs, types and poses, sees them only after
`apply_missed_messages()`, and can tell from the accepted capture messages when
each part was last captured.

The design and its reasons are in
[coraplex_belief_consumption.md](../../robokudo_acrambly/doc/coraplex_belief_consumption.md)
("Part 1" and "Decided in the planning session"). The
[glossary](../../GLOSSARY.md) defines *accepted capture*, *capture time*,
*perception point* and *settle time*.

### In scope

- The aircraft engine in `robokudo_acrambly`.
- A stand-in world server in `acrambly_temp/`, which stands in for `acrambly/`
  in this PR.
- Tests in `test/robokudo_acrambly_test/`.
- Removal of the two rear wheels from the perception catalog.

### Out of scope

- Part 2: holding back updates in the Coraplex demo, splitting tasks into plans
  at perception points, plans reading parts from the world, and installing the
  aircraft types in Coraplex's environment.
- The real camera: gating the pipeline, and the camera link's second parent.
- The Tetris engine, which keeps its local belief world until a later PR.
- `stacking_robokudo.py`.
- Any change in `robokudo/`, `semantic_digital_twin/`, `giskardpy/`, `coraplex/`
  or `acrambly/`.

## Starting point

Branch from `main` after PR 13 and the Tetris PR are merged. The file and line
references in the design doc and in these tickets were read before the Tetris
PR; check each one again. The Tetris PR may have reshaped the aircraft catalog
(its prompt asks whether to share one catalog between aircraft and Tetris).

## Ownership

| Owner | Responsibility |
| --- | --- |
| `robokudo_acrambly` | world source abstraction and its two implementations; aircraft engine startup on a fetched world; world frame from the fetched root; accepted capture message, publishing annotator and receiver; catalog without rear wheels |
| `acrambly_temp/` | stand-in world server: a world with root `map`, served by `FetchWorldServer`, with its own `WorldSynchronizer` |
| Existing `semantic_digital_twin` interfaces | `fetch_world_from_service`, `FetchWorldServer`, `WorldSynchronizer`, `StreamPosition`, `has_applied` |
| Existing `robokudo` interfaces | `set_world`, `init_world_entity_tracker_from_world`, `SemanticDigitalTwinConnector`, belief tracking |
| `test/robokudo_acrambly_test/` | one shared fixture giving the engine a world with root `map`; synchronization, held-back update and accepted capture tests |

Generic RoboKudo and Semantic Digital Twin modules never import
`robokudo_acrambly` ([ADR 0003](../../docs/adr/0003-keep-aircraft-semantics-in-acrambly.md)).

## Contracts

### World source

The aircraft engine depends on an abstract world source, an explicit superclass
(not a `Protocol`), that supplies the robot's world before the pipeline is built.

- The production implementation fetches the world with
  `fetch_world_from_service` over a ROS node.
- The test implementation hands over a world the test built, with root `map`.
- After the world is supplied, the engine installs it with `set_world` and
  `init_world_entity_tracker_from_world`, and attaches one `WorldSynchronizer`
  on the normal `/semantic_digital_twin/world_sync` topic when a node is
  available.
- The engine never creates its own world and never calls a function that
  clears the world (`unsafe_clear_world`, `clear_world`,
  `init_world_with_entity_tracker`) once the synchronizer is attached.

### World frame

- The aircraft world frame is the root of the fetched world. `world_frame`
  leaves `aircraft_frames.json` and `AircraftFrames`.
- No extra frame body is added under the root.
- The camera frame and the scripted camera-to-world calibration stay as they
  are.

### Accepted capture

| Field | Contract |
| --- | --- |
| capture time | the CAS capture time (`cas.data_timestamp`), integer Unix nanoseconds |
| accepted parts | belief UUIDs of the parts accepted from this capture; parts in view but not accepted are not listed |
| stream position | RoboKudo's `WorldSynchronizer.latest_published_position`, read after the accepted poses were published |

- One message per capture from which at least one observation was accepted.
- An `AcceptedCapture` dataclass, sent as `std_msgs/String` holding JSON from
  `DataclassJSONSerializer`. The topic name is a `StrEnum` member. Volatile QoS.
- The publishing annotator runs after the connector in the aircraft pipeline.
  It finds the accepted beliefs as those whose latest hypothesis is one of the
  current CAS's object hypotheses, and it shares the synchronizer's node.
- The receiver records the newest accepted capture per part UUID (capture time
  and stream position), with no history. A receiver started late knows only the
  captures published after it started.
- A capture counts as applied in a receiving world only when that world's
  synchronizer reports `has_applied(stream position)`.
- The capture time is never written into the world.

### Stand-in world server

- Builds a world with one root body named `map`, serves it with
  `FetchWorldServer`, and attaches its own `WorldSynchronizer`, so a process that
  fetches later gets the parts already accepted.
- Run only against this stand-in or the synthetic target. Never run part 1
  against a real Giskard.

### Inspection

- `AircraftBeliefInspection` fetches its world from the stand-in.
- When no world server answers within a timeout, it raises a custom
  `DataclassException` whose `suggest_correction` names the stand-in script.
  It never starts a server itself, and never waits without a timeout.

### Rear wheels

- `RearWheelLeft`, `RearWheelRight`, their `AircraftLabel` and `AircraftModel`
  members, and their rows in `test/robokudo_acrambly_test/dataset/aircraft_models.csv`
  are removed. The catalog has 10 parts.
- Every STL, IPT and mesh-report file stays.
- `aircraft_models.md`, `semantic_digital_twin_connector.md`,
  `scripted_aircraft.md` and `aircraft_belief_verification.md` change to 10
  parts. The discussion record and PR 13's spec, tickets and PR text stay as
  they are.

## Testing

- Test first for every ticket, with pytest, following `AGENTS.md`.
- Existing aircraft tests get their world through one shared fixture in
  `conftest.py`; their bodies stay as they are. The user approved this change.
- Synchronization tests run in one pytest process with the `rclpy_node` fixture
  from `test/conftest.py`, modelled on
  `test/semantic_digital_twin_test/test_ros/test_world_synchronizer.py` and
  `test_world_fetcher.py`. No test needs a live external service.
- Run each package's tests on their own with `--orm-build never`, a memory cap,
  and a check of free memory first. Never run the whole suite at once.

## Tickets

| Ticket | Outcome | Blocked by |
| --- | --- | --- |
| [01](issues/01-remove-rear-wheels.md) | the catalog has 10 parts | none |
| [02](issues/02-aircraft-engine-on-a-fetched-world.md) | the engine runs on a fetched, synchronized world | 01 |
| [03](issues/03-world-frame-from-fetched-root.md) | the world frame is the fetched root | 02 |
| [04](issues/04-accepted-captures.md) | accepted capture messages are published and received | 02 |
| [05](issues/05-stand-in-world-server.md) | a stand-in world server, and inspection that needs it | 02, 03 |
| [06](issues/06-documentation-and-regressions.md) | docs and focused regression runs | 01, 02, 03, 04, 05 |
