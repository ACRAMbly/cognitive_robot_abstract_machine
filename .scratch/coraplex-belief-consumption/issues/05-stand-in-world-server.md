# 05: Stand-in world server, and inspection that needs it

Status: ready-for-agent
Type: task
Blocked by: 02, 03

Implementation permission: not granted.

## Outcome

A stand-in world server in `acrambly_temp/` serves a world with root `map`, and
the aircraft inspection tool runs against it. Without a server, inspection stops
with a clear error. Follow the
[specification](../spec.md#stand-in-world-server).

## Acceptance criteria

- [ ] Test first: a world fetched from the stand-in has root `map`; a world
  fetched after a part was accepted contains that part.
- [ ] Test first: inspection with no world server raises a custom
  `DataclassException` after a timeout, and its `suggest_correction` names the
  stand-in script.
- [ ] The stand-in builds the world, serves it with `FetchWorldServer`, and
  attaches its own `WorldSynchronizer`.
- [ ] `acrambly_temp/` stands in for `acrambly/`; nothing in `acrambly/` changes.
- [ ] `AircraftBeliefInspection` fetches its world through the production world
  source; it never starts a server itself and never waits without a timeout.
- [ ] The timeout is a parameter or dataclass field with a default, not a
  module-level constant.

## Verification

Run the stand-in and inspection tests with `--orm-build never` and a memory cap.
Then run the stand-in and the inspection by hand on the Linux machine and check
the parts in RViz.

## Comments

- 2026-11-21: Specified in the planning session; no code written. Never run
  this against a real Giskard.
