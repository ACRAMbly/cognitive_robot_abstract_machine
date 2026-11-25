# 02: Run the aircraft engine on a fetched, synchronized world

Status: done
Type: task
Blocked by: 01

Implementation permission: granted 2026-11-25.

## Outcome

The aircraft engine gets the robot's world from a world source, installs it,
and publishes every new part and accepted pose through a `WorldSynchronizer`. A
receiving world sees the same parts, and with held-back updates sees them only
after `apply_missed_messages()`. Follow the
[specification](../spec.md#world-source) and
[ADR 0001](../../../docs/adr/0001-accept-beliefs-into-the-shared-robot-world.md).

## Acceptance criteria

- [ ] Test first: two worlds from one source, a synchronizer on each, one
  accepted aircraft observation in the RoboKudo world. The receiving world has
  the same UUIDs, semantic types and poses.
- [ ] Test first: with `defer_incoming_updates=True` on the receiving side, the
  receiving world is unchanged until `apply_missed_messages()` runs.
- [ ] An abstract world source (explicit superclass) supplies the world; one
  implementation fetches it with `fetch_world_from_service`, one hands over a
  world built by the caller.
- [ ] The engine installs the supplied world with `set_world` and
  `init_world_entity_tracker_from_world`, so `world_entity_tracker` follows the
  new world, and attaches one `WorldSynchronizer` on the normal topic.
- [ ] No path after the synchronizer is attached clears the world.
- [ ] The engine no longer creates its own world; there is no local mode.
- [ ] Existing aircraft tests get a world with root `map` through one shared
  fixture in `test/robokudo_acrambly_test/conftest.py`. Their bodies stay as they
  are.
- [ ] Names follow `AGENTS.md`: what the thing is, not how it does it.

## Verification

Run the new synchronization tests and all aircraft tests in
`test/robokudo_acrambly_test` with `--orm-build never` and a memory cap. The
synchronization tests use the `rclpy_node` fixture in one pytest process.

## Comments

- 2026-11-21: Specified in the planning session; no code written. The user
  approved changing the existing aircraft tests through one shared fixture.
