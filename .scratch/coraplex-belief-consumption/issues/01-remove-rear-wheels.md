# 01: Remove the rear wheels from the perception catalog

Status: ready-for-agent
Type: task
Blocked by: None

Implementation permission: not granted.

## Outcome

The aircraft perception catalog has 10 parts. `RearWheelLeft` and
`RearWheelRight` no longer exist as semantic types, labels or models. Follow the
[specification](../spec.md#rear-wheels).

## Acceptance criteria

- [ ] Before changing code, check how the Tetris PR left the aircraft catalog and
  remove the wheels wherever the catalog now lives.
- [ ] `RearWheelLeft` and `RearWheelRight` leave `aircraft/types.py`; their
  `AircraftLabel` and `AircraftModel` members leave `aircraft/models.py`; the
  docstring that counts the models matches the catalog.
- [ ] Their rows leave `test/robokudo_acrambly_test/dataset/aircraft_models.csv`.
- [ ] Tests that cover every model kind run with 10 kinds, and their expected
  values come from the catalog, not from a retyped count.
- [ ] Every STL, IPT and mesh-report file stays.
- [ ] `aircraft_models.md`, `semantic_digital_twin_connector.md` (including its
  parts table), `scripted_aircraft.md` and `aircraft_belief_verification.md`
  say 10 parts. The discussion record and PR 13's spec, tickets and PR text stay
  as they are.

## Verification

Run the aircraft model, belief creation and acceptance tests in
`test/robokudo_acrambly_test` with `--orm-build never` and a memory cap.

## Comments

- 2026-11-21: Specified in the planning session; no code written.
