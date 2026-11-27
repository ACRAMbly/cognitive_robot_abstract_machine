# Aircraft annotation

Status: resolved
Type: task
Blocked by: 03

Add the `Aircraft` annotation: rooted at the body of its `Chassis`, with one
field per other aircraft part, typed as that part's class or `None`,
defaulting to `None`, and marked with `IsPartWholeRelationship` so mounting a
part fills its field.

Tests, test-first:

- an aircraft with no mounted parts besides its chassis is valid;
- mounting a part fills exactly that part's field;
- the part-whole relation finder of `semantic_digital_twin`
  (`part_finding/relationships.py`) reports the mounted parts as parts of the
  aircraft.

Done when the tests pass and `krrood check` is clean on the new code.

## Comments

## Answer

Built in `d182638fcf` and merged in `fdbb264645`. `Aircraft(HasRootBody,
PartWholeRelationship)` is rooted at the chassis body. It has eleven optional
part fields marked with `IsPartWholeRelationship`, and `aircraft.add(part)`
fills each one inside `modify_world()`. The tests cover three cases: an
aircraft with only its chassis, mounting a part filling exactly its field, and
`PartRelationshipInspector` reporting the mounted parts. All 32 tests in
`test/semantic_digital_twin_acrambly_test` pass.

Open: `Aircraft` has no `chassis` field. The `Chassis` annotation sits on
`aircraft.root`.
