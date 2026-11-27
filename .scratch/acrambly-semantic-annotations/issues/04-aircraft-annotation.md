# Aircraft annotation

Status: claimed
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
