# 03: Create typed beliefs with complete mesh geometry

Status: claimed
Type: task
Blocked by: 01, 02

Implementation permission: granted by the user's explicit ticket 03 implementation
request on 2026-10-28. Other tickets remain outside this authorization.

## Outcome

A validated unmatched hypothesis creates one persistent belief, mesh-bearing
body, and semantic annotation through existing world APIs.

Follow the [specification](../spec.md) and use the application configuration from
ticket 01 through the generic extension point from ticket 02.

## Acceptance criteria

- [ ] Initialize visual and collision geometry from the configured model before
  adding its body to the local world.
- [ ] Keep the body frame at the mesh origin and retain model-local offsets and
  explicit metric scale in both collections.
- [ ] Register the body, belief, and correctly typed semantic annotation once,
  retaining the classification label and persistent body UUID.
- [ ] Accept hypotheses containing a model label and supported pose without
  requiring a per-observation mesh or bounding box.
- [ ] Prepare required geometry before persistent mutation so missing model
  resources do not leave partially created beliefs.
- [ ] Preserve existing generic bounding-box creation when no model geometry is
  configured.
- [ ] Add failing generic creation/geometry tests first, then application tests
  connecting actual aircraft model definitions to the acceptance interface.

## Verification

Use existing world/CAS fixtures and deterministic local geometry for focused
checks. Verify the geometry present at insertion as well as final state. Run
generic and application tests in separate memory-capped pytest batches with
`--orm-build never`; format modified Python files and record results.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
