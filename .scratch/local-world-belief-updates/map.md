# Local object belief updates

Status: ready-for-agent

Implementation permission: granted for tickets 01, 02, and 03 by explicit user
requests on 2026-10-28. Other tickets still require explicit implementation
permission.

## Notes

- Scope brief: [semantic digital twin connector](../../robokudo_acrambly/doc/semantic_digital_twin_connector.md).
- Specification: [persistent aircraft-part beliefs](spec.md).
- Inspected checkout: `main`, commit `e09436a30`, initially clean working tree.
- The checkout already includes the scripted Tetris source. The previous
  brief's notes about its absence from `main` were corrected; plane acceptance
  remains independent of that source.
- [Plane collision geometry](../../plane/collision_geometry_design.md) has a
  separate confirmed plan whose implementation has not been authorized.
- The grilling and domain-modeling session established the requirements. The
  user's subsequent instruction authorizes specification writing and expressly
  requires separate permission before implementation.

## Decisions-so-far

- [Ticket 01](issues/01-aircraft-types-and-models.md#answer) is implemented with
  application-owned aircraft types and replaceable model resources. Its focused
  suite passes; broader regression limitations are recorded in the ticket.
- [Ticket 02](issues/02-observation-validation-and-frames.md#answer) adds generic
  configured semantic acceptance, validation before belief changes, and precise
  camera/world pose preparation. Its 76 connector cases and 41 aircraft model
  cases pass; broader regression limitations are recorded in the ticket.
- [Ticket 03](issues/03-typed-belief-creation-and-geometry.md#answer) adds configured
  mesh-bearing belief creation and typed annotation registration. Geometry is
  complete before insertion, and resource failures preserve persistent state.
  All 85 focused generic cases and 42 aircraft cases pass; broader regression
  limitations are recorded in the ticket.

Requirements retained from the brief:

- Accept CAS object hypotheses into RoboKudo's local belief world; cross-process
  synchronization is separate.
- Preserve persistent object identity, accepted geometry, and the last accepted
  state while an object is absent.
- Detection identifiers and model identifiers are not physical-object identities.
- Reuse the existing association and world-update machinery.
- Exercise creation, translation, rotation, partial absence, empty observations,
  and reappearance through deterministic local pipeline tests.

User clarification, 2026-10-28:

- This task concerns the plane meshes; Tetris belongs to the previous task.
- Later simplified meshes will supply both visual and collision geometry. This
  supersedes the brief's detailed CAD visual mesh plus enclosing collision box
  requirement for this task, and differs from the earlier plane plan's choice
  to retain detailed visual meshes.
- No mesh simplification work has been requested in this connector interview.
- Plane-part semantic annotations belong in `robokudo_acrambly`, rather than
  generic `robokudo`. This also settles Q3: semantic annotations are in scope.
- Each detected physical plane part has its own persistent belief, UUID, and
  independently updated pose (Q2 confirmed).
- The existing dependency direction is `robokudo_acrambly` to `robokudo` and
  `semantic_digital_twin`. Generic packages must not import plane-part types
  from the application package.
- The generic connector accepts a configurable classification-label-to-semantic-
  type mapping supplied by `robokudo_acrambly` (Q4 confirmed).
- A common `AircraftPart` base has distinct semantic types for the 12 named mesh
  parts (Q5 confirmed).
- Plane hypotheses identify their model and observed pose. Application
  configuration supplies model geometry, loaded once per configured model;
  observations need not carry the mesh (Q6 confirmed).
- The same configured model resource supplies visual and collision geometry;
  subsequent pose updates retain that geometry.
- Distinct known model kinds are incompatible for identity association; model
  identity never substitutes for a physical object's UUID (Q7 confirmed).
- Missing, unknown, or conflicting model labels are invalid in the configured
  plane pipeline (Q8 confirmed).
- An invalid pose or missing required capture/frame/transform metadata rejects
  the observation before any persistent belief changes (Q9 confirmed).
- Acceptance tests cover all 12 part kinds through creation, translation,
  rotation, partial absence, total absence, and reappearance. The additional
  case with two physical copies of the same model is not required (Q10).
- Observation poses locate the mesh origin. Plain poses are camera-relative;
  stamped poses can use the declared camera or world frame. Transform camera
  poses exactly once and retain poses already in world coordinates (Q11).
- This change covers types, labels, bodies, geometry, and poses. Grasp
  definitions and assembly relationships remain later work (Q12).
- Use the original STL meshes for both visual and collision geometry now,
  replacing configured resources with simplified meshes later (Q13).

See [the application-semantics decision](../../docs/adr/0003-keep-aircraft-semantics-in-acrambly.md).

## Verified implementation gaps

- The existing connector associates current hypotheses with persistent beliefs;
  rejected assignments and excess hypotheses create beliefs, while absent
  beliefs remain unchanged.
- Current association compares image regions, world poses, and bounding-box
  dimensions. Classification labels do not constrain matches.
- World updates use a bounding-box pose, ignore mesh-bearing shape annotations,
  replace visual geometry with a box, and do not initialize collision geometry.
  A plain pose alone cannot move an existing belief body.
- Configured creation now registers semantic annotations from classifications;
  unconfigured generic creation remains untyped.
- These are code findings, not decisions to add new tracking, confidence,
  expiry, or durable-storage policies.

Resolved vocabulary is recorded in [the glossary](../../GLOSSARY.md).

## Design tree status

All requirement questions have answers. The updated
[implementation brief](../../robokudo_acrambly/doc/semantic_digital_twin_connector.md)
records the agreed scope. The user authorized specifications and subsequently
implemented tickets 01, 02, and 03. Remaining implementation tickets require separate
explicit permission.

- Q1, answered: plane meshes are the current target, with future simplified
  meshes used for both visual and collision geometry.
- Q2, answered: each physical part is an independent persistent object belief.
- Q3, answered: plane-part semantic annotations are in scope and belong in
  `robokudo_acrambly`.
- Q4, answered: application-supplied configurable mapping in the generic connector.
- Q5, answered: 12 distinct part types under `AircraftPart`.
- Q6, answered: application-owned model meshes; hypotheses need not carry meshes.
- Q7, answered: distinct known model kinds cannot match.
- Q8, answered: missing, unknown, and conflicting model labels are invalid.
- Q9, answered: validate the whole observation before changing beliefs.
- Q10, answered: all 12 part kinds through the six-stage sequence; omit the
  separate test with two physical copies of one model.
- Q11, answered: mesh-origin poses; camera-relative plain poses; stamped
  camera/world poses; convert camera coordinates exactly once.
- Q12, answered: grasp definitions and assembly relationships remain later work.
- Q13, answered: original STL meshes serve both geometry roles until simplified
  resources are supplied.
- No aircraft-part semantic types were found in the inspected semantic digital
  twin and RoboKudo source. RoboKudo's existing `Plane` annotation denotes a
  geometric plane, not an aircraft.

## Implementation tickets

Tickets 01, 02, and 03 are implemented; the remaining tickets are specified and unclaimed. Dependency readiness is separate from
the user-permission requirement above.

| Ticket | Dependencies | Status |
| --- | --- | --- |
| [01: Aircraft types and model resources](issues/01-aircraft-types-and-models.md) | None | resolved |
| [02: Observation validation and frames](issues/02-observation-validation-and-frames.md) | None | resolved |
| [03: Typed belief creation and geometry](issues/03-typed-belief-creation-and-geometry.md) | 01, 02 | resolved |
| [04: Association and pose updates](issues/04-association-and-pose-updates.md) | 02, 03 | ready-for-agent |
| [05: Plane pipeline acceptance](issues/05-plane-pipeline-acceptance.md) | 01, 02, 03, 04 | ready-for-agent |
| [06: Documentation and regressions](issues/06-documentation-and-regressions.md) | 01, 02, 03, 04, 05 | ready-for-agent |

## Fog

No requirement questions remain open. Tickets 01, 02, and 03 record their implemented
interfaces and verification results. Inspect the then-current checkout and
environment when the remaining implementation tickets are explicitly authorized.

## Comments

- 2026-10-28: User instruction: proceed to specifications; do not start
  implementation without explicit permission. Created the specification and six
  implementation tickets without writing production code, tests, or fixtures.

- 2026-10-28: User authorized ticket 01 implementation and confirmed its public
  test seams. [Ticket 01](issues/01-aircraft-types-and-models.md#answer) now
  supplies aircraft types and model resources; focused coverage passes, with
  broader regression limitations recorded in its answer. Other tickets remain
  unimplemented.
- 2026-10-28: User authorized ticket 02 and confirmed its public test boundaries.
  [Ticket 02](issues/02-observation-validation-and-frames.md#answer) is resolved;
  tickets 03 through 06 remain outside implementation authorization.
- 2026-10-28: User authorized ticket 03 implementation. Configured typed creation
  is implemented and reviewed; tickets 04 through 06 remain outside authorization.
