# 01: Define aircraft semantic types and model resources

Status: claimed
Type: task
Blocked by: None

Implementation permission: not granted. Do not claim or execute this ticket
without explicit user permission to start implementation.

## Outcome

`robokudo_acrambly` owns the 12 configured aircraft-part kinds under
`AircraftPart`, with labels and model resources ready for generic acceptance.
Follow the [specification](../spec.md) and [ADR 0003](../../../docs/adr/0003-keep-aircraft-semantics-in-acrambly.md).

## Acceptance criteria

- [ ] Add documented dataclass semantic types using existing body-root and
  perception interfaces, retaining the accepted classification label.
- [ ] Define each model's label, concrete semantic type, resource, and metric
  scale once, using the inventory in the connector brief.
- [ ] Accept a configured mesh directory and report unavailable resources with
  an actionable existing or custom dataclass exception.
- [ ] Load each configured model once and permit its resource to be replaced
  later without changing observation labels.
- [ ] Supply the same model geometry for both visual and collision collections,
  preserving mesh origin and local offsets with independent per-body state.
- [ ] Add test-first application coverage for all 12 mappings, semantic subtype
  queries, resource loading, and missing resources. Use checked-in assets where
  the behavior depends on those models.
- [ ] Keep application types out of generic RoboKudo and semantic digital twin
  imports. Grasp definitions and assembly relationships remain outside scope.

## Verification

Run the focused aircraft model/type tests in `test/robokudo_acrambly_test` with
the repository's interpreter, memory controls, and `--orm-build never`. Format
modified Python files and record results when implementation is authorized.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
- 2026-10-28: User explicitly requested implementation of ticket 01 and confirmed
  semantic subtype/label queries and model catalog/geometry as the test seams.
  This supersedes the earlier permission note for ticket 01 only.
