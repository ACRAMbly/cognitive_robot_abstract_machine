# 02: Validate configured observations and resolve world poses

Status: ready-for-agent
Type: task
Blocked by: None

Implementation permission: not granted. Do not claim or execute this ticket
without explicit user permission to start implementation.

## Outcome

The generic connector accepts application-supplied semantic configuration and
prepares valid observations before changing persistent beliefs. Generic tests
use local test semantic types rather than aircraft imports.

Follow the [specification](../spec.md), especially its observation and validation
contracts. Coordinate this extension point with ticket 01's application models.

## Acceptance criteria

- [ ] Add a configurable label-to-semantic-type acceptance interface without
  introducing application-package imports into generic RoboKudo.
- [ ] Validate every configured hypothesis's model label, supported pose, frame,
  capture metadata, and required transform before persistent belief changes.
- [ ] Reject missing, unknown, and conflicting model labels with meaningful
  exceptions; distinguish validation from association confidence.
- [ ] Interpret plain poses in the camera frame and stamped poses in the declared
  camera or world frame, placing the mesh origin correctly.
- [ ] Convert camera poses once; accept a world-frame stamped pose without an
  otherwise unnecessary camera transform. Preserve capture time and frame.
- [ ] Reprocessing annotations does not compound transforms or replace capture
  metadata with processing time.
- [ ] Test an invalid hypothesis after a valid one and compare persistent state
  exactly before and after, for both empty and populated belief worlds.
- [ ] Preserve supported generic behavior when semantic configuration is absent.

## Verification

Add meaningful failing tests before implementation. Run focused generic connector
tests with a nonidentity camera transform, existing fixtures, memory controls,
and `--orm-build never`. Format modified Python files and record results.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
