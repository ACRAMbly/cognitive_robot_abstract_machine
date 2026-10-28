# 05: Document aircraft usage and verify retained behavior

Status: ready-for-agent
Type: task
Blocked by: 01, 02, 03, 04.

Implementation permission: not granted. Follow the approved
[specification](../spec.md); completion records must describe executed checks.

## Outcome

Developers can run the aircraft source headlessly, opt into mesh verification,
and understand the minimal CAS output and separate PR #13 consumer setup.

## Acceptance criteria

- [ ] Make `scripted_aircraft` the primary synthetic example in the package
  README while retaining the working Tetris command and guides. Document separate
  commands and configuration for the independent engines; no combined mode is
  introduced.
- [ ] Document exact configured-interpreter/ROS setup, engine selection, required
  frame configuration, capture cadence, tick rate, and finite completion.
- [ ] Document optional preview activation, explicit mesh-directory setup using
  `plane/` as a checkout example, and final-scene inspection/exit behavior.
- [ ] Describe labels, mesh-origin camera poses, detection IDs, scheduled capture
  timestamps, frames, and transform fields. Explain that image evidence and
  per-detection mesh/box annotations are absent.
- [ ] Distinguish source emission from PR #13 consumption: application-owned frame
  initialization and catalog semantic/geometry mappings are consumer prerequisites;
  the source-only engine creates no beliefs or world objects.
- [ ] Document editing poses, motion, holds, absence membership, and timing in
  structured configuration without copying the model inventory into a new source
  of truth.
- [ ] Run new aircraft source/compatibility/preview coverage and relevant existing
  Tetris, pipeline, CAS, and native lifecycle regressions in focused batches.
- [ ] Exercise finite headless completion with documented configuration; perform
  any native visual inspection only in an available display environment and
  report whether it was performed.
- [ ] Record exact commands, results, formatter status for modified Python files,
  and preexisting failures or unavailable dependencies. Report only observed
  success, and leave unresolved checks explicit.

## Verification

After permission, follow the specification's TDD, memory, interpreter, ORM, and
formatting requirements. Tests require no live services or credentials. Use the
existing package CI entry, retaining always-running local-fixture coverage.
Review documentation against actual implemented configuration and public output.

## Completion record

Pending explicit implementation permission. Record executed checks and delivered
documentation after the preceding tickets have been implemented and verified.
