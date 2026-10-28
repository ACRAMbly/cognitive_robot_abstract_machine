# 03: Verify aircraft observations with native 3D meshes

Status: ready-for-agent
Type: task
Blocked by: 01: Minimal aircraft CAS source; 02: Finite aircraft playback.

Implementation permission: not granted. Follow the approved
[specification](../spec.md), especially its optional 3D verification contract.

## Outcome

A developer can inspect catalog meshes moving at the poses emitted by the
aircraft source without adding mesh or image evidence to the CAS contract.

## Acceptance criteria

- [ ] Use existing native annotator geometry output and `AircraftModelCatalog`
  with an explicit mesh directory; reuse existing catalog resource errors.
- [ ] Load configured preview meshes once per preview configuration. Preserve
  original metric units, source origins, and component offsets.
- [ ] Place independent display mesh copies using the emitted camera-relative
  poses. Preserve shared catalog geometry, CAS poses, and world state.
- [ ] Render without requiring CAS shape/box annotations, masks, 2D regions,
  calibration, or a fabricated camera image. Leave Tetris visualization intact.
- [ ] Replace displayed part geometry on completed captures, clear absent parts,
  and retain prior output on waiting ticks.
- [ ] Retain the final scene and normal native exit controls after exhaustion;
  keep finite headless completion independent of display processing.
- [ ] Verify mesh bounds/offsets and pose placement using deterministic local
  resources, deriving expectations from the consumed catalog and scenario.
- [ ] Add display-independent pytest coverage for movement, empty output,
  geometry isolation, final-scene retention, and exit behavior before implementing
  the corresponding behavior. Mock desktop boundaries only as required.

## Verification

After permission, reuse the existing native-output/visualization fixtures. Core
checks require no display or live services. Actual aircraft-resource coverage
may supplement always-running local fixture checks. Run relevant existing Tetris
visualization and native lifecycle regressions as separate bounded batches.

## Completion record

Pending explicit implementation permission. Record resource configuration,
red/green evidence, lifecycle checks, and verification limitations afterward.
