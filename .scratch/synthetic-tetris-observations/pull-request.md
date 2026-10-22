# Add synthetic Tetris observations and an observation-only analysis engine

## Summary

Provide predictable CAS object hypotheses for later annotators without a camera
or inference model. The editable default sequence emits 301 observations over
thirty seconds, including motion, pauses, disappearance, and reappearance.

```text
scripted_tetris
  scheduled capture
    fresh CAS and optical camera metadata
    metric CAD hypotheses, offset boxes, and projected foreground masks
    native annotator image and geometry outputs
```

The headless command exits after playback. Optional native OpenCV and Open3D
windows retain the final scene for inspection. Existing runner and visualizer
lifecycle behavior provides both modes. Beliefs and world contents remain
unchanged. [Setup and run guide](../../robokudo/doc/scripted_tetris.md).

## Evidence

- **Before:** the workspace had no `scripted_tetris` analysis engine or finite
  synthetic Tetris observation source.
  **After:** 91 tests passed with the configured Python interpreter and Open3D
  0.20.0: 63 scripted feature cases and 28 existing pipeline, CAS, garden, GUI,
  and type-conversion checks. Tests used `--orm-build never`, serialized runs,
  a free-memory check, and an 8 GiB memory cap.
- Public-engine checks cover all 301 scheduled observations, early and late
  ticks, exact camera metadata, CAD geometry, silhouette gaps, motion, absence,
  reappearance, stable exhaustion, and unchanged world/belief state. Native
  visualization tests exercise the real lifecycle with desktop windows mocked.
- All 17 changed Python files pass `scripts/format_docstrings.py` unchanged;
  `git diff --check` passes. The separate standards and specification reviews
  are recorded in [review.md](review.md), including all eight standards fixes.
- The real documented headless command completed with the supplied PLY meshes:
  exit status 0 in 37.63 seconds including startup, with no Python exceptions.

## Merge Danger

**Door:** two-way

The engine and its observation support are additive; no schema migration or
existing runner change is required.

**Blast Radius:** RoboKudo

Changes add the synthetic descriptor, its support classes, tests, and guides.
