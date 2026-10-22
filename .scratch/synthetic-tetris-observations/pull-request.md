# Add ACRAMbly's RoboKudo extension package for synthetic Tetris observations

## Summary

Supply finite scripted observations through `robokudo_acrambly`, using the
repository's existing RoboKudo interfaces. The default sequence emits 301
observations over thirty seconds, including motion, pauses, disappearance,
reappearance, foreground masks, and optional native visualization.

```text
robokudo_acrambly/
  pyproject.toml                  workspace dependencies and wheel resources
  src/robokudo_acrambly/
    annotators/                  scripted observation source
    descriptors/analysis_engines/ scripted_tetris pipeline
    scripted_observations/       scenario, geometry, camera, and visualization
    data/                        both supplied PLY meshes
  doc/                           setup, masks, and native visualization
test/robokudo_acrambly_test/      feature and package-loading tests
```

Launch with `_ros_pkg robokudo_acrambly` and configure the mesh directory through
`TETRIS_MESH_DIRECTORY`. RoboKudo matches the repository's base branch unchanged.
[Setup and run guide](../../robokudo_acrambly/doc/scripted_tetris.md).

## Evidence

- **Before:** `test_launcher_loads_the_engine_from_the_extension_package` failed
  with `ModuleNotFoundError` for `robokudo_acrambly`.
  **After:** all 64 extension tests pass against an installed wheel, including
  the original 63 feature cases and the package-loading regression.
- Separate runs pass all 28 selected RoboKudo regression tests and all 26
  workspace dependency/version checks: **118 passed** in total. Runs used the
  configured Python interpreter, `--orm-build never`, memory checks, and an
  8 GiB virtual-memory cap.
- The wheel contains both PLY meshes. All 11 moved source modules retain their
  implementation apart from import namespaces and formatting.
- The documented headless command completes with exit status 0 using
  `_ros_pkg robokudo_acrambly` and the relocated mesh directory.
- All 26 package and test Python files pass `scripts/format_docstrings.py`
  unchanged. CI workflow YAML parses, its shell steps pass `bash -n`, and
  `git diff --check` passes.

## Merge Danger

**Door:** two-way

No schema migration is required. Imports of the custom components now use
`robokudo_acrambly`; launch commands must select that package explicitly.

**Blast Radius:** Perception

Adds a workspace member, dependency declarations, and a separate CI entry. The
new CI run executes serially with a memory cap and explicit ORM regeneration.
Existing RoboKudo runtime code, beliefs, and world contents remain unchanged.
