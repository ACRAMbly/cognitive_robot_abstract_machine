# 02: Add foreground masks from projected CAD outlines

**What to build:** A later annotator can inspect each block's image region and
cropped binary foreground mask in completed engine observations. The mask follows
the projected CAD outline, including background gaps within the surrounding image
rectangle, while the CAD mesh and enclosing box remain separate geometry inputs.

**Blocked by:** 01: Emit a static observation with metric geometry.

**Status:** resolved

Implementation requires separate explicit user permission. Follow the approved
synthetic Tetris observation specification, domain glossary, and recorded decision
on foreground masks. Use existing dependencies; ask before adding one.

- [x] Every visible block's emitted hypothesis carries an image region and a
  binary mask cropped to that region, using the existing RoboKudo representation.
  Pixel positions agree with the configured camera intrinsics and CAD-origin pose.
- [x] Masks follow the projected CAD surface outline and exclude background gaps
  inside the enclosing image rectangle. A filled rectangle or enclosing convex
  outline cannot satisfy the fixture-based foreground and background assertions.
- [x] Public-engine tests use a deterministic local mesh with a known projected
  outline. They verify exact foreground and background pixels, mask dimensions,
  and image-region alignment at configured static translations and rotations.
  These checks can run independently of timed motion playback.
- [x] Mask generation preserves the existing model-frame CAD mesh, metric object
  pose, and offset bounding box. Tests verify those annotations remain consistent
  with the image evidence.
- [x] The supplied example poses keep both blocks fully in frame and
  non-overlapping. Real-model checks may depend on local asset availability;
  equivalent deterministic fixture checks always run in CI. Inter-object
  occlusion and partial visibility remain outside scope.
- [x] Document the image-region and foreground-mask output as illustrative
  synthetic observations and explain the distinction from visual and collision
  geometry supplied to later processing.
- [x] Implement test-first with pytest through public engine output, reuse
  existing fixtures, and compare against consumed configuration and mesh data.
  Run focused CI-compatible checks with the configured interpreter and
  `--orm-build never`, checking and capping memory before each pytest run. Format
  modified Python docstrings using the repository formatter.

## Answer

Completed hypotheses now carry cropped `ImageROI` evidence from projected CAD
triangles, with binary uint8 foreground masks preserving concave gaps and holes.
Projection reuses the existing camera helper and leaves metric model-frame meshes,
CAD-origin poses, and offset bounding boxes unchanged. Camera views provide
matching point-cloud intrinsics and same-resolution image scaling. Unsupported
projections identify the configured object pose in a dataclass exception.

Public-engine tests first failed for the missing image region at three static
translated/rotated poses, then passed with exact region dimensions and independent
foreground/background pixel expectations. Deterministic and supplied-mesh checks
confirm the example poses remain in frame and disjoint. Supplied checks skip when
the configured assets are absent; deterministic coverage always runs in CI.
Output contracts are documented in
[scripted_tetris_masks.md](../../../robokudo_acrambly/doc/scripted_tetris_masks.md).

Validation: the memory-bounded, serialized configured-interpreter run of
`test_scripted_tetris.py`, `test_scripted_tetris_masks.py`, and `test_pipeline.py`
passed **20 tests** with supplied meshes enabled and `--orm-build never`.
`scripts/format_docstrings.py` is stable on all five modified Python files.
