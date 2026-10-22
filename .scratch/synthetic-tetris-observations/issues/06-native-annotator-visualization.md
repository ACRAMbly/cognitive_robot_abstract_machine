# 06: Show scripted observations in RoboKudo's native annotator visualizers

**What to build:** A developer can opt into RoboKudo's existing 2D and 3D
annotator visualization windows while running the synthetic analysis engine.
The selected annotator's views update from completed observations. After finite
playback finishes, the native windows remain responsive for inspection of the
final scene until the user exits through the existing GUI controls.

**Blocked by:** 02: Add foreground masks from projected CAD outlines;
03: Run finite headless playback without losing observations.

**Status:** resolved

Implementation requires separate explicit user permission. The user explicitly
chose RoboKudo's native OpenCV and Open3D annotator visualizers. Follow the approved
synthetic Tetris observation specification, using the existing annotator output
and visualization lifecycle interfaces.

- [x] The observation annotator supplies its image and 3D geometry through the
  existing annotator visualization outputs. The native windows retain normal
  selection and navigation between annotators within the pipeline.
- [x] The 2D output shows an illustrative synthetic camera view of the projected
  blocks with foreground-mask, image-region, and model-label overlays, aligned
  with the corresponding completed CAS observation.
- [x] The 3D output shows the CAD meshes at their camera-relative poses together
  with bounding boxes and coordinate frames. Visualization transforms agree with
  the emitted annotations without altering or double-transforming CAS geometry.
- [x] New completed observations replace preceding visual outputs. Waiting ticks
  introduce no additional observation. Publishing empty block geometry clears
  prior block drawings, so single-block and empty scenes can be represented
  through the same output contract when those scenarios are available.
- [x] After the last scheduled observation, generation stops and the final CAS
  and visual outputs remain unchanged. Native GUI processing continues so the
  user can move the 3D camera and navigate annotator outputs. Exiting through
  existing GUI controls terminates the run; playback does not restart.
- [x] Headless execution stays independent of GUI event processing and exits on
  exhaustion. Documentation gives exact native-GUI and headless invocations and
  explains inspection after playback and the existing GUI exit controls.
- [x] Display-independent pytest coverage checks the produced images and
  geometries, empty-output clearing, and the lifecycle distinction between
  exhausted observation generation and ongoing GUI processing. Mock native
  visualizer backends where needed so CI opens no windows and needs no display.
- [x] This ticket is independently verifiable with a finite stationary sequence
  from its blockers. Its output contract also accepts the moving and changing
  visibility observations supplied by the other tickets.
- [x] Implement test-first with existing annotator-output, ROS, and blackboard
  fixtures and deterministic local meshes. Run focused checks with the configured
  interpreter and `--orm-build never`, checking and capping memory before each
  pytest run. Format modified Python docstrings using the repository formatter.

## Answer

The observation annotator now publishes illustrative 2D foreground, image-region,
and model-label evidence plus camera-relative CAD meshes, enclosing boxes, and
CAD-origin frames through its existing annotator output structure. Display meshes
are independent metric copies, so stored CAS geometry remains model-local. Each
completed capture replaces both outputs, including an empty geometry list for an
empty scene; waiting and exhausted ticks retain the preceding outputs.

The existing native visualization manager already continues processing OpenCV
and Open3D events after the observation source is exhausted. Display-independent
tests retain the real manager and native selection behavior while intercepting
desktop operations. They verify final-scene retention, annotator navigation,
native close termination, and headless independence without launcher or GUI
lifecycle changes. Image dimensions and missing geometry were demonstrated by
failing public-engine tests before output publication was implemented.

The [native visualization guide](../../../robokudo_acrambly/doc/scripted_tetris_visualization.md)
documents exact headless and native-GUI invocations, final-scene inspection, and
existing exit/navigation controls. It is linked from the main engine guide.

Validation with the configured interpreter and staged Open3D 0.20.0:
the static, masks, finite-playback, motion, and visualization batch passed
**50 tests** after merging ticket 04, including local supplied meshes. The
existing pipeline/garden/GUI batch passed
**14 tests**. Both runs used the serialized memory-bounded launcher and
`--orm-build never`. The formatter is stable on all three modified Python files.
