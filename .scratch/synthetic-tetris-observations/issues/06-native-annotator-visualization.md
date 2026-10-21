# 06: Show scripted observations in RoboKudo's native annotator visualizers

**What to build:** A developer can opt into RoboKudo's existing 2D and 3D
annotator visualization windows while running the synthetic analysis engine.
The selected annotator's views update from completed observations. After finite
playback finishes, the native windows remain responsive for inspection of the
final scene until the user exits through the existing GUI controls.

**Blocked by:** 02: Add foreground masks from projected CAD outlines;
03: Run finite headless playback without losing observations.

**Status:** ready-for-agent

Implementation requires separate explicit user permission. The user explicitly
chose RoboKudo's native OpenCV and Open3D annotator visualizers. Follow the approved
synthetic Tetris observation specification, using the existing annotator output
and visualization lifecycle interfaces.

- [ ] The observation annotator supplies its image and 3D geometry through the
  existing annotator visualization outputs. The native windows retain normal
  selection and navigation between annotators within the pipeline.
- [ ] The 2D output shows an illustrative synthetic camera view of the projected
  blocks with foreground-mask, image-region, and model-label overlays, aligned
  with the corresponding completed CAS observation.
- [ ] The 3D output shows the CAD meshes at their camera-relative poses together
  with bounding boxes and coordinate frames. Visualization transforms agree with
  the emitted annotations without altering or double-transforming CAS geometry.
- [ ] New completed observations replace preceding visual outputs. Waiting ticks
  introduce no additional observation. Publishing empty block geometry clears
  prior block drawings, so single-block and empty scenes can be represented
  through the same output contract when those scenarios are available.
- [ ] After the last scheduled observation, generation stops and the final CAS
  and visual outputs remain unchanged. Native GUI processing continues so the
  user can move the 3D camera and navigate annotator outputs. Exiting through
  existing GUI controls terminates the run; playback does not restart.
- [ ] Headless execution stays independent of GUI event processing and exits on
  exhaustion. Documentation gives exact native-GUI and headless invocations and
  explains inspection after playback and the existing GUI exit controls.
- [ ] Display-independent pytest coverage checks the produced images and
  geometries, empty-output clearing, and the lifecycle distinction between
  exhausted observation generation and ongoing GUI processing. Mock native
  visualizer backends where needed so CI opens no windows and needs no display.
- [ ] This ticket is independently verifiable with a finite stationary sequence
  from its blockers. Its output contract also accepts the moving and changing
  visibility observations supplied by the other tickets.
- [ ] Implement test-first with existing annotator-output, ROS, and blackboard
  fixtures and deterministic local meshes. Run focused checks with the configured
  interpreter and `--orm-build never`, checking and capping memory before each
  pytest run. Format modified Python docstrings using the repository formatter.
