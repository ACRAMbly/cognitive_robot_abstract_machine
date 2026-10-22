# Native scripted Tetris visualization

Complete the [scripted Tetris setup](scripted_tetris.md#setup-and-headless-execution)
and configure `TETRIS_MESH_DIRECTORY` before running either command below.

The default documented run remains headless and exits after its final observation:

```bash
python -m robokudo.scripts.main _ae scripted_tetris _headless _tickrate 20
```

To opt into RoboKudo's native OpenCV image window and Open3D scene window, run:

```bash
python -m robokudo.scripts.main _ae scripted_tetris _tickrate 20
```

The existing launcher also enables its normal ROS visualization outputs. Keep
Open3D enabled for the native 3D scene; `_no3d` omits that window. Both commands
preserve all scheduled observations if delivery takes longer than capture time.

The selected observation annotator shows an illustrative camera image with
colored foreground masks, surrounding image regions, and model labels. Background
gaps inside the block outlines remain visible. These images are synthetic evidence
without sensor or lighting effects. The native 3D view shows each CAD mesh at its
camera-relative pose, its mesh-derived enclosing box, and a coordinate frame at
the original CAD origin. The CAD meshes stored in the CAS remain in their metric
model frames.

Each completed observation replaces the annotator's image and block geometries.
Empty scenes show only the image background and clear the previous block geometry.
Waiting ticks retain the preceding output. After finite playback finishes, no
new observations are generated; the final CAS, image, and scene remain available
while native GUI processing continues. You can move the Open3D camera and inspect
other annotator outputs until exiting.

Existing native controls remain available in the focused OpenCV window:

- Right arrow, `n`, or `k`: select the next annotator.
- Left arrow, `p`, or `j`: select the previous annotator.
- Escape: exit the run.

Both native views share the selected annotator. Closing the Open3D window also
exits the run. Space uses RoboKudo's existing pipeline-trigger control; it does not
pause or restart this finite observation sequence.

Display-independent tests run the public engine with the real visualization
manager and native backend selection. They intercept desktop window operations,
so CI verifies geometry, image evidence, final-scene interaction, and exit behavior
without opening windows or requiring a display.
