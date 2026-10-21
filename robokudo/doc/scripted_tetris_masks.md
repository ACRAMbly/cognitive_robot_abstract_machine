# Synthetic image regions and foreground masks

Each visible scripted Tetris block has the normal RoboKudo
`ObjectHypothesis.roi` image evidence:

- `roi.roi` gives the top-left image pixel, width, and height.
- `roi.mask` is a cropped `numpy.uint8` array whose shape is
  `(roi.roi.height, roi.roi.width)`. Foreground pixels are 255; background pixels
  are 0. The pixel at `mask[row, column]` belongs to image position
  `(roi.roi.pos.x + column, roi.roi.pos.y + row)`.

The mask follows the CAD surface projected through the observation's camera
intrinsics at its camera-relative CAD-origin pose. Concave outlines and holes
retain their background pixels inside the surrounding image rectangle. These
are illustrative synthetic observations, without lighting or sensor effects.

The model-local CAD mesh in `Shape`, the plain CAD-origin `PoseAnnotation`, and
the offset `BoundingBox3DAnnotation` remain separate from this image evidence.
Later processing can use the CAD mesh for visual geometry and the enclosing box
for collision geometry. A foreground mask supplies image evidence and cannot
replace either 3D shape. Projection does not transform or rescale the stored CAD
mesh.

The example poses keep both blocks fully within the image and separated.
Partially visible meshes and inter-object occlusion are outside this source's
contract. An unsupported mesh projection raises `InvalidCameraProjection` with
its configured camera-relative pose.

Core mask checks use deterministic local CAD fixtures in the RoboKudo test suite.
Additional checks use supplied PLY files when available in
`TETRIS_MESH_DIRECTORY`, falling back to the checkout root, and skip when those
resources are absent. The fixture checks always run without external assets or
services.
