# Pose estimates reach the world as mesh shapes added by their own annotator

An object hypothesis with only a classification and a 6D pose, as a mesh-based pose
estimator leaves it, is dropped by the world update, because a perception belief
needs a shape. A separate, generic annotator in `robokudo`, configured with the
meshes of the object classes (`ObjectClassMesh`), adds the mesh of each classified
hypothesis as a `Shape` at its pose, right before `WorldUpdatingAnnotator`. The
application, such as `robokudo_acrambly`, only supplies the meshes.

## Considered Options

- Let `FoundationPoseAnnotator` add the shape itself: scripted observations and any
  other pose source would not get one.
- Let `PerceptionBelief` look up a mesh from the semantic class: needs a class-to-mesh
  contract on every perceivable class in `semantic_digital_twin`.
- Add the shape in `robokudo_acrambly`: every other user of a mesh-based pose
  estimator would hit the same gap.
