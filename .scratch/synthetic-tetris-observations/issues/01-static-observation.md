# 01: Emit a static observation with metric geometry

**What to build:** A developer can configure the two supplied Tetris block models
and inspect a static observation through the public `scripted_tetris` analysis
engine. The completed CAS contains camera metadata and block hypotheses with
consistent metric poses, CAD meshes, and enclosing boxes, ready for later
annotators to consume.

**Blocked by:** None (can start immediately).

**Status:** resolved

Implementation requires separate explicit user permission. Follow the approved
synthetic Tetris observation specification, domain glossary, and recorded
architecture decisions. Preserve unrelated local work and use existing RoboKudo
interfaces and dependencies.

- [x] An editable, structured descriptor supplies the scene and camera settings;
  structured model definitions supply model identifiers, mesh resources, and
  scale. A developer can configure the mesh directory, and missing resources
  raise an actionable custom exception.
- [x] Both supplied PLY meshes declare `0.001` metres per mesh unit. Conversion
  happens once, preserves the original CAD origin, and derives box dimensions
  and model-local center offsets from mesh geometry.
- [x] Each visible block produces one `ObjectHypothesis` with the appropriate
  `child_cube_0` or `child_cube_2` classification, a deterministic detection
  identifier, a plain camera-relative `PoseAnnotation`, a
  `BoundingBox3DAnnotation`, and an existing `Shape` annotation carrying its CAD
  mesh in the model frame. Identifiers describe detections rather than persistent
  physical objects.
- [x] Translations use metres and quaternions use xyzw order. The hypothesis pose
  locates the original CAD origin; the box pose transforms its mesh-derived
  center offset correctly, including for a configured rotated static pose.
  Geometry is neither transformed nor scaled twice.
- [x] The completed observation is accessible through normal pipeline CAS access
  and contains the configured camera frame, image dimensions, intrinsics, and
  capture time. Defaults use the agreed optical frame, 640-by-480 images, and a
  sixty-degree horizontal field of view, calculated with existing camera support.
- [x] Tests run the public engine with deterministic local mesh fixtures and
  assert exact classifications, configured poses, metadata, metric bounds, and
  nonzero box-center offsets. Expected values come from the consumed definitions
  and geometry. Reuse existing ROS, blackboard, and annotator-output fixtures.
- [x] The engine creates no beliefs and leaves world contents unchanged from an
  isolated baseline. Existing launcher allocation of an empty shared world stays
  outside scope. Execution requires no physical camera, inference model, worker
  process, credentials, live API, or connector feature branch.
- [x] Add meaningful failing pytest coverage before implementation, retain all
  existing tests, and include fixture-based coverage in RoboKudo CI. Run focused
  checks with the configured interpreter and `--orm-build never`, checking and
  capping memory before each pytest run. Format modified Python docstrings using
  the repository formatter.


## Answer

Implemented the public `scripted_tetris` analysis engine with editable dataclass
scene and camera descriptors, structured model definitions, explicit millimetre
scale, and mesh-directory configuration through an argument or
`TETRIS_MESH_DIRECTORY`. Missing resources raise `MissingMeshResource` with the
expected resource path and a correction.

Observations carry both model classifications, deterministic per-detection IDs,
plain CAD-origin poses, existing mesh shape annotations, and mesh-derived oriented
boxes. Mesh geometry stays in the model frame; the object's orientation transforms
the nonzero box-center offset. CAS and camera headers retain the configured
optical frame, matching intrinsics, image dimensions, and exact capture nanoseconds.
The injectable `Clock` separates capture time from CAS construction time. No
beliefs or world contents are changed.

Validation used the existing ROS, blackboard, and annotator-output fixtures and
locally generated deterministic CAD meshes. Test-first slices established missing
engine, geometry, resource error, and metadata behavior before implementation.
Focused validation with `--orm-build never` and an 8 GiB virtual-memory cap passed:
`test_scripted_tetris.py` plus `test_pipeline.py`: **15 passed** (19 existing
dependency warnings). Modified Python files were formatted with
`scripts/format_docstrings.py`. Masks and finite playback remain in their
separate dependent tickets.
