# 01: Mesh shapes for classified pose estimates

Status: resolved
Type: task
Blocked by: None (can start immediately)

**What to build:** a generic RoboKudo annotator that lets a mesh-based 6D pose
estimate reach the world. It is configured with object class meshes, the same type
the FoundationPose annotator takes. It gives each classified object hypothesis that
carries a pose annotation the mesh of its class as a shape, placed at that pose, so
that the world update can turn it into a perception belief with that mesh. It knows
nothing about ACRAMbly. See the spec and ADR 0003.

- [ ] A classified object hypothesis with a pose annotation, whose class has a mesh, gets a shape annotation whose geometry is that mesh, with its origin at the pose (the mesh's own frame in the camera optical frame).
- [ ] An object hypothesis whose class has no mesh is left unchanged.
- [ ] An object hypothesis without a pose annotation is left unchanged.
- [ ] Object class meshes are reused from where they live today; nothing is moved or reimplemented.
- [ ] Tested in the RoboKudo tests with a small test mesh and no ACRAMbly code, on a CAS holding hypotheses built with the existing hypothesis helper.
- [ ] No change to the FoundationPose annotator, the perception belief, the object matcher or the world-updating annotator.

## Comments

## Answer

Built in `6fcb6923f0`. `MeshShapeAnnotator` in
`robokudo/src/robokudo/annotators/mesh_shape.py` is built as
`MeshShapeAnnotator(class_meshes=[ObjectClassMesh(...)])`. It collects the meshes into
`ObjectClassMeshes`, so two meshes for one class raise `DuplicateObjectClassMesh`.
It uses a hypothesis's latest classification and latest pose annotation, and gives it
a `Shape` with that mesh as geometry, with its origin at the pose. Hypotheses without
a mesh or a pose are left unchanged. Nothing else in `robokudo` changed.
`test/robokudo_test/test_mesh_shape.py` covers this with a small box mesh: 4 tests.
Together with `test_world_updating.py` and `test_foundation_pose.py`, 44 tests pass.
