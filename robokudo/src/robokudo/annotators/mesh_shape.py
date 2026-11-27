"""
Shapes of classified object hypotheses with a 6D pose, from the meshes of their classes.
"""

from __future__ import annotations

from dataclasses import InitVar, dataclass, field, replace

from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from py_trees.common import Status

from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.foundation_pose import ObjectClassMesh, ObjectClassMeshes
from robokudo.types.annotation import PoseAnnotation, Shape
from robokudo.types.scene import ObjectHypothesis

# %% mesh shapes


@dataclass(kw_only=True, eq=False)
class MeshShapeAnnotator(BaseAnnotator):
    """
    Gives each object hypothesis whose latest classification names a class with a mesh,
    and that has a :class:`PoseAnnotation`, a :class:`Shape` of that mesh whose origin is
    its latest pose annotation: the pose of the mesh's own frame in the camera optical
    frame. Every other object hypothesis is left unchanged.
    """

    class_meshes: InitVar[list[ObjectClassMesh]]
    """
    The meshes of the object classes, at most one per class.
    """

    object_class_meshes: ObjectClassMeshes = field(init=False)
    """
    The meshes of the object classes by the name of their class.
    """

    def __post_init__(self, class_meshes: list[ObjectClassMesh]) -> None:
        """
        :raises DuplicateObjectClassMesh: If a class has more than one mesh.
        """
        super().__init__(name=type(self).__name__)
        self.object_class_meshes = ObjectClassMeshes.from_class_meshes(class_meshes)

    def update(self) -> Status:
        """
        Give the object hypotheses of the CAS the mesh shapes of their classes, and
        succeed.
        """
        for hypothesis in self.get_cas().filter_annotations_by_type(ObjectHypothesis):
            class_mesh = self.object_class_meshes.meshes_by_class_name.get(
                hypothesis.class_label
            )
            pose_annotation = hypothesis.latest_annotation_of_type(
                annotation_type=PoseAnnotation
            )
            if class_mesh is None or pose_annotation is None:
                continue
            hypothesis.annotations.append(
                Shape(
                    source=self.get_class_name(),
                    geometry=replace(
                        class_mesh.shape,
                        origin=HomogeneousTransformationMatrix.from_xyz_quaternion(
                            *pose_annotation.translation, *pose_annotation.rotation
                        ),
                    ),
                )
            )
        return Status.SUCCESS
