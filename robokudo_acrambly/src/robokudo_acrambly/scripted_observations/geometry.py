# %% Imports

from __future__ import annotations

from dataclasses import dataclass

import numpy
from scipy.spatial.transform import Rotation
from semantic_digital_twin.world_description.geometry import Mesh, Scale
from typing_extensions import TYPE_CHECKING

from robokudo_acrambly.scripted_observations.exceptions import MissingMeshResource
from robokudo.types.annotation import BoundingBox3DAnnotation, PoseAnnotation, Shape
from robokudo.types.tf import Pose
from robokudo.utils.transform import get_transform_matrix


if TYPE_CHECKING:
    from pathlib import Path

    from robokudo_acrambly.scripted_observations.models import TetrisModel


# %% Metric CAD geometry


@dataclass
class ModelGeometry:
    """
    A block mesh expressed in metres in its original CAD frame.
    """

    mesh: Mesh
    """Metric CAD geometry with an identity local origin."""

    @classmethod
    def from_directory(cls, model: TetrisModel, mesh_directory: Path) -> ModelGeometry:
        """
        Load a model resource using its declared coordinate scale.
        """
        definition = model.value
        mesh_path = mesh_directory / definition.mesh_filename
        if not mesh_path.is_file():
            raise MissingMeshResource(mesh_path)
        scale = definition.metres_per_mesh_unit
        return cls(
            mesh=Mesh.from_file(
                str(mesh_path),
                scale=Scale(scale, scale, scale),
            )
        )

    def create_annotations(
        self, pose: Pose, source: str
    ) -> list[PoseAnnotation | Shape | BoundingBox3DAnnotation]:
        """
        Describe the CAD origin, model mesh, and camera-relative enclosing box.
        """
        mesh = self.mesh.mesh
        model_P_box = numpy.append(mesh.bounds.mean(axis=0), 1.0)
        camera_T_model = get_transform_matrix(
            Rotation.from_quat(pose.rotation).as_matrix(), pose.translation
        )
        camera_P_box = camera_T_model @ model_P_box
        return [
            PoseAnnotation(
                source=source,
                translation=list(pose.translation),
                rotation=list(pose.rotation),
            ),
            Shape(source=source, geometry=self.mesh),
            BoundingBox3DAnnotation(
                source=source,
                x_length=float(mesh.extents[0]),
                y_length=float(mesh.extents[1]),
                z_length=float(mesh.extents[2]),
                pose=Pose(
                    translation=camera_P_box[:3].tolist(), rotation=list(pose.rotation)
                ),
            ),
        ]
