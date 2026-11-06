# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import TYPE_CHECKING

from robokudo.types.annotation import Classification, PoseAnnotation
from robokudo.types.scene import ObjectHypothesis
from robokudo.utils.o3d_helper import trimesh_to_o3d_mesh
from robokudo.utils.type_conversion import get_transform_matrix_from_pose_annotation

if TYPE_CHECKING:
    import open3d

    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog


# %% Catalog mesh display


@dataclass
class AircraftPreview:
    """
    Display independent catalog meshes at captured camera-relative origins.
    """

    catalog: AircraftModelCatalog
    """Explicitly loaded metric mesh templates for the configured aircraft kinds."""

    def create_geometries(self, observation: CAS) -> list[open3d.geometry.TriangleMesh]:
        """
        Create display copies without changing catalog meshes or captured poses.
        """
        geometries = []
        for hypothesis in observation.filter_annotations_by_type(ObjectHypothesis):
            (classification,) = observation.filter_by_type(
                Classification, hypothesis.annotations
            )
            (pose,) = observation.filter_by_type(PoseAnnotation, hypothesis.annotations)
            template = self.catalog.models[classification.classname].geometry.mesh
            geometry = trimesh_to_o3d_mesh(template)
            camera_T_part = get_transform_matrix_from_pose_annotation(pose)
            geometry.transform(camera_T_part)
            geometries.append(geometry)
        return geometries
