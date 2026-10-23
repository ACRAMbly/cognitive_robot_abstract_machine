# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import cv2
import numpy
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo_acrambly.scripted_observations.exceptions import InvalidCameraProjection
from robokudo.types.cv import ImageROI, Point2D, Rect
from robokudo.utils.o3d_helper import project_points_to_image
from robokudo.utils.transform import get_transform_matrix


if TYPE_CHECKING:
    from semantic_digital_twin.world_description.geometry import Mesh

    from robokudo.cas import CAS
    from robokudo.types.tf import Pose


# %% Foreground rasterization


class MaskPixel(IntEnum):
    """
    Binary foreground evidence in RoboKudo image masks.
    """

    BACKGROUND = 0
    FOREGROUND = numpy.iinfo(numpy.uint8).max


@dataclass
class ForegroundProjection:
    """
    Image evidence for a CAD surface at one camera-relative pose.
    """

    mesh: Mesh
    """
    Metric surface geometry in the original CAD frame.
    """

    object_pose: Pose
    """
    Camera-relative translation and orientation of the CAD origin.
    """

    def create_region(self, observation: CAS) -> ImageROI:
        """
        Return a cropped silhouette of a fully visible mesh surface.

        :raises InvalidCameraProjection: The mesh extends outside the camera image.
        """
        mesh = self.mesh.mesh
        model_P_vertices = numpy.column_stack(
            (mesh.vertices, numpy.ones(len(mesh.vertices)))
        )
        camera_T_model = get_transform_matrix(
            Rotation.from_quat(self.object_pose.rotation).as_matrix(),
            self.object_pose.translation,
        )
        camera_P_vertices = (camera_T_model @ model_P_vertices.T).T
        pixels, valid = project_points_to_image(observation, camera_P_vertices[:, :3])
        camera = observation.camera_info
        if (
            not numpy.all(valid)
            or len(pixels) == 0
            or numpy.any(pixels < 0)
            or numpy.any(pixels[:, 0] >= camera.width)
            or numpy.any(pixels[:, 1] >= camera.height)
        ):
            raise InvalidCameraProjection(self.object_pose)
        left, top, width, height = cv2.boundingRect(pixels)
        mask = numpy.full((height, width), MaskPixel.BACKGROUND, dtype=numpy.uint8)
        local_pixels = pixels - [left, top]
        for triangle in mesh.faces:
            cv2.fillConvexPoly(mask, local_pixels[triangle], MaskPixel.FOREGROUND)
        return ImageROI(
            roi=Rect(pos=Point2D(x=left, y=top), width=width, height=height),
            mask=mask,
        )
