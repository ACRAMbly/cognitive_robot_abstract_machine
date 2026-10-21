# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo.scripted_observations.exceptions import InvalidCameraProjection
from robokudo.types.cv import ImageROI, Point2D, Rect
from robokudo.utils.o3d_helper import project_points_to_image


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
    FOREGROUND = np.iinfo(np.uint8).max


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
        vertices = Rotation.from_quat(self.object_pose.rotation).apply(mesh.vertices)
        vertices += self.object_pose.translation
        pixels, valid = project_points_to_image(observation, vertices)
        camera = observation.camera_info
        if (
            not np.all(valid)
            or len(pixels) == 0
            or np.any(pixels < 0)
            or np.any(pixels[:, 0] >= camera.width)
            or np.any(pixels[:, 1] >= camera.height)
        ):
            raise InvalidCameraProjection(self.object_pose)
        left, top, width, height = cv2.boundingRect(pixels)
        mask = np.full((height, width), MaskPixel.BACKGROUND, dtype=np.uint8)
        local_pixels = pixels - [left, top]
        for triangle in mesh.faces:
            cv2.fillConvexPoly(mask, local_pixels[triangle], MaskPixel.FOREGROUND)
        return ImageROI(
            roi=Rect(pos=Point2D(x=left, y=top), width=width, height=height),
            mask=mask,
        )
