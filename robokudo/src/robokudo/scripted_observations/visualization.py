# %% Imports

from __future__ import annotations

from dataclasses import astuple, dataclass
from enum import Enum

import numpy
import open3d
from typing_extensions import TYPE_CHECKING

from robokudo.scripted_observations.projection import MaskPixel
from robokudo.types.annotation import (
    BoundingBox3DAnnotation,
    Classification,
    PoseAnnotation,
    Shape,
)
from robokudo.types.scene import ObjectHypothesis
from robokudo.utils.annotator_helper import draw_bounding_boxes_from_object_hypotheses
from robokudo.utils.o3d_helper import trimesh_to_o3d_mesh
from robokudo.utils.type_conversion import (
    get_o3d_obb_from_bounding_box_annotation,
    get_transform_matrix_from_pose_annotation,
)


if TYPE_CHECKING:
    import numpy.typing

    from robokudo.cas import CAS


# %% Synthetic image appearance


@dataclass(frozen=True)
class ImageChannels:
    """
    Unsigned image-channel intensities in OpenCV blue-green-red order.
    """

    blue: int
    """
    Blue channel intensity.
    """

    green: int
    """
    Green channel intensity.
    """

    red: int
    """
    Red channel intensity.
    """


class ImageColor(Enum):
    """
    Background and foreground colors in the illustrative camera image.
    """

    BACKGROUND = ImageChannels(blue=24, green=24, red=24)
    FOREGROUND = ImageChannels(blue=64, green=176, red=240)


# %% Observation output


@dataclass
class ObservationVisualization:
    """
    Native image and geometry evidence for one completed observation.
    """

    observation: CAS
    """
    Completed camera observation whose annotations are displayed.
    """

    coordinate_axis_length: float = 0.05
    """
    Length in metres of the displayed CAD-origin coordinate axes.
    """

    def create_image(self) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Draw foreground masks, image regions, and model labels.
        """
        camera = self.observation.camera_info
        image = numpy.full(
            (camera.height, camera.width, 3),
            astuple(ImageColor.BACKGROUND.value),
            numpy.uint8,
        )
        hypotheses = self.observation.filter_annotations_by_type(ObjectHypothesis)
        for hypothesis in hypotheses:
            left, top, right, bottom = hypothesis.roi.roi.get_corner_points()
            image[top:bottom, left:right][
                hypothesis.roi.mask == MaskPixel.FOREGROUND
            ] = astuple(ImageColor.FOREGROUND.value)
        draw_bounding_boxes_from_object_hypotheses(
            image, hypotheses, self.get_model_label
        )
        return image

    def get_model_label(self, hypothesis: ObjectHypothesis) -> str:
        """
        Read the emitted model classification of a displayed hypothesis.
        """
        classifications = self.observation.filter_by_type(
            Classification, hypothesis.annotations
        )
        return classifications[0].classname

    def create_geometries(self) -> list[open3d.geometry.Geometry3D]:
        """
        Create camera-relative CAD meshes, enclosing boxes, and origin frames.
        """
        geometries = []
        for hypothesis in self.observation.filter_annotations_by_type(ObjectHypothesis):
            pose = self.observation.filter_by_type(
                PoseAnnotation, hypothesis.annotations
            )[0]
            shape = self.observation.filter_by_type(Shape, hypothesis.annotations)[0]
            box = self.observation.filter_by_type(
                BoundingBox3DAnnotation, hypothesis.annotations
            )[0]
            camera_T_model = get_transform_matrix_from_pose_annotation(pose)
            visual_mesh = trimesh_to_o3d_mesh(shape.geometry.mesh)
            visual_mesh.transform(camera_T_model)
            visual_box = get_o3d_obb_from_bounding_box_annotation(box)
            origin_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(
                size=self.coordinate_axis_length
            )
            origin_frame.transform(camera_T_model)
            geometries.extend([visual_mesh, visual_box, origin_frame])
        return geometries
