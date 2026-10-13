"""
Human detection with body keypoint estimation.

Detects humans with a YOLO pose model and annotates each one as a human hypothesis that
carries a person classification and its 2D body keypoints. With depth, each hypothesis
also gets the point cloud of its bounding box and, optionally, one point cloud per
clearly visible keypoint.

.. note::
   The model needs the packages of RoboKudo's optional ``dev`` extra.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from timeit import default_timer

import cv2
import numpy as np
import open3d as o3d
from py_trees.common import Status
from typing_extensions import TYPE_CHECKING, Any, List, Optional, Tuple
from ultralytics import YOLO

from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.yolo_annotator import PERSON_CLASS_NAME, YoloModelParameters
from robokudo.cas import CASViews
from robokudo.types.annotation import Classification, CloudAnnotation
from robokudo.types.human import KeypointAnnotation
from robokudo.types.scene import HumanHypothesis
from robokudo.utils.annotator_helper import scale_camera_intrinsics
from robokudo.utils.cv_helper import get_scaled_color_image_for_depth_image
from robokudo.utils.decorators import record_time
from robokudo.utils.o3d_helper import (
    concatenate_clouds,
    get_cloud_from_rgb_depth_and_mask,
)

if TYPE_CHECKING:
    import numpy.typing as npt
    from ultralytics.engine.results import Results

WARMUP_IMAGE_SHAPE = (512, 512, 3)
"""
Shape of the blank image the model is warmed up on.
"""

NON_MAXIMUM_SUPPRESSION_OVERLAP = 0.5
"""
Intersection over union above which the model merges overlapping detections.
"""

MASK_FOREGROUND = 255
"""
Value of the pixels a point cloud is built from in a depth mask.
"""


class PoseKeypoint(IntEnum):
    """
    Index of each body keypoint in the keypoints a YOLO pose model reports.
    """

    NOSE = 0
    LEFT_EYE = 1
    RIGHT_EYE = 2
    LEFT_EAR = 3
    RIGHT_EAR = 4
    LEFT_SHOULDER = 5
    RIGHT_SHOULDER = 6
    LEFT_ELBOW = 7
    RIGHT_ELBOW = 8
    LEFT_WRIST = 9
    RIGHT_WRIST = 10
    LEFT_HIP = 11
    RIGHT_HIP = 12
    LEFT_KNEE = 13
    RIGHT_KNEE = 14
    LEFT_ANKLE = 15
    RIGHT_ANKLE = 16


# %% depth


@dataclass
class DepthFrame:
    """
    The depth measurements point clouds are built from, with matching color.
    """

    depth_image: npt.NDArray[np.uint16]
    """
    The depth image.
    """

    rgb_image: npt.NDArray[np.uint8]
    """
    The color image in RGB order, scaled to the depth image resolution.
    """

    color_to_depth_ratio: Tuple[float, float]
    """
    Size of the depth image relative to the color image, along x and y.
    """

    camera_intrinsics: o3d.camera.PinholeCameraIntrinsic
    """
    Intrinsics of the camera at the depth image resolution.
    """

    def depth_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """
        :return: The depth image pixel at the given color image position.
        """
        ratio_x, ratio_y = self.color_to_depth_ratio
        return int(x * ratio_x), int(y * ratio_y)

    def box_cloud(
        self, corners: npt.NDArray[np.float32], depth_truncate: float
    ) -> o3d.geometry.PointCloud:
        """
        :param corners: Corners (x1, y1, x2, y2) of a box in color image pixels.
        :param depth_truncate: Distance in meters beyond which measurements are left out.
        :return: The point cloud of the depth measurements inside the box.
        """
        x1, y1 = self.depth_pixel(corners[0], corners[1])
        x2, y2 = self.depth_pixel(corners[2], corners[3])
        mask = np.zeros_like(self.depth_image, dtype=np.uint8)
        mask[y1:y2, x1:x2] = MASK_FOREGROUND
        return self._masked_cloud(mask, depth_truncate=depth_truncate)

    def disc_cloud(self, x: float, y: float, radius: int) -> o3d.geometry.PointCloud:
        """
        :param x: Horizontal disc center in color image pixels.
        :param y: Vertical disc center in color image pixels.
        :param radius: Disc radius in depth image pixels.
        :return: The point cloud of the depth measurements inside the disc.
        """
        mask = np.zeros_like(self.depth_image, dtype=np.uint8)
        cv2.circle(
            mask, self.depth_pixel(x, y), radius, MASK_FOREGROUND, thickness=cv2.FILLED
        )
        return self._masked_cloud(mask)

    def _masked_cloud(
        self, mask: npt.NDArray[np.uint8], **cloud_settings: float
    ) -> o3d.geometry.PointCloud:
        return get_cloud_from_rgb_depth_and_mask(
            rgb_image=self.rgb_image,
            depth_image=self.depth_image.copy(),
            mask=mask,
            camera_intrinsics=self.camera_intrinsics,
            **cloud_settings,
        )


# %% annotator


class HumanAndPoseAnnotator(BaseAnnotator):
    """
    Detects humans with a YOLO pose model and annotates them with their keypoints.
    """

    class Descriptor(BaseAnnotator.Descriptor):
        """
        Descriptor for the HumanAndPoseAnnotator.
        """

        class Parameters(YoloModelParameters):
            """
            Parameters for the HumanAndPoseAnnotator.
            """

            def __init__(self) -> None:
                super().__init__()
                self.pretrained_model = "yolov8n-pose.pt"
                self.confidence_threshold: float = 0.7
                """
                Minimum confidence of a human detection to be annotated.
                """
                self.global_with_depth: bool = False
                """
                Whether to build point clouds from the depth image.
                """
                self.shrink_bounding_box: Optional[float] = None
                """
                Fraction in [0, 1] of its width and height the bounding box of each
                human is shrunk by around its center, or None to keep it.

                Shrinking may interfere with activity recognition.
                """
                self.shrink_as_annotation: bool = False
                """
                Whether, with depth, the shrunk box only yields a cloud annotation
                instead of replacing the region of the human hypothesis.
                """
                self.generate_clouds_for_keypoints: bool = False
                """
                Whether, with depth, to build a point cloud around every keypoint.

                .. warning::
                   This is computationally expensive.
                """
                self.keypoint_confidence_threshold_for_clouds: float = 0.6
                """
                Minimum confidence of a keypoint to get a non-empty point cloud.
                """
                self.keypoint_masking_radius: int = 6
                """
                Radius in depth image pixels around each keypoint its cloud is built
                from.
                """
                self.warmup_annotator: bool = True
                """
                Whether setup runs the model once, so that the first real detection is
                not slowed down by its initialization.
                """
                self.depth_truncate: float = 3.5
                """
                Distance in meters beyond which depth measurements are left out of the
                box clouds, removing background from the human detection.
                """

    def __init__(
        self,
        name: str = "HumanAndPoseAnnotator",
        descriptor: Optional[HumanAndPoseAnnotator.Descriptor] = None,
    ) -> None:
        super().__init__(name, descriptor)
        self.model_location: str = self.descriptor.parameters.model_location(self.name)
        """
        Where the pose model is loaded from.
        """
        self.model: Optional[YOLO] = None
        """
        The YOLO pose model, loaded during setup.
        """
        self.camera_intrinsics: Optional[o3d.camera.PinholeCameraIntrinsic] = None
        """
        Intrinsics of the camera at the resolution of the depth image.
        """

    def setup(self, **kwargs: Any) -> bool:
        """
        Load the model and, if configured, warm it up.
        """
        self.model = YOLO(self.model_location)
        if self.descriptor.parameters.warmup_annotator:
            self._predict(np.zeros(WARMUP_IMAGE_SHAPE, dtype=np.uint8))
        return super().setup(**kwargs)

    def _predict(self, image: npt.NDArray[np.uint8]) -> Results:
        """
        :return: The humans and keypoints the model detects in the image.
        """
        return self.model.predict(
            image,
            conf=self.descriptor.parameters.confidence_threshold,
            iou=NON_MAXIMUM_SUPPRESSION_OVERLAP,
            save=False,
            verbose=False,
        )[0]

    @record_time
    def update(self) -> Status:
        """
        Detect humans and annotate them as human hypotheses with their keypoints.
        """
        start_timer = default_timer()
        parameters: HumanAndPoseAnnotator.Descriptor.Parameters = (
            self.descriptor.parameters
        )
        depth_frame = self._depth_frame() if parameters.global_with_depth else None
        result = self._predict(self.get_cas().get(CASViews.COLOR_IMAGE))

        hypotheses = [
            self._hypothesis(result, index, depth_frame)
            for index in range(len(result.boxes))
        ]
        self.get_cas().annotations.extend(hypotheses)

        if parameters.global_with_visualization:
            self._visualize(result, hypotheses)
        self.feedback_message = (
            f"Processing took {(default_timer() - start_timer):.4f}s"
        )
        return Status.SUCCESS

    def _depth_frame(self) -> DepthFrame:
        """
        :return: The depth measurements of the CAS with the color image scaled to them.
        """
        cas = self.get_cas()
        self.camera_intrinsics = cas.get_copy(CASViews.CAMERA_INTRINSIC)
        scale_camera_intrinsics(self)
        scaled_color = get_scaled_color_image_for_depth_image(
            cas, cas.get(CASViews.COLOR_IMAGE)
        )
        return DepthFrame(
            depth_image=cas.get(CASViews.DEPTH_IMAGE),
            rgb_image=cv2.cvtColor(scaled_color, cv2.COLOR_BGR2RGB),
            color_to_depth_ratio=cas.get(CASViews.COLOR2DEPTH_RATIO),
            camera_intrinsics=self.camera_intrinsics,
        )

    def _hypothesis(
        self, result: Results, index: int, depth_frame: Optional[DepthFrame]
    ) -> HumanHypothesis:
        """
        :return: The human hypothesis of the detection at the index of the result.
        """
        parameters: HumanAndPoseAnnotator.Descriptor.Parameters = (
            self.descriptor.parameters
        )
        corners = result.boxes.xyxy[index].cpu().numpy()
        shrunk_corners = (
            corners if parameters.shrink_bounding_box is None else self._shrunk(corners)
        )
        keeps_full_box = depth_frame is not None and parameters.shrink_as_annotation
        region_corners = corners if keeps_full_box else shrunk_corners

        hypothesis = HumanHypothesis()
        hypothesis.id = index
        hypothesis.source = self.name
        x1, y1, x2, y2 = region_corners
        hypothesis.roi.roi.pos.x = int(x1)
        hypothesis.roi.roi.pos.y = int(y1)
        hypothesis.roi.roi.width = int(x2 - x1)
        hypothesis.roi.roi.height = int(y2 - y1)
        hypothesis.annotations.append(
            Classification(
                source=self.name,
                classname=PERSON_CLASS_NAME,
                confidence=float(result.boxes.conf[index]),
            )
        )

        keypoints = result.keypoints.data[index].cpu().numpy().tolist()
        hypothesis.annotations.append(
            KeypointAnnotation(
                source=self.name,
                keypoints=keypoints,
                type=KeypointAnnotation.KP_TYPE_2D,
            )
        )
        if depth_frame is None:
            return hypothesis

        hypothesis.points = depth_frame.box_cloud(
            region_corners, parameters.depth_truncate
        )
        if parameters.shrink_bounding_box is not None and keeps_full_box:
            hypothesis.annotations.append(
                CloudAnnotation(
                    source=self.name,
                    points=depth_frame.box_cloud(
                        shrunk_corners, parameters.depth_truncate
                    ),
                )
            )
        if parameters.generate_clouds_for_keypoints:
            hypothesis.annotations.append(
                KeypointAnnotation(
                    source=self.name,
                    keypoints=self._keypoint_clouds(keypoints, depth_frame),
                    type=KeypointAnnotation.KP_TYPE_3D,
                )
            )
        return hypothesis

    def _shrunk(self, corners: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
        """
        :return: The box shrunk by the configured fraction around its center.
        """
        x1, y1, x2, y2 = corners
        shrink_fraction = self.descriptor.parameters.shrink_bounding_box
        margin_x = (x2 - x1) * shrink_fraction / 2
        margin_y = (y2 - y1) * shrink_fraction / 2
        return np.array([x1 + margin_x, y1 + margin_y, x2 - margin_x, y2 - margin_y])

    def _keypoint_clouds(
        self, keypoints: List[List[float]], depth_frame: DepthFrame
    ) -> List[List[Any]]:
        """
        :param keypoints: The (x, y, confidence) of each keypoint in color image pixels.
        :return: The (point cloud, confidence) of each keypoint, the cloud empty for
            keypoints below the confidence threshold.
        """
        parameters: HumanAndPoseAnnotator.Descriptor.Parameters = (
            self.descriptor.parameters
        )
        keypoint_clouds = []
        for x, y, confidence in keypoints:
            if confidence < parameters.keypoint_confidence_threshold_for_clouds:
                keypoint_clouds.append([o3d.geometry.PointCloud(), confidence])
                continue
            cloud = depth_frame.disc_cloud(x, y, parameters.keypoint_masking_radius)
            keypoint_clouds.append([cloud, confidence])
        return keypoint_clouds

    def _visualize(self, result: Results, hypotheses: List[HumanHypothesis]) -> None:
        """
        Output the detections drawn by the model and the clouds of the hypotheses.
        """
        self.get_annotator_output_struct().set_image(result.plot())
        clouds = [
            hypothesis.points
            for hypothesis in hypotheses
            if hypothesis.points is not None
        ]
        if len(clouds) > 0:
            self.get_annotator_output_struct().set_geometries(
                concatenate_clouds(clouds)
            )
