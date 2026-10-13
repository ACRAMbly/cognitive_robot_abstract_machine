"""
Tests for detecting humans and their body keypoints with a YOLO pose model.

The pose model is replaced by a stand-in reporting scripted detections, so no weights
are downloaded.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d
import pytest
from py_trees.common import Status
from typing_extensions import List

import robokudo.annotators.human_and_pose
from robokudo.annotators.human_and_pose import (
    WARMUP_IMAGE_SHAPE,
    HumanAndPoseAnnotator,
    PoseKeypoint,
)
from robokudo.annotators.yolo_annotator import PERSON_CLASS_NAME
from robokudo.cas import CASViews
from robokudo.exceptions import DetectionModelNotConfigured
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import Classification, CloudAnnotation
from robokudo.types.human import KeypointAnnotation
from robokudo.types.scene import HumanHypothesis
from .yolo_stand_in import (
    NormalizedBox,
    NormalizedKeypoint,
    ScriptedDetection,
    ScriptedUltralytics,
    ScriptedYoloModel,
)

pytestmark = pytest.mark.usefixtures("annotator_output_buffer")

# %% scripted pose model

PERSON_CLASS_ID = 0
"""
Class id of persons in the pose model.
"""

PERSON_BOX = NormalizedBox(left=0.25, top=0.25, right=0.75, bottom=0.75)
"""
Where the person is detected in every image.
"""

PERSON_CONFIDENCE = 0.9
"""
Confidence of the person detection.
"""

VISIBLE_KEYPOINT_CONFIDENCE = 0.95
"""
Confidence of the keypoints the pose model sees clearly.
"""

HIDDEN_KEYPOINT_CONFIDENCE = 0.1
"""
Confidence of the keypoints the pose model barely sees.
"""


def _person_keypoints() -> List[NormalizedKeypoint]:
    """
    :return: One keypoint per body part inside the person's box, with only the nose
        clearly visible.
    """
    return [
        NormalizedKeypoint(
            x=0.3 + 0.02 * keypoint,
            y=0.5,
            confidence=(
                VISIBLE_KEYPOINT_CONFIDENCE
                if keypoint == PoseKeypoint.NOSE
                else HIDDEN_KEYPOINT_CONFIDENCE
            ),
        )
        for keypoint in PoseKeypoint
    ]


@pytest.fixture
def ultralytics(monkeypatch) -> ScriptedUltralytics:
    """
    Make the annotator load a scripted pose model detecting one person.
    """
    scripted = ScriptedUltralytics(
        model=ScriptedYoloModel(
            names={PERSON_CLASS_ID: PERSON_CLASS_NAME},
            detections=[
                ScriptedDetection(
                    class_id=PERSON_CLASS_ID,
                    confidence=PERSON_CONFIDENCE,
                    box=PERSON_BOX,
                    keypoints=_person_keypoints(),
                )
            ],
        )
    )
    monkeypatch.setattr(robokudo.annotators.human_and_pose, "YOLO", scripted.load_model)
    return scripted


# %% scene setup

COLOR_IMAGE_SHAPE = (40, 80, 3)
"""
Shape of the color images in the CAS.
"""

DEPTH_IN_MILLIMETERS = 1000
"""
Distance every depth image pixel measures.
"""

MILLIMETERS_PER_METER = 1000.0
"""
Scale between the depth image values and point coordinates.
"""


def _color_image() -> np.ndarray:
    return np.zeros(COLOR_IMAGE_SHAPE, dtype=np.uint8)


def _descriptor() -> HumanAndPoseAnnotator.Descriptor:
    """
    :return: A descriptor ignoring depth and skipping the warmup.
    """
    descriptor = HumanAndPoseAnnotator.Descriptor()
    descriptor.parameters.warmup_annotator = False
    return descriptor


def _depth_descriptor() -> HumanAndPoseAnnotator.Descriptor:
    descriptor = _descriptor()
    descriptor.parameters.global_with_depth = True
    return descriptor


def _annotator_in_pipeline(
    descriptor: HumanAndPoseAnnotator.Descriptor, color_to_depth_ratio: float = 1.0
) -> HumanAndPoseAnnotator:
    """
    :param color_to_depth_ratio: Size of the depth image relative to the color image.
    :return: A set up annotator whose CAS holds a color and a matching depth image.
    """
    annotator = HumanAndPoseAnnotator(descriptor=descriptor)
    annotator.setup()
    pipeline = Pipeline("Sequence")
    pipeline.add_child(annotator)
    pipeline.setup()

    color_height, color_width = COLOR_IMAGE_SHAPE[:2]
    depth_height = int(color_height * color_to_depth_ratio)
    depth_width = int(color_width * color_to_depth_ratio)
    cas = annotator.get_cas()
    cas.set(CASViews.COLOR_IMAGE, _color_image())
    cas.set(
        CASViews.DEPTH_IMAGE,
        np.full((depth_height, depth_width), DEPTH_IN_MILLIMETERS, dtype=np.uint16),
    )
    cas.set(CASViews.COLOR2DEPTH_RATIO, (color_to_depth_ratio, color_to_depth_ratio))
    cas.set(
        CASViews.CAMERA_INTRINSIC,
        o3d.camera.PinholeCameraIntrinsic(
            color_width,
            color_height,
            100.0,
            100.0,
            color_width / 2.0,
            color_height / 2.0,
        ),
    )
    return annotator


def _only_hypothesis(annotator: HumanAndPoseAnnotator) -> HumanHypothesis:
    (hypothesis,) = annotator.get_cas().filter_annotations_by_type(HumanHypothesis)
    return hypothesis


def _annotations_of_type(hypothesis: HumanHypothesis, annotation_type: type) -> list:
    return [
        annotation
        for annotation in hypothesis.annotations
        if isinstance(annotation, annotation_type)
    ]


def _keypoints_of_type(
    hypothesis: HumanHypothesis, keypoint_type: str
) -> KeypointAnnotation:
    (keypoints,) = [
        annotation
        for annotation in _annotations_of_type(hypothesis, KeypointAnnotation)
        if annotation.type == keypoint_type
    ]
    return keypoints


def _person_corners(image: np.ndarray) -> List[int]:
    """
    :return: The corners (x1, y1, x2, y2) of the person in pixels of the given image.
    """
    return [int(corner) for corner in PERSON_BOX.corners_in(image)]


def _area(corners: List[int]) -> int:
    x1, y1, x2, y2 = corners
    return (x2 - x1) * (y2 - y1)


# %% model loading


def test_setup_loads_configured_pose_model_once(ultralytics):
    descriptor = _descriptor()

    HumanAndPoseAnnotator(descriptor=descriptor).setup()

    assert ultralytics.loaded_model_locations == [
        descriptor.parameters.pretrained_model
    ]


def test_annotator_without_model_raises(ultralytics):
    descriptor = _descriptor()
    descriptor.parameters.pretrained_model = None

    with pytest.raises(DetectionModelNotConfigured):
        HumanAndPoseAnnotator(descriptor=descriptor)


def test_setup_warms_model_up_on_blank_image(ultralytics):
    descriptor = _descriptor()
    descriptor.parameters.warmup_annotator = True

    HumanAndPoseAnnotator(descriptor=descriptor).setup()

    assert ultralytics.model.received_image_shape == WARMUP_IMAGE_SHAPE


def test_construction_loads_no_model(ultralytics):
    HumanAndPoseAnnotator(descriptor=_descriptor())

    assert ultralytics.loaded_model_locations == []


# %% detection


def test_person_becomes_human_hypothesis_classified_as_person(ultralytics):
    annotator = _annotator_in_pipeline(_descriptor())

    status = annotator.update()

    assert status == Status.SUCCESS
    hypothesis = _only_hypothesis(annotator)
    (classification,) = _annotations_of_type(hypothesis, Classification)
    assert classification.classname == PERSON_CLASS_NAME
    assert classification.confidence == pytest.approx(PERSON_CONFIDENCE)
    assert classification.source == annotator.name


def test_hypothesis_region_is_person_bounding_box(ultralytics):
    annotator = _annotator_in_pipeline(_descriptor())

    annotator.update()

    assert _only_hypothesis(annotator).roi.roi.get_corner_points() == tuple(
        _person_corners(_color_image())
    )


def test_hypothesis_carries_two_dimensional_keypoints_in_pixels(ultralytics):
    annotator = _annotator_in_pipeline(_descriptor())

    annotator.update()

    keypoints = _keypoints_of_type(
        _only_hypothesis(annotator), KeypointAnnotation.KP_TYPE_2D
    )
    np.testing.assert_allclose(
        keypoints.keypoints,
        [keypoint.in_pixels_of(_color_image()) for keypoint in _person_keypoints()],
        rtol=1e-5,
    )


def test_model_predicts_with_configured_threshold(ultralytics):
    descriptor = _descriptor()
    descriptor.parameters.confidence_threshold = 0.42
    annotator = _annotator_in_pipeline(descriptor)

    annotator.update()

    assert (
        ultralytics.model.received_confidence
        == descriptor.parameters.confidence_threshold
    )


def test_shrinking_narrows_hypothesis_region_around_its_center(ultralytics):
    descriptor = _descriptor()
    descriptor.parameters.shrink_bounding_box = 0.5
    annotator = _annotator_in_pipeline(descriptor)

    annotator.update()

    x1, y1, x2, y2 = PERSON_BOX.corners_in(_color_image())
    margin_x = (x2 - x1) * descriptor.parameters.shrink_bounding_box / 2
    margin_y = (y2 - y1) * descriptor.parameters.shrink_bounding_box / 2
    assert _only_hypothesis(annotator).roi.roi.get_corner_points() == (
        int(x1 + margin_x),
        int(y1 + margin_y),
        int(x2 - margin_x),
        int(y2 - margin_y),
    )


# %% depth


def test_point_cloud_holds_depth_measured_inside_bounding_box(ultralytics):
    annotator = _annotator_in_pipeline(_depth_descriptor())

    annotator.update()

    points = np.asarray(_only_hypothesis(annotator).points.points)
    assert len(points) == _area(_person_corners(_color_image()))
    np.testing.assert_allclose(
        points[:, 2], DEPTH_IN_MILLIMETERS / MILLIMETERS_PER_METER
    )


def test_point_cloud_is_built_on_smaller_depth_resolution(ultralytics):
    color_to_depth_ratio = 0.5
    annotator = _annotator_in_pipeline(
        _depth_descriptor(), color_to_depth_ratio=color_to_depth_ratio
    )

    annotator.update()

    depth_image = annotator.get_cas().get(CASViews.DEPTH_IMAGE)
    points = np.asarray(_only_hypothesis(annotator).points.points)
    assert len(points) == _area(_person_corners(depth_image))


def test_shrunk_cloud_is_added_as_annotation_when_requested(ultralytics):
    descriptor = _depth_descriptor()
    descriptor.parameters.shrink_bounding_box = 0.5
    descriptor.parameters.shrink_as_annotation = True
    annotator = _annotator_in_pipeline(descriptor)

    annotator.update()

    hypothesis = _only_hypothesis(annotator)
    full_area = _area(_person_corners(_color_image()))
    (shrunk_cloud,) = _annotations_of_type(hypothesis, CloudAnnotation)
    assert hypothesis.roi.roi.width * hypothesis.roi.roi.height == full_area
    assert 0 < len(shrunk_cloud.points.points) < full_area


def test_keypoint_clouds_cover_only_confident_keypoints(ultralytics):
    descriptor = _depth_descriptor()
    descriptor.parameters.generate_clouds_for_keypoints = True
    annotator = _annotator_in_pipeline(descriptor)

    annotator.update()

    keypoints = _keypoints_of_type(
        _only_hypothesis(annotator), KeypointAnnotation.KP_TYPE_3D
    )
    point_counts = {
        keypoint: len(cloud.points)
        for keypoint, (cloud, confidence) in zip(PoseKeypoint, keypoints.keypoints)
    }
    assert point_counts[PoseKeypoint.NOSE] > 0
    assert all(
        count == 0
        for keypoint, count in point_counts.items()
        if keypoint != PoseKeypoint.NOSE
    )
