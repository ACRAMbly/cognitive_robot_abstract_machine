"""
Tests for detecting objects with YOLO and annotating them as hypotheses.

The YOLO and SAM models are replaced by stand-ins reporting scripted results, so no
weights are downloaded.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np
import open3d as o3d
import pytest
from py_trees.common import Status

import robokudo.annotators.yolo_annotator
from robokudo.annotators.yolo_annotator import YoloAnnotator
from robokudo.cas import CASViews
from robokudo.descriptors.analysis_engines.yolo_demo_from_storage import (
    AnalysisEngine as YoloDemoAnalysisEngine,
)
from robokudo.exceptions import DetectionModelNotConfigured
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import Classification
from robokudo.types.scene import HumanHypothesis, ObjectHypothesis
from robokudo.utils.file_loader import FileLoader
from ..test_analysis_engine_query_composition import (
    bounded_build_time,
    install_fake_collection_reader_descriptor,
)
from ._yolo_stand_in import (
    NormalizedBox,
    ScriptedDetection,
    ScriptedUltralytics,
    ScriptedYoloModel,
)

pytestmark = pytest.mark.usefixtures("annotator_output_buffer")

# %% scene setup


@dataclass
class CupScene:
    """
    A camera view of a cup at a constant distance, which the scripted model detects.
    """

    class_names: dict[int, str] = field(
        default_factory=lambda: {
            0: YoloAnnotator.person_class_name,
            41: "cup",
            46: "banana",
        }
    )
    """
    The classes the scripted model detects, by class id.
    """

    cup_class_id: int = 41
    """
    Class id of the cup the scripted model detects.
    """

    cup_box: NormalizedBox = field(
        default_factory=lambda: NormalizedBox(
            left=0.25, top=0.25, right=0.75, bottom=0.75
        )
    )
    """
    Where the cup is detected in every image.
    """

    pretrained_model: str = "yolov8n.pt"
    """
    The pretrained model the annotators under test load.
    """

    color_image_shape: tuple[int, int, int] = (40, 80, 3)
    """
    Shape of the color images in the CAS.
    """

    depth_in_millimeters: int = 1000
    """
    Distance every depth image pixel measures.
    """

    millimeters_per_meter: float = 1000.0
    """
    Scale between the depth image values and point coordinates.
    """

    def color_image(self) -> np.ndarray:
        """
        :return: A black color image of the scene's shape.
        """
        return np.zeros(self.color_image_shape, dtype=np.uint8)

    def cup_corners(self) -> list[int]:
        """
        :return: The corners (x1, y1, x2, y2) of the cup in color image pixels.
        """
        return [int(corner) for corner in self.cup_box.corners_in(self.color_image())]

    def descriptor(self) -> YoloAnnotator.Descriptor:
        """
        :return: A descriptor loading the pretrained model and ignoring depth.
        """
        descriptor = YoloAnnotator.Descriptor()
        descriptor.parameters.pretrained_model = self.pretrained_model
        descriptor.parameters.threshold = 0.5
        descriptor.parameters.global_with_depth = False
        return descriptor

    def annotator_in_pipeline(
        self, descriptor: YoloAnnotator.Descriptor, color_to_depth_ratio: float = 1.0
    ) -> YoloAnnotator:
        """
        :param color_to_depth_ratio: Size of the depth image relative to the color
            image.
        :return: An annotator whose CAS holds a color image and a matching depth image.
        """
        annotator = YoloAnnotator(descriptor=descriptor)
        annotator.setup()
        pipeline = Pipeline("Sequence")
        pipeline.add_child(annotator)
        pipeline.setup()

        color_height, color_width = self.color_image_shape[:2]
        depth_height = int(color_height * color_to_depth_ratio)
        depth_width = int(color_width * color_to_depth_ratio)
        cas = annotator.get_cas()
        cas.set(CASViews.COLOR_IMAGE, self.color_image())
        cas.set(
            CASViews.DEPTH_IMAGE,
            np.full(
                (depth_height, depth_width),
                self.depth_in_millimeters,
                dtype=np.uint16,
            ),
        )
        cas.set(
            CASViews.COLOR2DEPTH_RATIO, (color_to_depth_ratio, color_to_depth_ratio)
        )
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

    def cup_shaped_hypothesis(self) -> ObjectHypothesis:
        """
        :return: A hypothesis covering exactly the region the cup is detected in.
        """
        x1, y1, x2, y2 = self.cup_corners()
        return _hypothesis_at(x=x1, y=y1, width=x2 - x1, height=y2 - y1)


@pytest.fixture
def scene() -> CupScene:
    """
    The camera view the annotators under test detect a cup in.
    """
    return CupScene()


@pytest.fixture
def ultralytics(monkeypatch, scene: CupScene) -> ScriptedUltralytics:
    """
    Make the annotator load scripted models detecting the scene's cup instead of real
    ones.
    """
    scripted = ScriptedUltralytics(
        model=ScriptedYoloModel(
            names=scene.class_names,
            detections=[
                ScriptedDetection(
                    class_id=scene.cup_class_id, confidence=0.8, box=scene.cup_box
                )
            ],
        )
    )
    monkeypatch.setattr(robokudo.annotators.yolo_annotator, "YOLO", scripted.load_model)
    monkeypatch.setattr(
        robokudo.annotators.yolo_annotator, "SAM", scripted.load_segmentation_model
    )
    return scripted


def _depth_descriptor(scene: CupScene) -> YoloAnnotator.Descriptor:
    descriptor = scene.descriptor()
    descriptor.parameters.global_with_depth = True
    return descriptor


def _hypotheses(annotator: YoloAnnotator) -> list[ObjectHypothesis]:
    return annotator.get_cas().filter_annotations_by_type(ObjectHypothesis)


def _only_hypothesis(annotator: YoloAnnotator) -> ObjectHypothesis:
    (hypothesis,) = _hypotheses(annotator)
    return hypothesis


def _classification(hypothesis: ObjectHypothesis) -> Classification:
    (classification,) = [
        annotation
        for annotation in hypothesis.annotations
        if isinstance(annotation, Classification)
    ]
    return classification


def _hypothesis_at(x: int, y: int, width: int, height: int) -> ObjectHypothesis:
    hypothesis = ObjectHypothesis()
    hypothesis.roi.roi.pos.x = x
    hypothesis.roi.roi.pos.y = y
    hypothesis.roi.roi.width = width
    hypothesis.roi.roi.height = height
    return hypothesis


# %% model loading


def test_pretrained_model_is_loaded_by_name(ultralytics, scene):
    descriptor = scene.descriptor()

    YoloAnnotator(descriptor=descriptor).setup()

    assert ultralytics.loaded_model_locations == [
        descriptor.parameters.pretrained_model
    ]


def test_annotator_without_model_raises(ultralytics, scene):
    descriptor = scene.descriptor()
    descriptor.parameters.pretrained_model = None

    with pytest.raises(DetectionModelNotConfigured):
        YoloAnnotator(descriptor=descriptor)


def test_custom_weights_and_class_names_are_read_from_ros_package(
    ultralytics, scene, monkeypatch, tmp_path
):
    monkeypatch.setattr(
        FileLoader,
        "get_path_to_file_in_ros_package",
        staticmethod(
            lambda ros_pkg_name, relative_path: tmp_path / ros_pkg_name / relative_path
        ),
    )
    descriptor = scene.descriptor()
    parameters = descriptor.parameters
    parameters.pretrained_model = None
    parameters.ros_package_name = "custom_models"
    parameters.weights_path = "models/best.pt"
    class_names_file = (
        tmp_path / parameters.ros_package_name / parameters.class_names_path
    )
    class_names_file.parent.mkdir(parents=True)
    class_names_file.write_text(json.dumps({"3": "mug"}), encoding="utf-8")

    annotator = YoloAnnotator(descriptor=descriptor)
    annotator.setup()

    assert ultralytics.loaded_model_locations == [
        str(tmp_path / parameters.ros_package_name / parameters.weights_path)
    ]
    assert annotator.class_names == {3: "mug"}


# %% detection


def test_detection_becomes_hypothesis_classified_as_detected_class(ultralytics, scene):
    annotator = scene.annotator_in_pipeline(scene.descriptor())

    status = annotator.compute()

    assert status == Status.SUCCESS
    hypothesis = _only_hypothesis(annotator)
    assert type(hypothesis) is ObjectHypothesis
    classification = _classification(hypothesis)
    assert classification.classname == scene.class_names[scene.cup_class_id]
    assert classification.class_id == scene.cup_class_id
    assert classification.confidence == pytest.approx(
        ultralytics.model.detections[0].confidence
    )
    assert classification.source == annotator.name


def test_hypothesis_region_is_detection_bounding_box(ultralytics, scene):
    annotator = scene.annotator_in_pipeline(scene.descriptor())

    annotator.compute()

    assert _only_hypothesis(annotator).roi.roi.get_corner_points() == tuple(
        scene.cup_corners()
    )


def test_person_detection_becomes_human_hypothesis(ultralytics, scene):
    person_class_id = next(
        class_id
        for class_id, name in scene.class_names.items()
        if name == YoloAnnotator.person_class_name
    )
    ultralytics.model.detections[0].class_id = person_class_id
    annotator = scene.annotator_in_pipeline(scene.descriptor())

    annotator.compute()

    assert type(_only_hypothesis(annotator)) is HumanHypothesis


def test_detection_of_ignored_class_is_not_annotated(ultralytics, scene):
    descriptor = scene.descriptor()
    descriptor.parameters.ignored_class_names = [
        f" {scene.class_names[scene.cup_class_id]} "
    ]
    annotator = scene.annotator_in_pipeline(descriptor)

    annotator.compute()

    assert _hypotheses(annotator) == []


def test_model_predicts_with_configured_threshold(ultralytics, scene):
    descriptor = scene.descriptor()
    descriptor.parameters.threshold = 0.42
    annotator = scene.annotator_in_pipeline(descriptor)

    annotator.compute()

    assert ultralytics.model.received_confidence == descriptor.parameters.threshold


# %% masks


def test_mask_fills_bounding_box_without_precision_mode(ultralytics, scene):
    annotator = scene.annotator_in_pipeline(scene.descriptor())

    annotator.compute()

    x1, y1, x2, y2 = scene.cup_corners()
    np.testing.assert_array_equal(
        _only_hypothesis(annotator).roi.mask,
        np.full(
            (y2 - y1, x2 - x1), YoloAnnotator.hypothesis_mask_foreground, dtype=np.uint8
        ),
    )


def _precision_descriptor(scene: CupScene) -> YoloAnnotator.Descriptor:
    descriptor = scene.descriptor()
    descriptor.parameters.precision_mode = True
    return descriptor


def _segmented_cup_mask(
    ultralytics: ScriptedUltralytics, scene: CupScene
) -> np.ndarray:
    """
    :return: The segmentation of the cup, cropped to its bounding box.
    """
    image = scene.color_image()
    segmentation = ultralytics.segmentation_model.segment(
        image, scene.cup_box.corners_in(image)
    )
    x1, y1, x2, y2 = scene.cup_corners()
    return (
        segmentation[y1:y2, x1:x2] * YoloAnnotator.hypothesis_mask_foreground
    ).astype(np.uint8)


def test_precision_mode_mask_is_segmentation_of_detection(ultralytics, scene):
    annotator = scene.annotator_in_pipeline(_precision_descriptor(scene))

    annotator.compute()

    np.testing.assert_array_equal(
        _only_hypothesis(annotator).roi.mask, _segmented_cup_mask(ultralytics, scene)
    )


def test_precision_mode_erosion_shrinks_segmentation(ultralytics, scene):
    descriptor = _precision_descriptor(scene)
    descriptor.parameters.precision_mode_apply_erosion = True
    descriptor.parameters.precision_mode_erosion_kernel_size = 3
    annotator = scene.annotator_in_pipeline(descriptor)

    annotator.compute()

    eroded = (
        _only_hypothesis(annotator).roi.mask == YoloAnnotator.hypothesis_mask_foreground
    )
    segmented = (
        _segmented_cup_mask(ultralytics, scene)
        == YoloAnnotator.hypothesis_mask_foreground
    )
    assert np.all(segmented[eroded])
    assert eroded.sum() < segmented.sum()


# %% depth


def test_point_cloud_holds_depth_measured_inside_bounding_box(ultralytics, scene):
    annotator = scene.annotator_in_pipeline(_depth_descriptor(scene))

    annotator.compute()

    x1, y1, x2, y2 = scene.cup_corners()
    points = np.asarray(_only_hypothesis(annotator).points.points)
    assert len(points) == (x2 - x1) * (y2 - y1)
    np.testing.assert_allclose(
        points[:, 2], scene.depth_in_millimeters / scene.millimeters_per_meter
    )


def test_region_detected_on_smaller_depth_resolution_is_in_color_pixels(
    ultralytics, scene
):
    annotator = scene.annotator_in_pipeline(
        _depth_descriptor(scene), color_to_depth_ratio=0.5
    )

    annotator.compute()

    hypothesis = _only_hypothesis(annotator)
    x1, y1, x2, y2 = scene.cup_corners()
    assert hypothesis.roi.roi.get_corner_points() == (x1, y1, x2, y2)
    assert hypothesis.roi.mask.shape == (y2 - y1, x2 - x1)


# %% existing hypotheses


def _existing_hypothesis_descriptor(scene: CupScene) -> YoloAnnotator.Descriptor:
    descriptor = scene.descriptor()
    descriptor.parameters.classify_existing_hypotheses = True
    return descriptor


def _annotator_with_existing_hypothesis(
    scene: CupScene, descriptor: YoloAnnotator.Descriptor, existing: ObjectHypothesis
) -> YoloAnnotator:
    annotator = scene.annotator_in_pipeline(descriptor)
    annotator.get_cas().annotations.append(existing)
    return annotator


def test_detection_overlapping_existing_hypothesis_classifies_it(ultralytics, scene):
    existing = scene.cup_shaped_hypothesis()
    annotator = _annotator_with_existing_hypothesis(
        scene, _existing_hypothesis_descriptor(scene), existing
    )

    annotator.compute()

    assert _hypotheses(annotator) == [existing]
    assert _classification(existing).classname == scene.class_names[scene.cup_class_id]


def test_detection_apart_from_existing_hypothesis_creates_new_one(ultralytics, scene):
    existing = _hypothesis_at(x=0, y=0, width=5, height=5)
    annotator = _annotator_with_existing_hypothesis(
        scene, _existing_hypothesis_descriptor(scene), existing
    )

    annotator.compute()

    assert len(_hypotheses(annotator)) == 2
    assert existing.annotations == []


def test_existing_hypotheses_are_left_alone_unless_enabled(ultralytics, scene):
    existing = scene.cup_shaped_hypothesis()
    annotator = _annotator_with_existing_hypothesis(scene, scene.descriptor(), existing)

    annotator.compute()

    assert len(_hypotheses(annotator)) == 2
    assert existing.annotations == []


# %% demo analysis engine


def test_demo_pipeline_contains_yolo_annotator(ultralytics, monkeypatch):
    install_fake_collection_reader_descriptor(monkeypatch)
    with bounded_build_time():
        pipeline = YoloDemoAnalysisEngine().implementation()

    assert any(isinstance(node, YoloAnnotator) for node in pipeline.children)


def test_building_demo_pipeline_loads_no_model(ultralytics, monkeypatch):
    """
    Models are loaded during setup, so that building every analysis engine, as the test
    suite does, downloads no weights.
    """
    install_fake_collection_reader_descriptor(monkeypatch)
    with bounded_build_time():
        YoloDemoAnalysisEngine().implementation()

    assert ultralytics.loaded_model_locations == []
