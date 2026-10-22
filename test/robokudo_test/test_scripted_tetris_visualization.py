# %% Imports

from __future__ import annotations

from dataclasses import astuple, dataclass, field
from enum import IntEnum
from unittest.mock import Mock

import cv2
import numpy
import open3d
import pytest
from py_trees.common import Status
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

import robokudo.vis.o3d_visualizer as native_geometry_windows
from robokudo.annotators.testing import EmptyAnnotator
from robokudo.descriptors.analysis_engines.scripted_tetris import AnalysisEngine
from robokudo.garden import grow_tree
from robokudo.scripted_observations.projection import MaskPixel
from robokudo.scripted_observations.scenario import (
    CameraSettings,
    PlaybackTiming,
    ScenarioPoses,
)
from robokudo.scripted_observations.visualization import (
    ImageColor,
    ObservationVisualization,
)
from robokudo.types.annotation import (
    BoundingBox3DAnnotation,
    Classification,
    PoseAnnotation,
    Shape,
)
from robokudo.types.scene import ObjectHypothesis
from robokudo.vis.cv_visualizer import CVVisualizer
from robokudo.vis.o3d_visualizer import O3DVisualizer
from robokudo.vis.visualizer import Visualizer

from .test_scripted_tetris import CaptureExample, ControlledClock, tetris_meshes


if TYPE_CHECKING:
    from typing_extensions import Iterator
    from .test_scripted_tetris import LocalMeshes
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap


# %% Illustrative camera output


@dataclass
class ImageExample:
    """
    Camera dimensions and native overlay margin for image evidence checks.
    """

    camera: CameraSettings = field(
        default_factory=lambda: CameraSettings(image_width=320, image_height=240)
    )
    """
    Calibration consumed by the public observation engine.
    """

    border_width_pixels: int = 2
    """
    Region edge excluded from foreground and background color comparisons.
    """


def test_completed_observation_draws_foreground_masks_and_image_regions(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Native image output aligns synthetic foreground with the completed CAS.
    """
    rectangle_drawing = Mock(wraps=cv2.rectangle)
    monkeypatch.setattr(cv2, "rectangle", rectangle_drawing)
    example = ImageExample()
    engine = AnalysisEngine(
        mesh_directory=tetris_meshes.directory, clock=ControlledClock()
    )
    engine.scenario.camera = example.camera
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    source = pipeline.get_annotators()[0]
    output = annotator_output_buffer.map[pipeline.name].outputs[source.name]
    camera = engine.scenario.camera
    assert output.image.shape == (camera.image_height, camera.image_width, 3)
    assert output.image.dtype == numpy.uint8
    hypothesis = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)[0]
    left, top, right, bottom = hypothesis.roi.roi.get_corner_points()
    mask = hypothesis.roi.mask
    border = example.border_width_pixels
    interior = output.image[top:bottom, left:right][border:-border, border:-border]
    interior_mask = mask[border:-border, border:-border]
    assert (
        interior[interior_mask == MaskPixel.FOREGROUND].tolist()
        == numpy.broadcast_to(
            astuple(ImageColor.FOREGROUND.value),
            interior[interior_mask == MaskPixel.FOREGROUND].shape,
        ).tolist()
    )
    assert (
        interior[interior_mask == MaskPixel.BACKGROUND].tolist()
        == numpy.broadcast_to(
            astuple(ImageColor.BACKGROUND.value),
            interior[interior_mask == MaskPixel.BACKGROUND].shape,
        ).tolist()
    )
    hypotheses = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    assert [call.args[1:3] for call in rectangle_drawing.call_args_list] == [
        (
            (detected.roi.roi.pos.x, detected.roi.roi.pos.y),
            (
                detected.roi.roi.pos.x + detected.roi.roi.width,
                detected.roi.roi.pos.y + detected.roi.roi.height,
            ),
        )
        for detected in hypotheses
    ]


def test_native_model_labels_use_the_emitted_classifications(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    The illustrative image labels identify the observed model kinds.
    """
    text_drawing = Mock(wraps=cv2.putText)
    monkeypatch.setattr(cv2, "putText", text_drawing)
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    hypotheses = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    assert [call.args[1] for call in text_drawing.call_args_list] == [
        pipeline.cas.filter_by_type(Classification, hypothesis.annotations)[0].classname
        for hypothesis in hypotheses
    ]


# %% Camera-relative geometry output


@pytest.mark.parametrize("quarter_turns", [0, 1])
def test_native_geometry_agrees_with_metric_cad_pose_and_offset_box(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    quarter_turns: int,
) -> None:
    """
    Visual meshes and boxes locate the emitted geometry without mutating it.
    """
    engine = AnalysisEngine(
        mesh_directory=tetris_meshes.directory, clock=ControlledClock()
    )
    engine.scenario.blocks[0].pose.rotation = (
        Rotation.from_euler("z", quarter_turns * ScenarioPoses().rotation_angle_radians)
        .as_quat()
        .tolist()
    )
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    source = pipeline.get_annotators()[0]
    geometries = (
        annotator_output_buffer.map[pipeline.name].outputs[source.name].geometries
    )
    hypotheses = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    assert [type(geometry) for geometry in geometries] == [
        geometry_type
        for _ in hypotheses
        for geometry_type in (
            open3d.geometry.TriangleMesh,
            open3d.geometry.OrientedBoundingBox,
            open3d.geometry.TriangleMesh,
        )
    ]
    geometry_iterator = iter(geometries)
    for block, hypothesis in zip(engine.scenario.blocks, hypotheses, strict=True):
        visual_mesh = next(geometry_iterator)
        visual_box = next(geometry_iterator)
        origin_frame = next(geometry_iterator)
        pose = pipeline.cas.filter_by_type(PoseAnnotation, hypothesis.annotations)[0]
        shape = pipeline.cas.filter_by_type(Shape, hypothesis.annotations)[0]
        box = pipeline.cas.filter_by_type(
            BoundingBox3DAnnotation, hypothesis.annotations
        )[0]
        reference_vertices = Rotation.from_quat(pose.rotation).apply(
            shape.geometry.mesh.vertices
        )
        reference_vertices += pose.translation

        assert numpy.asarray(visual_mesh.vertices) == pytest.approx(reference_vertices)
        assert visual_box.center == pytest.approx(box.pose.translation)
        assert visual_box.extent == pytest.approx(
            [box.x_length, box.y_length, box.z_length]
        )
        assert visual_box.R == pytest.approx(
            Rotation.from_quat(box.pose.rotation).as_matrix()
        )
        assert shape.geometry.mesh.bounds == pytest.approx(
            tetris_meshes.geometries[block.model].bounds
            * block.model.value.metres_per_mesh_unit
        )
        reference_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(
            size=ObservationVisualization.coordinate_axis_length
        )
        frame_vertices = Rotation.from_quat(pose.rotation).apply(
            numpy.asarray(reference_frame.vertices)
        )
        frame_vertices += pose.translation
        assert numpy.asarray(origin_frame.vertices) == pytest.approx(frame_vertices)


# %% Replacing and preserving output


def test_each_completed_capture_replaces_the_previous_visual_outputs(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    A later stationary capture receives its own image and visual geometry.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    source = pipeline.get_annotators()[0]
    output = annotator_output_buffer.map[pipeline.name].outputs[source.name]
    previous_image = output.image
    previous_geometries = output.geometries
    clock.timestamp_nanoseconds += engine.scenario.timing.sample_period_nanoseconds
    pipeline.tick_once()

    assert pipeline.status == Status.SUCCESS
    assert output.image is not previous_image
    assert output.geometries is not previous_geometries


def test_empty_observation_clears_previous_image_and_block_geometry(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    An empty completed scene replaces earlier visual evidence explicitly.
    """
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    engine.scenario.blocks.clear()
    pipeline = engine.implementation()
    pipeline.setup()
    source = pipeline.get_annotators()[0]
    output = annotator_output_buffer.map[pipeline.name].outputs[source.name]
    camera = engine.scenario.camera
    output.set_image(
        numpy.full(
            (camera.image_height, camera.image_width, 3),
            astuple(ImageColor.FOREGROUND.value),
            numpy.uint8,
        )
    )
    output.set_geometries([open3d.geometry.TriangleMesh.create_box()])
    pipeline.tick_once()

    assert output.geometries == []
    assert (
        output.image.tolist()
        == numpy.full(
            (camera.image_height, camera.image_width, 3),
            astuple(ImageColor.BACKGROUND.value),
            numpy.uint8,
        ).tolist()
    )


@pytest.mark.parametrize("exhausted", [False, True])
def test_waiting_and_exhausted_ticks_preserve_completed_visual_outputs(
    exhausted: bool,
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Ticks without a completed capture retain the last image and geometry.
    """
    engine = AnalysisEngine(
        mesh_directory=tetris_meshes.directory, clock=ControlledClock()
    )
    if exhausted:
        engine.scenario.timing.duration_nanoseconds = 0
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    source = pipeline.get_annotators()[0]
    output = annotator_output_buffer.map[pipeline.name].outputs[source.name]
    completed_observation = pipeline.get_cas()
    completed_image = output.image
    completed_geometries = output.geometries
    pipeline.tick_once()

    assert pipeline.status == (Status.FAILURE if exhausted else Status.RUNNING)
    assert pipeline.get_cas() is completed_observation
    assert output.image is completed_image
    assert output.geometries is completed_geometries


# %% Native windows without a desktop


class DesktopKey(IntEnum):
    """
    Existing native key events exercised without a display.
    """

    NONE = -1
    NEXT_ANNOTATOR = ord("n")
    PREVIOUS_ANNOTATOR = ord("p")


@dataclass
class InteractiveScene:
    """
    Window boundary retaining displayed geometry while processing events.
    """

    title: str
    """
    Native window title supplied by the real Open3D backend.
    """

    geometries: list[open3d.geometry.Geometry3D] = field(default_factory=list)
    """
    Geometry currently displayed by the window.
    """

    event_ticks: int = 0
    """
    Number of native event-loop iterations processed.
    """

    is_open: bool = True
    """
    Whether the window continues accepting native events.
    """

    def update_cloud(self, geometries: list[open3d.geometry.Geometry3D] | None) -> None:
        """
        Replace a scene when an annotator publishes visual geometry.
        """
        if geometries is None:
            return
        self.geometries = list(geometries)

    def tick(self) -> bool:
        """
        Process one native iteration and report whether the window is open.
        """
        self.event_ticks += 1
        return self.is_open


@pytest.fixture
def native_key_events(monkeypatch: pytest.MonkeyPatch) -> Iterator[Mock]:
    """
    Run real native visualizers with their desktop operations intercepted.
    """
    Visualizer.clear_visualizer_instances()
    key_events = Mock(return_value=DesktopKey.NONE)
    monkeypatch.setattr(cv2, "namedWindow", Mock())
    monkeypatch.setattr(cv2, "setMouseCallback", Mock())
    monkeypatch.setattr(cv2, "imshow", Mock())
    monkeypatch.setattr(cv2, "waitKey", key_events)
    monkeypatch.setattr(native_geometry_windows, "Viewer3D", InteractiveScene)
    yield key_events
    Visualizer.clear_visualizer_instances()


def test_native_windows_remain_interactive_after_observations_are_exhausted(
    tetris_meshes: LocalMeshes,
    node: Node,
    native_key_events: Mock,
) -> None:
    """
    Final-scene inspection and annotator navigation continue until native exit.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    engine.scenario.timing = PlaybackTiming(
        duration_nanoseconds=CaptureExample.SAMPLE_PERIOD_NANOSECONDS,
        sample_period_nanoseconds=CaptureExample.SAMPLE_PERIOD_NANOSECONDS,
    )
    pipeline = engine.implementation()
    comparison_annotator = EmptyAnnotator()
    pipeline.add_child(comparison_annotator)
    tree = grow_tree(
        pipeline,
        node=node,
        include_gui=True,
        visualizer_types=[CVVisualizer, O3DVisualizer],
    )
    tree.tick()
    clock.timestamp_nanoseconds += engine.scenario.timing.duration_nanoseconds
    tree.tick()
    completed_observation = pipeline.get_cas()
    source = pipeline.get_annotators()[0]
    output = source.get_annotator_output_struct()
    completed_image = output.image.copy()
    completed_geometries = output.geometries
    tree.tick()

    image_backend = next(
        visualizer
        for visualizer in Visualizer.instances
        if isinstance(visualizer, CVVisualizer)
    )
    geometry_backend = next(
        visualizer
        for visualizer in Visualizer.instances
        if isinstance(visualizer, O3DVisualizer)
    )
    scene = geometry_backend.viewer3d
    assert type(scene) is InteractiveScene
    previous_scene_ticks = scene.event_ticks
    previous_key_polls = native_key_events.call_count
    for _ in range(2):
        tree.tick()
        assert tree.root.children[0].status == Status.INVALID
        assert pipeline.get_cas() is completed_observation
    assert scene.event_ticks == previous_scene_ticks + 2
    assert native_key_events.call_count == previous_key_polls + 2
    assert output.image.tolist() == completed_image.tolist()
    assert output.geometries is completed_geometries

    assert image_backend.handle_keycallback(DesktopKey.NEXT_ANNOTATOR)
    tree.tick()
    assert (
        image_backend.shared_visualizer_state.active_annotator is comparison_annotator
    )
    assert (
        geometry_backend.shared_visualizer_state.active_annotator
        is comparison_annotator
    )
    assert scene.geometries == []
    assert image_backend.handle_keycallback(DesktopKey.PREVIOUS_ANNOTATOR)
    tree.tick()
    assert geometry_backend.shared_visualizer_state.active_annotator is source
    assert [type(geometry) for geometry in scene.geometries] == [
        type(geometry) for geometry in completed_geometries
    ]

    scene.is_open = False
    tree.tick()
    assert tree.root.children[0].status == Status.FAILURE
    assert pipeline.get_cas() is completed_observation


def test_headless_exhaustion_requires_no_native_event_processing(
    tetris_meshes: LocalMeshes,
    node: Node,
    native_key_events: Mock,
) -> None:
    """
    The finite headless engine completes without creating native windows.
    """
    engine = AnalysisEngine(
        mesh_directory=tetris_meshes.directory, clock=ControlledClock()
    )
    engine.scenario.timing.duration_nanoseconds = 0
    pipeline = engine.implementation()
    tree = grow_tree(pipeline, node=node, include_gui=False)
    tree.tick()
    tree.tick()

    assert tree.root.children[0].status == Status.FAILURE
    assert Visualizer.instances == []
    assert native_key_events.call_count == 0
