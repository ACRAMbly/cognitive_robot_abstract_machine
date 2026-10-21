# %% Imports

from __future__ import annotations

import os
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo.descriptors.analysis_engines.scripted_tetris import (
    AnalysisEngine,
    MeshEnvironment,
)
from robokudo.scripted_observations.models import TetrisModel
from robokudo.scripted_observations.projection import MaskPixel
from robokudo.scripted_observations.scenario import CameraSettings
from robokudo.types.cv import Point2D, Rect
from robokudo.types.scene import ObjectHypothesis
from robokudo.types.tf import Pose

from .test_scripted_tetris import LocalMeshes, tetris_meshes


if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap


# %% Worked image projections


@dataclass
class OutlineExample:
    """
    Exact raster evidence for the local L-shaped CAD fixture.
    """

    object_pose: Pose
    """Camera-relative pose of the fixture's CAD origin."""

    image_region: Rect
    """
    Expected inclusive pixel support of the projected mesh.
    """

    foreground_pixels: list[Point2D]
    """Pixels inside the two arms of the projected fixture."""

    background_pixel: Point2D
    """
    Pixel in the L-shaped gap inside its convex enclosing outline.
    """


# %% Cropped foreground evidence


@pytest.mark.parametrize(
    "example",
    [
        OutlineExample(
            Pose(translation=[-0.30, -0.15, 1.0], rotation=[0.0, 0.0, 0.0, 1.0]),
            Rect(pos=Point2D(x=69, y=84), width=12, height=11),
            [Point2D(x=76, y=86), Point2D(x=71, y=91)],
            Point2D(x=74, y=89),
        ),
        OutlineExample(
            Pose(translation=[-0.20, -0.15, 1.0], rotation=[0.0, 0.0, 0.0, 1.0]),
            Rect(pos=Point2D(x=79, y=84), width=11, height=11),
            [Point2D(x=86, y=86), Point2D(x=81, y=91)],
            Point2D(x=84, y=89),
        ),
        OutlineExample(
            Pose(
                translation=[-0.20, -0.15, 1.0],
                rotation=Rotation.from_euler("z", np.pi / 2.0).as_quat().tolist(),
            ),
            Rect(pos=Point2D(x=69, y=84), width=12, height=11),
            [Point2D(x=78, y=91), Point2D(x=72, y=86)],
            Point2D(x=74, y=89),
        ),
    ],
)
def test_foreground_masks_follow_translated_and_rotated_cad_outlines(
    example: OutlineExample,
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    L-shaped gaps remain background at their configured image coordinates.
    """
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    engine.scenario.camera = CameraSettings(
        image_width=200, image_height=200, horizontal_field_of_view_degrees=90.0
    )
    engine.scenario.blocks[0].pose = example.object_pose
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    hypothesis = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)[0]
    region = hypothesis.roi.roi
    mask = hypothesis.roi.mask
    assert region.as_tuple() == example.image_region.as_tuple()
    assert mask.shape == (region.height, region.width)
    assert mask.dtype == np.uint8
    for pixel in example.foreground_pixels:
        assert (
            mask[pixel.y - region.pos.y, pixel.x - region.pos.x] == MaskPixel.FOREGROUND
        )
    assert (
        mask[
            example.background_pixel.y - region.pos.y,
            example.background_pixel.x - region.pos.x,
        ]
        == MaskPixel.BACKGROUND
    )


# %% Fixture and supplied CAD resources


class ProjectionResources(Enum):
    """
    Mesh resource sets accepted by the image evidence checks.
    """

    FIXTURE = auto()
    SUPPLIED = auto()


@pytest.fixture(params=list(ProjectionResources))
def projection_mesh_directory(
    request: pytest.FixtureRequest, tetris_meshes: LocalMeshes
) -> Path:
    """
    Select deterministic resources or locally available supplied CAD files.
    """
    if request.param is ProjectionResources.FIXTURE:
        return tetris_meshes.directory
    directory = Path(
        os.environ.get(MeshEnvironment.DIRECTORY, Path(__file__).resolve().parents[2])
    )
    if not all(
        (directory / model.value.mesh_filename).is_file() for model in TetrisModel
    ):
        pytest.skip("Supplied CAD resources are unavailable in the mesh directory.")
    return directory


def test_default_example_poses_stay_fully_in_frame_and_separate(
    projection_mesh_directory: Path,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Both projected blocks remain visible and disjoint during example motion.
    """
    engine = AnalysisEngine(mesh_directory=projection_mesh_directory)
    initial_translation = list(engine.scenario.blocks[0].pose.translation)
    for translation_offset, rotation_angle in [
        (0.0, 0.0),
        (0.10, 0.0),
        (0.10, np.pi / 4.0),
        (0.10, np.pi / 2.0),
    ]:
        block_pose = engine.scenario.blocks[0].pose
        block_pose.translation = list(initial_translation)
        block_pose.translation[0] += translation_offset
        block_pose.rotation = (
            Rotation.from_euler("z", rotation_angle).as_quat().tolist()
        )
        pipeline = engine.implementation()
        pipeline.setup()
        pipeline.tick_once()

        camera = engine.scenario.camera
        occupancy = np.zeros((camera.image_height, camera.image_width), dtype=np.uint8)
        hypotheses = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
        assert len(hypotheses) == len(engine.scenario.blocks)
        for hypothesis in hypotheses:
            region = hypothesis.roi.roi
            left, top, right, bottom = region.get_corner_points()
            assert 0 <= left < right <= camera.image_width
            assert 0 <= top < bottom <= camera.image_height
            assert hypothesis.roi.mask.shape == (region.height, region.width)
            assert np.isin(hypothesis.roi.mask, list(MaskPixel)).all()
            occupancy[top:bottom, left:right] += (
                hypothesis.roi.mask == MaskPixel.FOREGROUND
            )
        assert np.count_nonzero(occupancy > 1) == 0
