"""Structured observations for deterministic perception playback."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import timedelta
from enum import Enum, StrEnum
from pathlib import Path

import numpy as np
import open3d
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from typing_extensions import ClassVar

from robokudo.types.annotation import (
    BoundingBox3DAnnotation,
    Classification,
    PoseAnnotation,
)
from robokudo.types.cv import ImageROI, Point2D, Rect
from robokudo.types.scene import ObjectHypothesis
from robokudo.types.tf import Pose

# %% object models


@dataclass(frozen=True)
class ObjectModel:
    """Identify a target object model and its original CAD mesh."""

    class_identifier: int
    """Classification identifier of the model."""

    mesh_filename: Path
    """Filename of the supplied millimetre-scale CAD mesh."""

    metres_per_mesh_unit: float = 0.001
    """Scale from the supplied mesh units to metres."""


class TetrisBlock(Enum):
    """The two supplied target object models."""

    FIRST = ObjectModel(1, Path("child_cube_0_colored.ply"))
    """First target model."""

    SECOND = ObjectModel(2, Path("child_cube_2_colored.ply"))
    """Second target model."""


class ScriptedIdentifier(StrEnum):
    """Names identifying the scripted pipeline and its outputs."""

    ENGINE = "scripted_tetris"
    """Analysis engine module name."""

    PIPELINE = "ScriptedTetrisPipeline"
    """Pipeline name in the annotator output map."""

    ANNOTATOR = "ScriptedObservationAnnotator"
    """Source name on generated annotations."""

    OUTPUT_BUFFER = "annotator_output_pipeline_map_buffer"
    """Existing RoboKudo blackboard output buffer key."""

    CAMERA = "synthetic_camera"
    """Reference frame for scripted camera-relative poses."""


class ScriptedResource(Enum):
    """Workspace resources consumed by scripted perception."""

    MESH_DIRECTORY = Path("robokudo_cad_data_new")
    """Directory containing the supplied CAD meshes."""

    def resolve(self) -> Path:
        """Locate the resource in the enclosing source checkout.

        :raises FileNotFoundError: If the checkout does not contain the resource.
        """
        for parent in Path(__file__).resolve().parents:
            candidate = parent / self.value
            if candidate.is_dir():
                return candidate
        raise FileNotFoundError(self.value)


class PreviewColor(Enum):
    """BGR colors in the diagnostic observation image."""

    BACKGROUND = (24, 24, 24)
    """Empty image regions."""

    FOREGROUND = (255, 180, 0)
    """Synthetic rectangular foreground regions."""

    TEXT = (255, 255, 255)
    """Scene and model labels."""


# %% camera metadata


@dataclass
class InvalidScriptedObservationError(ValueError):
    """A scripted scene cannot represent a valid camera observation."""

    reason: str
    """Description of the invalid scene or camera configuration."""

    def __str__(self) -> str:
        """Describe why the observation cannot be produced."""
        return self.reason


@dataclass(frozen=True)
class ScriptedCamera:
    """Describe the reference camera for synthetic observations."""

    frame: str = ScriptedIdentifier.CAMERA
    """Reference frame of detection poses."""

    width: int = 640
    """Image width in pixels."""

    height: int = 480
    """Image height in pixels."""

    focal_length_x: float = 500.0
    """Horizontal focal length in pixels."""

    focal_length_y: float = 500.0
    """Vertical focal length in pixels."""

    nanoseconds_per_second: ClassVar[int] = 1_000_000_000
    """Number of nanoseconds in each ROS timestamp second."""

    def create_info(self, capture_time: int) -> CameraInfo:
        """Create camera metadata stamped with the synthetic capture time."""
        information = CameraInfo()
        information.width = self.width
        information.height = self.height
        information.k = [
            self.focal_length_x,
            0.0,
            self.width / 2,
            0.0,
            self.focal_length_y,
            self.height / 2,
            0.0,
            0.0,
            1.0,
        ]
        information.header.frame_id = self.frame
        information.header.stamp.sec = capture_time // self.nanoseconds_per_second
        information.header.stamp.nanosec = capture_time % self.nanoseconds_per_second
        return information

    def create_region(self, geometry: ModelGeometry, pose: Pose) -> ImageROI:
        """Enclose the projected mesh in a cropped rectangular foreground mask.

        :raises InvalidScriptedObservationError: If a declared detection is not visible.
        """
        camera_vertices = Rotation.from_quat(pose.rotation).apply(geometry.vertices)
        camera_vertices += pose.translation
        if np.any(camera_vertices[:, 2] <= 0):
            raise InvalidScriptedObservationError(
                "Scripted mesh vertices must lie in front of the camera."
            )
        horizontal = (
            self.focal_length_x * camera_vertices[:, 0] / camera_vertices[:, 2]
            + self.width / 2
        )
        vertical = (
            self.focal_length_y * camera_vertices[:, 1] / camera_vertices[:, 2]
            + self.height / 2
        )
        left = max(0, int(np.floor(horizontal.min())))
        right = min(self.width, int(np.ceil(horizontal.max())))
        top = max(0, int(np.floor(vertical.min())))
        bottom = min(self.height, int(np.ceil(vertical.max())))
        if right <= left or bottom <= top:
            raise InvalidScriptedObservationError(
                "A declared scripted detection projects outside the image."
            )
        region = Rect(
            pos=Point2D(x=left, y=top), width=right - left, height=bottom - top
        )
        return ImageROI(
            roi=region,
            mask=np.ones((region.height, region.width), dtype=np.uint8),
        )


@dataclass(frozen=True)
class PlaybackTiming:
    """Specify the capture timeline of a scripted sequence."""

    frame_interval: timedelta = timedelta(seconds=1)
    """Time between consecutive scripted observations."""

    capture_start: int = field(default_factory=time.time_ns)
    """Capture time of the first observation, in nanoseconds since the epoch."""

    def __post_init__(self) -> None:
        """Require capture time to advance between observations."""
        if self.frame_interval <= timedelta(0):
            raise InvalidScriptedObservationError(
                "The scripted frame interval must be positive."
            )

    def capture_time(self, observation_index: int) -> int:
        """Return the capture time at the given position in the timeline."""
        interval = int(
            self.frame_interval.total_seconds() * ScriptedCamera.nanoseconds_per_second
        )
        return self.capture_start + observation_index * interval


# %% CAD geometry


@dataclass(frozen=True)
class ModelGeometry:
    """Metric mesh vertices retaining the original CAD origin."""

    vertices: NDArray[np.float64]
    """Mesh vertices in metres, without centering."""

    @classmethod
    def load(cls, block: TetrisBlock, mesh_directory: Path) -> ModelGeometry:
        """Read a supplied model and convert its vertices to metres."""
        mesh_path = mesh_directory / block.value.mesh_filename
        if not mesh_path.is_file():
            raise FileNotFoundError(mesh_path)
        mesh = open3d.io.read_triangle_mesh(str(mesh_path))
        return cls(np.asarray(mesh.vertices) * block.value.metres_per_mesh_unit)

    def create_bounding_box(self, pose: Pose, source: str) -> BoundingBox3DAnnotation:
        """Place the model-aligned enclosing box relative to the camera."""
        lower = self.vertices.min(axis=0)
        upper = self.vertices.max(axis=0)
        model_center = (lower + upper) / 2
        camera_center = Rotation.from_quat(pose.rotation).apply(model_center)
        camera_center += pose.translation
        dimensions = upper - lower
        return BoundingBox3DAnnotation(
            source=source,
            x_length=float(dimensions[0]),
            y_length=float(dimensions[1]),
            z_length=float(dimensions[2]),
            pose=Pose(translation=camera_center.tolist(), rotation=list(pose.rotation)),
        )


# %% scripted scenes


class ObservationStep(StrEnum):
    """The visibility and motion situations in the demonstration."""

    BOTH_VISIBLE = "Both blocks visible"
    """Both blocks are observed at their initial poses."""

    TRANSLATED = "Blocks translated"
    """Both blocks have changed position."""

    ROTATED = "Blocks rotated"
    """Both blocks have changed orientation."""

    FIRST_ABSENT = "First block absent"
    """Only the second block is observed."""

    BOTH_ABSENT = "Both blocks absent"
    """Neither block is observed."""

    REAPPEARED = "Both blocks reappeared"
    """Both blocks are observed again."""


@dataclass(frozen=True)
class ScriptedDetection:
    """Specify an observed object model and its camera-relative CAD pose."""

    block: TetrisBlock
    """Object model represented by this detection."""

    pose: Pose
    """Original CAD frame relative to the camera, in metres and xyzw order."""

    confidence: float = 1.0
    """Declared classification confidence of the synthetic detection."""

    def create_hypothesis(
        self, geometry: ModelGeometry, camera: ScriptedCamera, source: str
    ) -> ObjectHypothesis:
        """Represent the detection with camera-relative CAS annotations."""
        model = self.block.value
        return ObjectHypothesis(
            source=source,
            roi=camera.create_region(geometry, self.pose),
            annotations=[
                Classification(
                    source=source,
                    class_id=model.class_identifier,
                    classname=model.mesh_filename.stem,
                    confidence=self.confidence,
                ),
                PoseAnnotation(
                    source=source,
                    translation=list(self.pose.translation),
                    rotation=list(self.pose.rotation),
                ),
                geometry.create_bounding_box(self.pose, source),
            ],
        )


@dataclass(frozen=True)
class ScriptedObservation:
    """Specify the detections present in one scripted scene."""

    step: ObservationStep
    """Visibility or motion situation represented by the scene."""

    detections: tuple[ScriptedDetection, ...]
    """Visible detections in the scene."""


@dataclass(frozen=True)
class ScriptedScenario:
    """An ordered, finite sequence of scripted observations."""

    observations: tuple[ScriptedObservation, ...]
    """Observations in playback order."""

    def __post_init__(self) -> None:
        """Require at least one observation, even when its detection list is empty."""
        if not self.observations:
            raise InvalidScriptedObservationError(
                "A scripted scenario must contain at least one observation."
            )
