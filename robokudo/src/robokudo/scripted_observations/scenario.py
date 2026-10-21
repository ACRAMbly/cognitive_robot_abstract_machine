# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from robokudo.scripted_observations.models import TetrisModel
from robokudo.types.tf import Pose


# %% Camera descriptor


class CameraFrame(StrEnum):
    """Optical reference frame with X right, Y down, and Z forward."""

    OPTICAL = "scripted_camera_optical_frame"


@dataclass
class CameraSettings:
    """Editable calibration of the fixed optical camera."""

    image_width: int = 640
    """Horizontal image extent in pixels."""

    image_height: int = 480
    """Vertical image extent in pixels."""

    horizontal_field_of_view_degrees: float = 60.0
    """Horizontal angular extent of the camera image."""

    frame: str = CameraFrame.OPTICAL
    """Reference frame locating every object pose in an observation."""


# %% Scene descriptor


@dataclass
class BlockState:
    """One visible block located by its original CAD origin."""

    model: TetrisModel
    """CAD model describing this block's object kind."""

    pose: Pose
    """Camera-relative CAD-origin pose in metres and xyzw quaternion order."""


@dataclass
class TetrisScenario:
    """Editable scene for synthetic Tetris observations."""

    camera: CameraSettings = field(default_factory=CameraSettings)
    """Calibration and reference frame of the synthetic camera."""

    blocks: list[BlockState] = field(
        default_factory=lambda: [
            BlockState(
                TetrisModel.FIRST,
                Pose(translation=[-0.30, -0.15, 1.00], rotation=[0.0, 0.0, 0.0, 1.0]),
            ),
            BlockState(
                TetrisModel.SECOND,
                Pose(translation=[0.05, -0.15, 1.00], rotation=[0.0, 0.0, 0.0, 1.0]),
            ),
        ]
    )
    """Visible blocks at the initial observation."""
