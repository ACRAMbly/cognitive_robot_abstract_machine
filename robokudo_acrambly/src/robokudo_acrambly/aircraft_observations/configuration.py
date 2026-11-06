# %% Imports

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

from typing_extensions import TYPE_CHECKING

from robokudo.annotators.semantic_observations import ObservationPose
from robokudo.types.annotation import PoseAnnotation
from robokudo.types.scene import ObjectHypothesis
from robokudo.types.tf import Pose
from robokudo_acrambly.aircraft_observations.exceptions import (
    MissingAircraftFrameConfiguration,
)
from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from typing_extensions import Any


# %% Launcher configuration


class AircraftEnvironment(StrEnum):
    """
    Environment settings accepted by the aircraft analysis-engine launcher.
    """

    FRAME_CONFIGURATION = "AIRCRAFT_FRAME_CONFIGURATION"
    """Path to the application's explicit frame calibration document."""

    MESH_DIRECTORY = "AIRCRAFT_MESH_DIRECTORY"
    """Required directory supplying aircraft belief and preview geometry."""


class AircraftConfigurationField(StrEnum):
    """
    Fields in an application's aircraft frame configuration document.
    """

    WORLD_FRAME = "world_frame"
    """Declared world coordinate frame identifier."""

    CAMERA_FRAME = "camera_frame"
    """Declared optical camera coordinate frame identifier."""

    CAMERA_POSE = "camera_pose"
    """Camera origin and orientation in world coordinates."""


@dataclass
class AircraftFrameConfiguration:
    """
    Declared frames and the camera's world pose.
    """

    world_frame: str
    """
    Application-owned world coordinate frame identifier.
    """

    camera_frame: str
    """
    Distinct optical camera coordinate frame identifier.
    """

    camera_pose: Pose
    """
    Camera origin in world coordinates, in metres and xyzw quaternion order.
    """

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> AircraftFrameConfiguration:
        """
        Read the required frame identifiers and explicit camera world pose.
        """
        return cls(
            world_frame=payload[AircraftConfigurationField.WORLD_FRAME],
            camera_frame=payload[AircraftConfigurationField.CAMERA_FRAME],
            camera_pose=Pose(**payload[AircraftConfigurationField.CAMERA_POSE]),
        )

    @classmethod
    def from_environment(cls) -> AircraftFrameConfiguration:
        """
        Read the application's explicitly selected frame configuration file.
        """
        configuration_filename = os.environ.get(AircraftEnvironment.FRAME_CONFIGURATION)
        if configuration_filename is None:
            raise MissingAircraftFrameConfiguration(
                AircraftEnvironment.FRAME_CONFIGURATION
            )
        configuration_path = Path(configuration_filename)
        return cls.from_json(json.loads(configuration_path.read_text()))

    def create_frames(self) -> AircraftFrames:
        """
        Create detached frame references without registering world bodies.
        """
        ObservationPose.from_hypothesis(
            ObjectHypothesis(
                annotations=[
                    PoseAnnotation(
                        translation=self.camera_pose.translation,
                        rotation=self.camera_pose.rotation,
                    )
                ]
            )
        )
        world_body = Body(name=PrefixedName(self.world_frame))
        camera_body = Body(name=PrefixedName(self.camera_frame))
        world_T_camera = HomogeneousTransformationMatrix.from_xyz_quaternion(
            *self.camera_pose.translation,
            *self.camera_pose.rotation,
            reference_frame=world_body,
            child_frame=camera_body,
        )
        return AircraftFrames(self.world_frame, self.camera_frame, world_T_camera)
