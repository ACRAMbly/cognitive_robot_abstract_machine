# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field

from builtin_interfaces.msg import Time
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.exceptions import InvalidObservationFrame, ObservationProblem
from robokudo.annotators.semantic_observations import ObservationCapture
from robokudo.cas import CAS
from robokudo.types.tf import Pose
from robokudo_acrambly.aircraft.models import AircraftModel

if TYPE_CHECKING:
    from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix


# %% Frame configuration


@dataclass
class AircraftFrames:
    """
    Explicit camera calibration retained across independent captures.
    """

    world_frame: str
    """Declared world coordinate frame supplied by the application."""

    camera_frame: str
    """
    Optical camera frame with X right, Y down, and Z forward.
    """

    camera_to_world_transform: HomogeneousTransformationMatrix
    """Rigid transform from the camera frame into the declared world frame."""

    def validate(self) -> None:
        """
        Require distinct frames and explicit finite rigid calibration.
        """
        if not self.world_frame:
            raise InvalidObservationFrame(ObservationProblem.WORLD_FRAME)
        if self.world_frame == self.camera_frame:
            raise InvalidObservationFrame(ObservationProblem.AMBIGUOUS_FRAME)
        observation = CAS()
        observation.camera_to_world_transform = self.camera_to_world_transform
        ObservationCapture(
            self.world_frame, self.camera_frame, Time()
        ).validate_camera_transform(observation)


# %% Independent part poses


@dataclass
class AircraftPartState:
    """
    One independently observed aircraft kind at its mesh-origin pose.
    """

    model: AircraftModel
    """
    Existing aircraft kind identifying the observed component.
    """

    pose: Pose
    """
    Camera-relative source mesh origin in metres and xyzw quaternion order.
    """


@dataclass
class AircraftLayout:
    """
    Editable spacing of independent aircraft parts for mesh inspection.
    """

    columns: int = 4
    """
    Number of parts arranged along each camera-X row.
    """

    horizontal_spacing: float = 0.4
    """
    Metres between neighbouring mesh origins along camera X.
    """

    vertical_spacing: float = 0.3
    """
    Metres between neighbouring rows along camera Y.
    """

    first_origin: Pose = field(
        default_factory=lambda: Pose(
            translation=[-0.6, -0.3, 1.0], rotation=[0.0, 0.0, 0.0, 1.0]
        )
    )
    """
    Camera-relative mesh origin of the first configured part.
    """

    def create_parts(self) -> list[AircraftPartState]:
        """
        Place each catalog kind at an independent camera-relative origin.
        """
        return [
            AircraftPartState(
                model=model,
                pose=Pose(
                    translation=[
                        self.first_origin.translation[0]
                        + index % self.columns * self.horizontal_spacing,
                        self.first_origin.translation[1]
                        + index // self.columns * self.vertical_spacing,
                        self.first_origin.translation[2],
                    ],
                    rotation=list(self.first_origin.rotation),
                ),
            )
            for index, model in enumerate(AircraftModel)
        ]


# %% Complete scenario


@dataclass
class AircraftScenario:
    """
    Editable independent aircraft parts in optical camera coordinates.
    """

    parts: list[AircraftPartState] = field(
        default_factory=lambda: AircraftLayout().create_parts()
    )
    """
    Configured aircraft kinds and their mesh-origin poses.
    """
