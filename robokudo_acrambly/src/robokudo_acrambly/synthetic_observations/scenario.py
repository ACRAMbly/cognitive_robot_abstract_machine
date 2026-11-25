"""
Scenarios of scripted observations: which parts the camera sees, and where each one is
at any time of the observation sequence.
"""

from __future__ import annotations

# %% Imports

import math
from dataclasses import dataclass, field
from datetime import timedelta

from krrood.patterns.classproperty import classproperty
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point2,
    Point3,
    Vector3,
)
from semantic_digital_twin.datastructures.camera_model import PinholeCameraModel
from semantic_digital_twin.datastructures.camera_resolution import CameraResolution
from typing_extensions import Sequence

from robokudo_acrambly.synthetic_observations.playback import PassSchedule


# %% Part paths


@dataclass(kw_only=True)
class PartPath:
    """
    The smooth, repeating path of one part in the camera optical frame: the part sways
    along a straight line through its centre while turning at a constant rate.
    """

    centre: Point3
    """
    The middle of the line the part sways along.
    """

    drift_direction: Vector3
    """
    The unit direction of the line the part sways along.
    """

    drift_amplitude: float
    """
    The farthest the part gets from :attr:`centre`, in metres.
    """

    drift_period: timedelta
    """
    How long the part takes to sway there and back.
    """

    drift_phase: float
    """
    Where in its sway the part is at the start of the sequence, in radians.
    """

    turn_axis: Vector3
    """
    The unit axis the part turns about.
    """

    initial_heading: float
    """
    How far the part is turned about :attr:`turn_axis` at the start of the sequence, in
    radians.
    """

    turn_rate: float
    """
    How fast the part turns about :attr:`turn_axis`, in radians per second.
    """

    def pose_at(self, *, elapsed: timedelta) -> HomogeneousTransformationMatrix:
        """
        :param elapsed: How long after the start of the sequence.
        :return: The pose of the part at that time.
        """
        seconds = elapsed.total_seconds()
        sway = self.drift_amplitude * math.sin(
            2.0 * math.pi * seconds / self.drift_period.total_seconds()
            + self.drift_phase
        )
        x, y, z = self.centre.to_np()[:3] + self.drift_direction.to_np()[:3] * sway
        return HomogeneousTransformationMatrix.from_xyz_axis_angle(
            x=x,
            y=y,
            z=z,
            axis=self.turn_axis,
            angle=self.initial_heading + self.turn_rate * seconds,
        )


@dataclass(kw_only=True)
class ScenarioPart:
    """
    One part a scenario shows the camera.
    """

    label: str
    """
    The class the part is classified as.
    """

    path: PartPath
    """
    Where the part is over the sequence.
    """


# %% Layout


@dataclass(kw_only=True)
class PlaneLayout:
    """
    Parts set out in a grid on a horizontal plane in front of a level camera, each
    swaying about its grid point and turning about the vertical.

    Neighbouring parts stay at least :attr:`spacing` minus twice :attr:`drift_amplitude`
    apart.
    """

    plane_height: float = 0.25
    """
    How far the plane lies below the optical axis, in metres.
    """

    nearest_depth: float = 1.6
    """
    How far in front of the camera the nearest row lies, in metres.
    """

    spacing: float = 0.45
    """
    The distance between neighbouring grid points, in metres.
    """

    columns: int = 4
    """
    How many parts a row holds at most.
    """

    drift_amplitude: float = 0.06
    """
    The farthest a part gets from its grid point, in metres.
    """

    drift_period: timedelta = timedelta(seconds=12)
    """
    How long a part takes to sway there and back.
    """

    turn_rate: float = math.radians(10.0)
    """
    How fast every part turns, in radians per second; neighbours turn in opposite
    senses.
    """

    @classproperty
    def vertical_axis(cls) -> Vector3:
        """
        Up, in the optical frame of a level camera, whose y axis points down.
        """
        return Vector3(x=0.0, y=-1.0, z=0.0)

    def arrange(self, *, labels: Sequence[str]) -> list[ScenarioPart]:
        """
        :param labels: The labels of the parts, in the order they fill the grid from the
            nearest row, left to right.
        :return: One part per label; neighbours sway across each other's lines and start
            at different points of their sway and heading.
        """
        used_columns = min(self.columns, len(labels))
        parts = []
        for index, label in enumerate(labels):
            row, column = divmod(index, self.columns)
            offset = 2.0 * math.pi * index / len(labels)
            parts.append(
                ScenarioPart(
                    label=label,
                    path=PartPath(
                        centre=Point3(
                            x=(column - (used_columns - 1) / 2.0) * self.spacing,
                            y=self.plane_height,
                            z=self.nearest_depth + row * self.spacing,
                        ),
                        drift_direction=(
                            Vector3(x=1.0, y=0.0, z=0.0)
                            if index % 2 == 0
                            else Vector3(x=0.0, y=0.0, z=1.0)
                        ),
                        drift_amplitude=self.drift_amplitude,
                        drift_period=self.drift_period,
                        drift_phase=offset,
                        turn_axis=self.vertical_axis,
                        initial_heading=offset,
                        turn_rate=self.turn_rate if index % 2 == 0 else -self.turn_rate,
                    ),
                )
            )
        return parts


# %% Scenario


@dataclass(kw_only=True)
class Scenario:
    """
    The parts a camera sees during an observation sequence, and when that camera
    captures its passes.
    """

    parts: list[ScenarioPart]
    """
    The parts in view, each with its own path.
    """

    schedule: PassSchedule = field(default_factory=PassSchedule)
    """
    When the passes are captured.
    """

    camera_frame: str = "synthetic_camera_optical_frame"
    """
    The name of the optical frame the part poses are given in.
    """

    camera_model: PinholeCameraModel = field(
        default_factory=lambda: PinholeCameraModel(
            image_resolution=CameraResolution(width=640, height=480),
            focal_length_x=525.0,
            focal_length_y=525.0,
            principal_point=Point2(x=320.0, y=240.0),
        )
    )
    """
    The calibration of the camera.
    """
