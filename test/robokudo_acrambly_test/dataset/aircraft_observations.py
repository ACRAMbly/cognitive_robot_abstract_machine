# %% Imports

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from enum import IntEnum

import numpy
from builtin_interfaces.msg import Time
from typing_extensions import TYPE_CHECKING

from robokudo.types.annotation import Classification, PoseAnnotation
from robokudo.types.scene import ObjectHypothesis
from robokudo_acrambly.aircraft.models import AircraftModel

from ...robokudo_test.dataset.semantic_observations import ObservationExample

if TYPE_CHECKING:
    from ...robokudo_test.dataset.semantic_observations import BeliefSnapshot

    from uuid import UUID

    from robokudo.cas import CAS
    from robokudo.types.belief_state import ObjectBeliefState
    from robokudo_acrambly.aircraft.types import AircraftPart
    from semantic_digital_twin.world_description.geometry import Shape
    from semantic_digital_twin.world_description.world_entity import Body


# %% Observation stages


class AircraftStage(IntEnum):
    """
    Successive captures of moving and temporarily absent aircraft parts.
    """

    INITIAL = 0
    TRANSLATED = 1
    ROTATED = 2
    PARTLY_ABSENT = 3
    ABSENT = 4
    REAPPEARED = 5


@dataclass
class AircraftSequence:
    """
    Deterministic camera observations with independently specified world poses.
    """

    capture: ObservationExample = field(default_factory=ObservationExample)
    """
    Camera calibration and initial capture metadata.
    """

    translation_step: float = 0.01
    """
    Metres of camera-x motion between captures.
    """

    separation: float = 0.02
    """
    Camera-x spacing of the independent parts.
    """

    absent_models: frozenset[AircraftModel] = field(
        default_factory=lambda: frozenset(list(AircraftModel)[::2])
    )
    """
    Nonempty proper subset hidden during partial absence.
    """

    def populate(
        self, cas: CAS, stage: AircraftStage
    ) -> dict[AircraftModel, ObjectHypothesis]:
        """
        Fill a fresh capture with plain camera poses and changing detection IDs.
        """
        capture = deepcopy(self.capture)
        capture.capture_time = Time(
            sec=self.capture.capture_time.sec + int(stage),
            nanosec=self.capture.capture_time.nanosec,
        )
        capture.configure_capture(cas)
        observations = {}
        for index, model in enumerate(AircraftModel):
            if stage is AircraftStage.ABSENT:
                continue
            if stage is AircraftStage.PARTLY_ABSENT and model in self.absent_models:
                continue
            hypothesis = ObjectHypothesis(
                id=f"{stage.value}:{index}",
                annotations=[
                    Classification(classname=model.value.label),
                    PoseAnnotation(
                        translation=[
                            self.capture.camera_pose.translation[0]
                            + index * self.separation
                            + int(stage) * self.translation_step,
                            *self.capture.camera_pose.translation[1:],
                        ],
                        rotation=(
                            [0.0, 0.0, 0.5**0.5, 0.5**0.5]
                            if stage >= AircraftStage.ROTATED
                            else deepcopy(self.capture.camera_pose.rotation)
                        ),
                    ),
                ],
            )
            observations[model] = hypothesis
        cas.annotations.extend(observations.values())
        return observations

    def expected_transform(
        self, model: AircraftModel, stage: AircraftStage
    ) -> numpy.ndarray:
        """
        Return the worked quarter-turn camera conversion for a part origin.
        """
        index = list(AircraftModel).index(model)
        world_T_part = numpy.eye(4)
        world_T_part[:3, 3] = self.capture.world_pose.translation
        world_T_part[1, 3] += (
            index * self.separation + int(stage) * self.translation_step
        )
        world_T_part[:3, :3] = (
            [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
            if stage >= AircraftStage.ROTATED
            else [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        )
        return world_T_part


# %% Retained identity


@dataclass
class AircraftIdentity:
    """
    Accepted identity and geometry retained across later captures.
    """

    belief: ObjectBeliefState
    """
    Persistent object belief instance.
    """

    body: Body
    """
    Original physical-part body.
    """

    identifier: UUID
    """
    Persistent UUID captured at creation.
    """

    annotation: AircraftPart
    """
    Original semantic annotation rooted at the body.
    """

    visual_shapes: list[Shape]
    """
    Original visual shapes in insertion order.
    """

    collision_shapes: list[Shape]
    """
    Original collision shapes in insertion order.
    """


# %% Accepted part state


@dataclass
class AcceptedAircraftPart:
    """Identity and last accepted evidence of one physical aircraft part."""

    identity: AircraftIdentity
    """Original belief, body, annotation and model shapes."""

    snapshot: BeliefSnapshot
    """Accepted persistent state before the next capture."""

    detection_identifier: str
    """Detection identifier of the last accepted observation."""

    stage: AircraftStage
    """Capture stage defining the last accepted pose and timestamp."""
