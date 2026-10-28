# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.semantic_observations import ObservationPose
from robokudo.types.annotation import Classification, PoseAnnotation
from robokudo.types.scene import ObjectHypothesis

if TYPE_CHECKING:
    from robokudo_acrambly.aircraft_observations.scenario import (
        AircraftFrames,
        AircraftScenario,
    )
    from robokudo_acrambly.scripted_observations.clock import Clock


# %% Observation source


class AircraftObservationLabel(StrEnum):
    """
    Public source and classification labels for aircraft observations.
    """

    SOURCE = "scripted_aircraft"
    MODEL_CLASS = "CLASS"


@dataclass(eq=False)
class AircraftObservationAnnotator(BaseAnnotator):
    """
    Emit camera-relative aircraft detections without persistent beliefs.
    """

    scenario: AircraftScenario
    """
    Editable aircraft parts supplied by the application.
    """

    frames: AircraftFrames
    """
    Explicit frame identifiers and camera-to-world transform.
    """

    clock: Clock
    """
    Clock supplying integer Unix capture timestamps.
    """

    observation_index: int = field(default=0, init=False)
    """
    Capture position used to distinguish independent detections.
    """

    def __post_init__(self) -> None:
        """
        Initialize the public observation source.
        """
        BaseAnnotator.__init__(self, name=AircraftObservationLabel.SOURCE)
        self.frames.validate()

    def update(self) -> Status:
        pipeline = self.get_parent_pipeline()
        pipeline.create_new_cas()
        observation = pipeline.cas
        observation.data_timestamp = self.clock.now_nanoseconds()
        observation.world_frame = self.frames.world_frame
        observation.camera_frame = self.frames.camera_frame
        observation.camera_to_world_transform = self.frames.camera_to_world_transform
        for part in self.scenario.parts:
            hypothesis = ObjectHypothesis(
                source=self.name,
                id=f"{self.name}:{self.observation_index}:{part.model.value.label}",
                annotations=[
                    Classification(
                        source=self.name,
                        classification_type=AircraftObservationLabel.MODEL_CLASS,
                        classname=part.model.value.label,
                        confidence=1.0,
                    ),
                    PoseAnnotation(
                        source=self.name,
                        translation=list(part.pose.translation),
                        rotation=list(part.pose.rotation),
                    ),
                ],
            )
            ObservationPose.from_hypothesis(hypothesis)
            observation.annotations.append(hypothesis)
        self.observation_index += 1
        return Status.SUCCESS
