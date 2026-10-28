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
from robokudo_acrambly.playback.schedule import CaptureSchedule

if TYPE_CHECKING:
    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft_observations.preview import AircraftPreview
    from robokudo_acrambly.aircraft_observations.scenario import (
        AircraftFrames,
        AircraftScenario,
    )
    from robokudo_acrambly.playback.clock import Clock
    from robokudo_acrambly.playback.schedule import ScheduledCapture


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

    preview: AircraftPreview | None = None
    """
    Optional display geometry prepared from the completed observation.
    """

    schedule: CaptureSchedule = field(init=False)
    """
    Finite capture times retained across pipeline ticks.
    """

    def __post_init__(self) -> None:
        """
        Initialize the public observation source.
        """
        BaseAnnotator.__init__(self, name=AircraftObservationLabel.SOURCE)
        self.frames.validate()
        self.scenario.timing.validate()
        for part in self.scenario.parts:
            part.validate_keyframes(self.scenario.timing)
            part.validate_absence_intervals(self.scenario.timing)
        self.schedule = CaptureSchedule(self.scenario.timing, self.clock)

    def update(self) -> Status:
        capture = self.schedule.schedule_capture()
        if isinstance(capture, Status):
            return capture
        pipeline = self.get_parent_pipeline()
        pipeline.create_new_cas()
        self.frames.write_metadata(pipeline.cas, capture.capture_timestamp)
        self.write_hypotheses(pipeline.cas, capture)
        self.write_preview(pipeline.cas)
        self.schedule.observation_index += 1
        return Status.SUCCESS

    def write_preview(self, observation: CAS) -> None:
        """
        Replace optional mesh display output for one completed capture.
        """
        if self.preview is None:
            return
        self.get_annotator_output_struct().set_geometries(
            self.preview.create_geometries(observation)
        )

    def write_hypotheses(self, observation: CAS, capture: ScheduledCapture) -> None:
        """
        Append the visible aircraft kinds at their scheduled mesh-origin poses.
        """
        for part in self.scenario.parts:
            if not part.is_visible(capture.elapsed_nanoseconds):
                continue
            object_pose = part.interpolate_pose(capture.elapsed_nanoseconds)
            hypothesis = ObjectHypothesis(
                source=self.name,
                id=f"{self.name}:{self.schedule.observation_index}:{part.model.value.label}",
                annotations=[
                    Classification(
                        source=self.name,
                        classification_type=AircraftObservationLabel.MODEL_CLASS,
                        classname=part.model.value.label,
                        confidence=1.0,
                    ),
                    PoseAnnotation(
                        source=self.name,
                        translation=list(object_pose.translation),
                        rotation=list(object_pose.rotation),
                    ),
                ],
            )
            ObservationPose.from_hypothesis(hypothesis)
            observation.annotations.append(hypothesis)
