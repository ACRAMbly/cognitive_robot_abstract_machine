# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.core import BaseAnnotator
from robokudo_acrambly.scripted_observations.camera import ObservationCamera
from robokudo_acrambly.scripted_observations.geometry import ModelGeometry
from robokudo_acrambly.scripted_observations.projection import ForegroundProjection
from robokudo_acrambly.scripted_observations.visualization import (
    ObservationVisualization,
)
from robokudo.types.annotation import Classification
from robokudo.types.scene import ObjectHypothesis


if TYPE_CHECKING:
    from pathlib import Path

    from robokudo.cas import CAS
    from robokudo_acrambly.scripted_observations.clock import Clock
    from robokudo_acrambly.scripted_observations.models import TetrisModel
    from robokudo_acrambly.scripted_observations.scenario import TetrisScenario


# %% Observation annotations


class ObservationLabel(StrEnum):
    """
    Labels identifying the synthetic observation source and classification kind.
    """

    SOURCE = "scripted_tetris"
    MODEL_CLASS = "CLASS"


@dataclass
class ScheduledCapture:
    """
    Capture offset and Unix timestamp for one due observation.
    """

    elapsed_nanoseconds: int
    """
    Scheduled time relative to the sequence's first capture.
    """

    capture_timestamp: int
    """
    Unix timestamp of the scheduled capture.
    """


@dataclass(eq=False)
class ScriptedObservationAnnotator(BaseAnnotator):
    """
    Emit configured object hypotheses into the pipeline CAS.
    """

    scenario: TetrisScenario
    """
    Scene descriptor supplying visible models and their camera-relative poses.
    """

    mesh_directory: Path
    """
    Directory containing the block CAD resources.
    """

    clock: Clock
    """
    Clock supplying Unix capture timestamps.
    """

    model_geometries: dict[TetrisModel, ModelGeometry] = field(init=False)
    """
    Loaded metric geometry for the scenario models.
    """

    sequence_start_timestamp: int | None = field(default=None, init=False)
    """
    Unix capture time at the beginning of this observation sequence.
    """

    observation_index: int = field(default=0, init=False)
    """
    Position of the next scheduled observation in the sequence.
    """

    def __post_init__(self) -> None:
        """
        Prepare observations from a valid schedule and available CAD resources.
        """
        BaseAnnotator.__init__(self, name=ObservationLabel.SOURCE)
        timing = self.scenario.timing
        timing.validate()
        for block in self.scenario.blocks:
            block.validate_keyframes(timing)
            block.validate_absence_intervals(timing)
        self.model_geometries = {
            block.model: ModelGeometry.from_directory(block.model, self.mesh_directory)
            for block in self.scenario.blocks
        }

    def update(self) -> Status:
        capture = self.schedule_capture()
        if isinstance(capture, Status):
            return capture
        pipeline = self.get_parent_pipeline()
        pipeline.create_new_cas()
        ObservationCamera(self.scenario.camera).write_metadata(
            pipeline.cas, capture.capture_timestamp
        )
        self.write_hypotheses(pipeline.cas, capture)
        visualization = ObservationVisualization(pipeline.cas)
        output = self.get_annotator_output_struct()
        output.set_image(visualization.create_image())
        output.set_geometries(visualization.create_geometries())
        self.observation_index += 1
        return Status.SUCCESS

    def schedule_capture(self) -> ScheduledCapture | Status:
        """
        Select the next due capture or report waiting and exhaustion.
        """
        capture_offset = (
            self.observation_index * self.scenario.timing.sample_period_nanoseconds
        )
        if capture_offset > self.scenario.timing.duration_nanoseconds:
            return Status.FAILURE
        current_timestamp = self.clock.now_nanoseconds()
        if self.sequence_start_timestamp is None:
            self.sequence_start_timestamp = current_timestamp
        capture_timestamp = self.sequence_start_timestamp + capture_offset
        if current_timestamp < capture_timestamp:
            return Status.RUNNING
        return ScheduledCapture(capture_offset, capture_timestamp)

    def write_hypotheses(self, observation: CAS, capture: ScheduledCapture) -> None:
        """
        Append the visible models and their scheduled geometry to a CAS.
        """
        for block in self.scenario.blocks:
            if not block.is_visible(capture.elapsed_nanoseconds):
                continue
            object_pose = block.interpolate_pose(capture.elapsed_nanoseconds)
            observation.annotations.append(
                ObjectHypothesis(
                    source=self.name,
                    id=(
                        f"{self.name}:{self.observation_index}:"
                        f"{block.model.value.identifier}"
                    ),
                    roi=ForegroundProjection(
                        self.model_geometries[block.model].mesh, object_pose
                    ).create_region(observation),
                    annotations=[
                        Classification(
                            source=self.name,
                            classification_type=ObservationLabel.MODEL_CLASS,
                            classname=block.model.value.identifier,
                            confidence=1.0,
                        )
                    ]
                    + self.model_geometries[block.model].create_annotations(
                        object_pose, self.name
                    ),
                )
            )
