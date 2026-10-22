# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.core import BaseAnnotator
from robokudo.scripted_observations.camera import ObservationCamera
from robokudo.scripted_observations.exceptions import InvalidPlaybackTiming
from robokudo.scripted_observations.geometry import ModelGeometry
from robokudo.scripted_observations.projection import ForegroundProjection
from robokudo.types.annotation import Classification
from robokudo.types.scene import ObjectHypothesis


if TYPE_CHECKING:
    from pathlib import Path

    from robokudo.scripted_observations.clock import Clock
    from robokudo.scripted_observations.models import TetrisModel
    from robokudo.scripted_observations.scenario import TetrisScenario


# %% Observation annotations


class ObservationLabel(StrEnum):
    """
    Labels identifying the synthetic observation source and classification kind.
    """

    SOURCE = "scripted_tetris"
    MODEL_CLASS = "CLASS"


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

    models: dict[TetrisModel, ModelGeometry] = field(init=False)
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
        if (
            timing.sample_period_nanoseconds <= 0
            or timing.duration_nanoseconds < 0
            or timing.duration_nanoseconds % timing.sample_period_nanoseconds != 0
        ):
            raise InvalidPlaybackTiming(
                duration_nanoseconds=timing.duration_nanoseconds,
                sample_period_nanoseconds=timing.sample_period_nanoseconds,
            )
        for block in self.scenario.blocks:
            block.validate_keyframes(timing)
            block.validate_absence_intervals(timing)
        self.models = {
            block.model: ModelGeometry.from_directory(block.model, self.mesh_directory)
            for block in self.scenario.blocks
        }

    def update(self) -> Status:
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
        pipeline = self.get_parent_pipeline()
        pipeline.create_new_cas()
        ObservationCamera(self.scenario.camera).write_metadata(
            pipeline.cas, capture_timestamp
        )
        for block in self.scenario.blocks:
            if not block.is_visible(capture_offset):
                continue
            object_pose = block.interpolate_pose(capture_offset)
            pipeline.cas.annotations.append(
                ObjectHypothesis(
                    source=self.name,
                    id=(
                        f"{self.name}:{self.observation_index}:"
                        f"{block.model.value.identifier}"
                    ),
                    roi=ForegroundProjection(
                        self.models[block.model].mesh, object_pose
                    ).create_region(pipeline.cas),
                    annotations=[
                        Classification(
                            source=self.name,
                            classification_type=ObservationLabel.MODEL_CLASS,
                            classname=block.model.value.identifier,
                            confidence=1.0,
                        )
                    ]
                    + self.models[block.model].create_annotations(
                        object_pose, self.name
                    ),
                )
            )
        self.observation_index += 1
        return Status.SUCCESS
