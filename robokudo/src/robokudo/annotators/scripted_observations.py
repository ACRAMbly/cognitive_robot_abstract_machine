"""
Write scripted perception results to a fresh CAS.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.core import BaseAnnotator
from robokudo.io.scripted_observations import (
    ModelGeometry,
    PlaybackTiming,
    ScriptedCamera,
    ScriptedIdentifier,
    ScriptedResource,
    TetrisBlock,
)
from robokudo.vis.scripted_observations import ObservationPreview

if TYPE_CHECKING:
    from robokudo.io.scripted_observations import ScriptedScenario

# %% observation source


@dataclass(eq=False, kw_only=True)
class ScriptedObservationAnnotator(BaseAnnotator):
    """
    Emit known object detections without camera capture or inference.
    """

    scenario: ScriptedScenario
    """
    Finite sequence of detections to emit.
    """

    camera: ScriptedCamera = field(default_factory=ScriptedCamera)
    """
    Reference camera metadata for generated observations.
    """

    timing: PlaybackTiming = field(default_factory=PlaybackTiming)
    """
    Capture timeline for the sequence.
    """

    repeat: bool = True
    """
    Whether to repeat the scenario while advancing its capture timeline.
    """

    clock: Callable[[], float] = time.monotonic
    """
    Elapsed-time source used to schedule playback.
    """

    mesh_directory: Path = field(
        default_factory=ScriptedResource.MESH_DIRECTORY.resolve
    )
    """
    Directory containing the original CAD meshes.
    """

    name: str = ScriptedIdentifier.ANNOTATOR
    """
    Name recorded as the source of generated annotations.
    """

    observation_index: int = field(init=False, default=0)
    """
    Index of the next observation to publish.
    """

    geometries: dict[TetrisBlock, ModelGeometry] = field(
        init=False, default_factory=dict
    )
    """
    Metric geometry loaded once for each target model.
    """

    next_observation_time: float = field(init=False)
    """
    Earliest elapsed time at which the next observation may be emitted.
    """

    preview: ObservationPreview = field(init=False)
    """
    Diagnostic image renderer for the current camera.
    """

    def __post_init__(self) -> None:
        """
        Initialize the perception stage.
        """
        super().__init__(self.name)
        self.next_observation_time = self.clock()
        self.preview = ObservationPreview(self.camera)
        self.geometries = {
            block: ModelGeometry.load(block, self.mesh_directory)
            for block in TetrisBlock
        }

    def update(self) -> Status:
        """
        Publish the visible target models in a fresh observation.
        """
        observation_count = len(self.scenario.observations)
        if not self.repeat and self.observation_index >= observation_count:
            return Status.FAILURE
        current_time = self.clock()
        if current_time < self.next_observation_time:
            return Status.RUNNING
        observation = self.scenario.observations[
            self.observation_index % observation_count
        ]
        hypotheses = [
            detection.create_hypothesis(
                self.geometries[detection.block], self.camera, self.name
            )
            for detection in observation.detections
        ]
        self.get_parent_pipeline().create_new_cas()
        current_observation = self.get_cas()
        current_observation.camera_frame = self.camera.frame
        current_observation.data_timestamp = self.timing.capture_time(
            self.observation_index
        )
        current_observation.camera_info = self.camera.create_info(
            current_observation.data_timestamp
        )
        current_observation.annotations.extend(hypotheses)
        image = self.preview.draw(observation, hypotheses)
        current_observation.color_image = image
        self.get_annotator_output_struct().set_image(image)
        self.observation_index += 1
        self.next_observation_time = (
            current_time + self.timing.frame_interval.total_seconds()
        )
        return Status.SUCCESS
