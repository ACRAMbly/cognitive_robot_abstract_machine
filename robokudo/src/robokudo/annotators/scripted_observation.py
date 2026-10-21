# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.core import BaseAnnotator
from robokudo.scripted_observations.camera import ObservationCamera
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

    def __post_init__(self) -> None:
        """
        Initialize the annotator's behavior-tree identity.
        """
        BaseAnnotator.__init__(self, name=ObservationLabel.SOURCE)
        self.models = {
            block.model: ModelGeometry.from_directory(block.model, self.mesh_directory)
            for block in self.scenario.blocks
        }

    def update(self) -> Status:
        pipeline = self.get_parent_pipeline()
        pipeline.create_new_cas()
        ObservationCamera(self.scenario.camera).write_metadata(
            pipeline.cas, self.clock.now_nanoseconds()
        )
        for block in self.scenario.blocks:
            pipeline.cas.annotations.append(
                ObjectHypothesis(
                    source=self.name,
                    id=f"{self.name}:0:{block.model.value.identifier}",
                    roi=ForegroundProjection(
                        self.models[block.model].mesh, block.pose
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
                        block.pose, self.name
                    ),
                )
            )
        return Status.SUCCESS
