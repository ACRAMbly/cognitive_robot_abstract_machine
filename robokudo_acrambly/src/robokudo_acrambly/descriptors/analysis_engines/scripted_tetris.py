# %% Imports

from __future__ import annotations

import os
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

from typing_extensions import TYPE_CHECKING

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo_acrambly.annotators.scripted_observation import (
    ObservationLabel,
    ScriptedObservationAnnotator,
)
from robokudo.pipeline import Pipeline
from robokudo_acrambly.scripted_observations.clock import SystemClock
from robokudo_acrambly.scripted_observations.scenario import TetrisScenario


if TYPE_CHECKING:
    from robokudo_acrambly.scripted_observations.clock import Clock


# %% Mesh resource configuration


class MeshEnvironment(StrEnum):
    """
    Environment configuration accepted by the ordinary analysis-engine launcher.
    """

    DIRECTORY = "TETRIS_MESH_DIRECTORY"


# %% Public analysis engine


@dataclass
class AnalysisEngine(AnalysisEngineInterface):
    """
    Expose scripted Tetris observations through a RoboKudo pipeline.
    """

    mesh_directory: Path | None = None
    """
    Directory containing the configured block CAD resources.
    """

    scenario: TetrisScenario = field(default_factory=TetrisScenario)
    """
    Editable camera-relative scene descriptor.
    """

    clock: Clock = field(default_factory=SystemClock)
    """
    Capture clock whose current time starts the observation sequence.
    """

    def name(self) -> str:
        return ObservationLabel.SOURCE

    def implementation(self) -> Pipeline:
        mesh_directory = self.mesh_directory
        if mesh_directory is None:
            mesh_directory = Path(os.environ.get(MeshEnvironment.DIRECTORY, Path.cwd()))
        pipeline = Pipeline(self.name())
        pipeline.add_child(
            ScriptedObservationAnnotator(
                scenario=self.scenario, mesh_directory=mesh_directory, clock=self.clock
            )
        )
        return pipeline
