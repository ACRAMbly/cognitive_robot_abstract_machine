# %% Imports

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

from typing_extensions import TYPE_CHECKING

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.pipeline import Pipeline
from robokudo_acrambly.aircraft.models import AircraftModelCatalog
from robokudo_acrambly.aircraft_observations.configuration import (
    AircraftEnvironment,
    AircraftFrameConfiguration,
)
from robokudo_acrambly.aircraft_observations.scenario import AircraftScenario
from robokudo_acrambly.aircraft_observations.preview import AircraftPreview
from robokudo_acrambly.annotators.aircraft_observation import (
    AircraftObservationAnnotator,
    AircraftObservationLabel,
)
from robokudo_acrambly.playback.clock import SystemClock

if TYPE_CHECKING:
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
    from robokudo_acrambly.playback.clock import Clock


# %% Public analysis engine


@dataclass
class AnalysisEngine(AnalysisEngineInterface):
    """
    Expose minimal aircraft observations through an independent pipeline.
    """

    frames: AircraftFrames | None = None
    """
    Declared frame identifiers and explicit camera-to-world calibration.
    """

    scenario: AircraftScenario = field(default_factory=AircraftScenario)
    """
    Editable camera-relative aircraft scene.
    """

    clock: Clock = field(default_factory=SystemClock)
    """
    Clock supplying scheduled capture time independently of CAS creation.
    """

    preview: AircraftPreview | None = None
    """
    Optional catalog geometry displayed independently of the CAS payload.
    """

    def name(self) -> str:
        return AircraftObservationLabel.SOURCE

    def implementation(self) -> Pipeline:
        frames = self.frames
        if frames is None:
            frames = AircraftFrameConfiguration.from_environment().create_frames()
        preview = self.preview
        mesh_directory = os.environ.get(AircraftEnvironment.MESH_DIRECTORY)
        if preview is None and mesh_directory is not None:
            preview = AircraftPreview(AircraftModelCatalog(Path(mesh_directory)))
        pipeline = Pipeline(self.name())
        pipeline.add_child(
            AircraftObservationAnnotator(
                scenario=self.scenario,
                frames=frames,
                clock=self.clock,
                preview=preview,
            )
        )
        return pipeline
