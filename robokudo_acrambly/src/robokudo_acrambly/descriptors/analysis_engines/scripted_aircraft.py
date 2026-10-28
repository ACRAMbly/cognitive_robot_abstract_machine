# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import TYPE_CHECKING

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.pipeline import Pipeline
from robokudo_acrambly.aircraft_observations.scenario import AircraftScenario
from robokudo_acrambly.annotators.aircraft_observation import (
    AircraftObservationAnnotator,
    AircraftObservationLabel,
)
from robokudo_acrambly.playback.clock import SystemClock

if TYPE_CHECKING:
    from robokudo_acrambly.aircraft_observations.preview import AircraftPreview
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
    from robokudo_acrambly.playback.clock import Clock


# %% Public analysis engine


@dataclass
class AnalysisEngine(AnalysisEngineInterface):
    """
    Expose minimal aircraft observations through an independent pipeline.
    """

    frames: AircraftFrames
    """Application-supplied frame identifiers and explicit calibration."""

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
        pipeline = Pipeline(self.name())
        pipeline.add_child(
            AircraftObservationAnnotator(
                scenario=self.scenario,
                frames=self.frames,
                clock=self.clock,
                preview=self.preview,
            )
        )
        return pipeline
