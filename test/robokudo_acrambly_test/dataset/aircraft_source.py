# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field

from robokudo.pipeline import Pipeline
from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
from robokudo_acrambly.annotators.aircraft_observation import (
    AircraftObservationAnnotator,
)
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)

# %% Observation source isolation


@dataclass(kw_only=True)
class AircraftObservationSequence(AnalysisEngine):
    """
    Exercise observation generation independently of persistent belief acceptance.
    """

    frames: AircraftFrames = field()
    """
    Explicit calibration for the isolated observation source.
    """

    def implementation(self) -> Pipeline:
        """
        Run only the source from :meth:`AnalysisEngine.implementation`.
        """
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
