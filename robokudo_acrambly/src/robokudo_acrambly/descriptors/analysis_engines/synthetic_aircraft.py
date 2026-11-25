"""
Scripted observations of the parts of the toy aircraft.
"""

from __future__ import annotations

# %% Synthetic aircraft engine

from dataclasses import dataclass, field

from robokudo_acrambly.synthetic_observations.analysis_engine import (
    SyntheticObservationEngine,
)
from robokudo_acrambly.synthetic_observations.parts import AircraftPart
from robokudo_acrambly.synthetic_observations.scenario import PlaneLayout, Scenario


@dataclass(kw_only=True)
class AnalysisEngine(SyntheticObservationEngine):
    """
    Reads 30 s of passes showing the aircraft's parts unassembled, each swaying and
    turning in front of the camera.
    """

    scenario: Scenario = field(
        default_factory=lambda: Scenario(
            parts=PlaneLayout().arrange(labels=list(AircraftPart))
        )
    )

    def name(self) -> str:
        return "synthetic_aircraft"
