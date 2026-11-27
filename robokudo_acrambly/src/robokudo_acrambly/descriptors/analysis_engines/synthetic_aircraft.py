"""
Scripted observations of the parts of the toy aircraft.
"""

from __future__ import annotations

# %% Synthetic aircraft engine

from dataclasses import dataclass, field

from krrood.patterns.specificity_ranking import concrete_subclasses

from robokudo_acrambly.synthetic_observations.analysis_engine import (
    SyntheticObservationEngine,
)
from robokudo_acrambly.synthetic_observations.scenario import PlaneLayout, Scenario
from semantic_digital_twin_acrambly.semantic_annotations import AircraftPart


@dataclass(kw_only=True)
class AnalysisEngine(SyntheticObservationEngine):
    """
    Reads 30 s of passes showing the aircraft's parts unassembled, each swaying and
    turning in front of the camera.
    """

    scenario: Scenario = field(
        default_factory=lambda: Scenario(
            parts=PlaneLayout().arrange(
                labels=[
                    part_type.__name__
                    for part_type in concrete_subclasses(AircraftPart)
                ]
            )
        )
    )

    def name(self) -> str:
        return "synthetic_aircraft"
