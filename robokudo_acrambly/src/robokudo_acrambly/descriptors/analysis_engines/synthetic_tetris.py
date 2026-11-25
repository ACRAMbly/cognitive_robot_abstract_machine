"""
Scripted observations of the tetris blocks.
"""

from __future__ import annotations

# %% Synthetic tetris engine

from dataclasses import dataclass, field

from robokudo_acrambly.synthetic_observations.analysis_engine import (
    SyntheticObservationEngine,
)
from robokudo_acrambly.synthetic_observations.parts import TetrisBlock
from robokudo_acrambly.synthetic_observations.scenario import PlaneLayout, Scenario


@dataclass(kw_only=True)
class AnalysisEngine(SyntheticObservationEngine):
    """
    Reads 30 s of passes showing one of each tetris block, swaying and turning in front
    of the camera.
    """

    scenario: Scenario = field(
        default_factory=lambda: Scenario(
            parts=PlaneLayout().arrange(labels=list(TetrisBlock))
        )
    )

    def name(self) -> str:
        return "synthetic_tetris"
