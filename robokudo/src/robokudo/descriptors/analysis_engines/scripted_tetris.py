"""
Run deterministic observations of the two supplied Tetris blocks.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field

from typing_extensions import TYPE_CHECKING

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.scripted_observations import ScriptedObservationAnnotator
from robokudo.descriptors.scripted_tetris_observations import create_tetris_scenario
from robokudo.io.scripted_observations import PlaybackTiming, ScriptedIdentifier
from robokudo.pipeline import Pipeline

if TYPE_CHECKING:
    from robokudo.io.scripted_observations import ScriptedScenario

# %% analysis engine


@dataclass
class AnalysisEngine(AnalysisEngineInterface):
    """
    Produce scripted CAS hypotheses without updating a belief world.
    """

    repeat: bool = True
    """
    Whether the demonstration repeats after its last observation.
    """

    clock: Callable[[], float] = time.monotonic
    """
    Elapsed-time source for playback scheduling.
    """

    timing: PlaybackTiming = field(default_factory=PlaybackTiming)
    """
    Capture timeline and configurable playback interval.
    """

    scenario: ScriptedScenario = field(default_factory=create_tetris_scenario)
    """
    Ordered scripted observations to play.
    """

    def name(self) -> str:
        """
        Return the engine name used by the RoboKudo runner.
        """
        return ScriptedIdentifier.ENGINE

    def implementation(self) -> Pipeline:
        """
        Build a pipeline that stops at object hypotheses.
        """
        return Pipeline(
            ScriptedIdentifier.PIPELINE,
            children=[
                ScriptedObservationAnnotator(
                    scenario=self.scenario,
                    timing=self.timing,
                    clock=self.clock,
                    repeat=self.repeat,
                )
            ],
        )
