"""
Analysis engines that run on scripted observations.
"""

from __future__ import annotations

# %% Imports

from dataclasses import dataclass

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline
from typing_extensions import TYPE_CHECKING

from robokudo_acrambly.synthetic_observations.camera_interface import (
    SyntheticCameraConfig,
    SyntheticObservationInterface,
)

if TYPE_CHECKING:
    from robokudo_acrambly.synthetic_observations.scenario import Scenario


# %% Synthetic observation engine


@dataclass(kw_only=True)
class SyntheticObservationEngine(AnalysisEngineInterface):
    """
    An analysis engine whose pipeline reads the passes of a scenario into its CAS and
    does nothing else with them.
    """

    scenario: Scenario
    """
    The scenario the pipeline reads.
    """

    def implementation(self) -> Pipeline:
        camera_config = SyntheticCameraConfig(scenario=self.scenario)
        pipeline = Pipeline(self.name())
        pipeline.add_children(
            [
                pipeline_init(),
                CollectionReaderAnnotator(
                    descriptor=CollectionReaderAnnotator.Descriptor(
                        camera_config=camera_config,
                        camera_interface=SyntheticObservationInterface(
                            camera_config=camera_config
                        ),
                    )
                ),
            ]
        )
        return pipeline
