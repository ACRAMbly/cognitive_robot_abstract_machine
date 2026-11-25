# %% Imports

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

from typing_extensions import TYPE_CHECKING

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.semantic_world_connector import SemanticDigitalTwinConnector
from robokudo.pipeline import Pipeline
from robokudo_acrambly.aircraft.models import AircraftModelCatalog
from robokudo_acrambly.aircraft_observations.configuration import (
    AircraftEnvironment,
    AircraftFrameConfiguration,
)
from robokudo_acrambly.aircraft_observations.exceptions import (
    MissingAircraftMeshConfiguration,
)
from robokudo_acrambly.aircraft_observations.scenario import AircraftScenario
from robokudo_acrambly.aircraft_observations.preview import AircraftPreview
from robokudo_acrambly.annotators.aircraft_observation import (
    AircraftObservationAnnotator,
    AircraftObservationLabel,
)
from robokudo_acrambly.playback.clock import SystemClock
from robokudo_acrambly.shared_world.world_source import WorldServerSource

if TYPE_CHECKING:
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
    from robokudo_acrambly.playback.clock import Clock
    from robokudo_acrambly.shared_world.world_source import WorldSource


# %% Public analysis engine


@dataclass
class AnalysisEngine(AnalysisEngineInterface):
    """
    Accept scripted aircraft observations into the robot's shared world.
    """

    frames: AircraftFrames | None = None
    """
    Declared camera frame and its calibration relative to the robot world's root.
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

    world_source: WorldSource = field(default_factory=WorldServerSource)
    """
    Supplier of the robot's world that the beliefs are accepted into.
    """

    def name(self) -> str:
        return AircraftObservationLabel.SOURCE

    def implementation(self) -> Pipeline:
        """
        Install the robot's world, then build the aircraft source and belief consumer.
        """
        frames = self.frames
        if frames is None:
            frames = AircraftFrameConfiguration.from_environment().create_frames()

        mesh_directory = os.environ.get(AircraftEnvironment.MESH_DIRECTORY)
        if not mesh_directory:
            raise MissingAircraftMeshConfiguration(AircraftEnvironment.MESH_DIRECTORY)
        preview = self.preview
        catalog = (
            AircraftModelCatalog(Path(mesh_directory))
            if preview is None
            else preview.catalog
        )
        if preview is None:
            preview = AircraftPreview(catalog)
        descriptor = SemanticDigitalTwinConnector.Descriptor()
        descriptor.parameters.semantic_types = {
            label: model.definition.semantic_type
            for label, model in catalog.models.items()
        }
        descriptor.parameters.model_geometry = catalog.models

        self.world_source.install()

        pipeline = Pipeline(self.name())
        pipeline.add_child(
            AircraftObservationAnnotator(
                scenario=self.scenario,
                frames=frames,
                clock=self.clock,
                preview=preview,
            )
        )
        pipeline.add_child(SemanticDigitalTwinConnector(descriptor=descriptor))
        return pipeline
