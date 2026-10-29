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
from robokudo.world import world_instance
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.connections import FixedConnection

if TYPE_CHECKING:
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
    from robokudo_acrambly.playback.clock import Clock


# %% Public analysis engine


@dataclass
class AnalysisEngine(AnalysisEngineInterface):
    """
    Accept scripted aircraft observations into RoboKudo's local belief world.
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
        """
        Construct the aircraft source and configured local belief consumer.
        """
        frames = self.frames
        if frames is None:
            frames = AircraftFrameConfiguration.from_environment().create_frames()
        frames.validate()

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

        runtime_world = world_instance()
        if not runtime_world.get_bodies_by_name(frames.world_frame):
            root = runtime_world.root
            world_body = Body(name=PrefixedName(frames.world_frame))
            with runtime_world.modify_world():
                if root is None:
                    runtime_world.add_body(world_body)
                else:
                    runtime_world.add_connection(
                        FixedConnection(parent=root, child=world_body)
                    )

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
