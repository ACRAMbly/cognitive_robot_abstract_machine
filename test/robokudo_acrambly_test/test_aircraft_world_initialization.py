# %% Imports

from __future__ import annotations

import pytest
from typing_extensions import TYPE_CHECKING

from robokudo import world as robokudo_world
from robokudo_acrambly.aircraft_observations.configuration import AircraftEnvironment
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)

from .test_scripted_aircraft import aircraft_frames
from .test_aircraft_belief_creation import aircraft_catalog

if TYPE_CHECKING:
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog
    from semantic_digital_twin.world import World


# %% World frame registration


def test_pipeline_registers_world_frame_once(
    aircraft_frames: AircraftFrames,
    aircraft_catalog: AircraftModelCatalog,
    mini_world: World,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Pipeline construction registers one frame and retains it on reconstruction.
    """
    monkeypatch.setattr(robokudo_world, "world", mini_world)
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY, str(aircraft_catalog.mesh_directory)
    )
    initial_bodies = list(mini_world.bodies)
    engine = AnalysisEngine(frames=aircraft_frames)

    engine.implementation()

    (world_body,) = mini_world.get_bodies_by_name(aircraft_frames.world_frame)
    assert list(mini_world.bodies) == initial_bodies + [world_body]

    engine.implementation()

    assert mini_world.get_bodies_by_name(aircraft_frames.world_frame) == [world_body]
    assert list(mini_world.bodies) == initial_bodies + [world_body]
