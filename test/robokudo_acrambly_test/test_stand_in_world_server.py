# %% Imports

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from threading import Thread

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from typing_extensions import TYPE_CHECKING

from acrambly_temp.stand_in_world_server import StandInWorldName, StandInWorldServer
from robokudo.world import world_instance
from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.aircraft.types import AircraftPart
from robokudo_acrambly.aircraft_observations.configuration import AircraftEnvironment
from robokudo_acrambly.aircraft_observations.inspection import AircraftBeliefInspection
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)
from robokudo_acrambly.shared_world.exceptions import (
    WorldServerScript,
    WorldServerUnavailable,
)
from robokudo_acrambly.shared_world.world_source import WorldServerSource
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer

from .conftest import ExecutorShutdown
from .dataset.aircraft_beliefs import AircraftBeliefCapture
from .dataset.synchronization import DeliveryWait
from .test_aircraft_belief_creation import aircraft_catalog, semdt_cas
from .test_aircraft_world_synchronization import build_pipeline, world_server
from .test_scripted_aircraft import aircraft_frames
from .test_scripted_tetris import ControlledClock

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames


# %% Served world


def test_a_world_fetched_from_the_stand_in_has_root_map(
    world_server: StandInWorldServer, spinning_node: Node
) -> None:
    """
    The stand-in serves a world whose only body is its root ``map``.
    """
    world = fetch_world_from_service(node=spinning_node)

    assert str(world.root.name) == StandInWorldName.ROOT
    assert len(world.bodies) == 1


@pytest.mark.served_world
def test_a_world_fetched_after_a_capture_holds_the_accepted_parts(
    world_server: StandInWorldServer,
    spinning_node: Node,
    aircraft_frames: AircraftFrames,
    aircraft_catalog: AircraftModelCatalog,
    semdt_cas: CAS,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    The engine fetches the stand-in's world, and a later fetch includes its parts.
    """
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY, str(aircraft_catalog.mesh_directory)
    )
    engine = AnalysisEngine(frames=aircraft_frames, clock=ControlledClock())
    engine.scenario.timing.duration_nanoseconds = 0
    pipeline = build_pipeline(engine=engine, server=world_server)
    AircraftBeliefCapture(pipeline).complete()
    robokudo_synchronizer = WorldSynchronizer.of_world(world_instance())
    position = robokudo_synchronizer.latest_published_position

    assert DeliveryWait().until(lambda: world_server.synchronizer.has_applied(position))
    fetched = fetch_world_from_service(node=spinning_node)
    robokudo_synchronizer.close()

    assert {
        part.root.id for part in fetched.get_semantic_annotations_by_type(AircraftPart)
    } == {
        part.root.id
        for part in world_instance().get_semantic_annotations_by_type(AircraftPart)
    }
    assert len(fetched.get_semantic_annotations_by_type(AircraftPart)) == len(
        AircraftModel
    )


@pytest.fixture
def stand_in_on_its_own_node(ros_default: None) -> Iterator[StandInWorldServer]:
    """
    Serve the stand-in world from a node of its own, as a separate process would.
    """
    stand_in_node = rclpy.create_node(StandInWorldName.NODE)
    executor = SingleThreadedExecutor()
    executor.add_node(stand_in_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    stand_in = StandInWorldServer(node=stand_in_node)
    yield stand_in
    stand_in.close()
    executor.shutdown()
    thread.join(timeout=ExecutorShutdown().timeout_seconds)
    stand_in_node.destroy_node()


@pytest.mark.served_world
def test_the_world_is_fetched_over_a_node_nobody_spins_yet(
    stand_in_on_its_own_node: StandInWorldServer, node: Node
) -> None:
    """
    Fetching works before RoboKudo spins its node, and leaves the node unowned.
    """
    world = WorldServerSource().supply_world()

    assert str(world.root.name) == StandInWorldName.ROOT
    assert node.executor is None


# %% Inspection without a world server


@pytest.mark.served_world
def test_inspection_without_a_world_server_stops_after_its_timeout(
    node: Node,
    aircraft_frames: AircraftFrames,
    aircraft_catalog: AircraftModelCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Inspection never waits forever, and the error names the stand-in script.
    """
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY, str(aircraft_catalog.mesh_directory)
    )
    engine = AnalysisEngine(
        frames=aircraft_frames,
        world_source=WorldServerSource(timeout_seconds=0.2),
    )

    with pytest.raises(WorldServerUnavailable) as error:
        with AircraftBeliefInspection(node=node, engine=engine):
            pass

    assert WorldServerScript.STAND_IN in error.value.suggest_correction()
    assert (Path(__file__).resolve().parents[2] / WorldServerScript.STAND_IN).is_file()
