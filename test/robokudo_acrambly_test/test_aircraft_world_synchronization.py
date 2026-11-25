# %% Imports

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

import numpy
import pytest
from typing_extensions import TYPE_CHECKING

from robokudo.world import world_instance
from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.aircraft.types import AircraftPart
from robokudo_acrambly.aircraft_observations.configuration import AircraftEnvironment
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)
from robokudo_acrambly.shared_world.world_source import ProvidedWorldSource
from semantic_digital_twin.adapters.ros.world_fetcher import (
    FetchWorldServer,
    fetch_world_from_service,
)
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer

from .dataset.aircraft_beliefs import AircraftBeliefCapture
from .dataset.robot_world import RobotWorldFrame, create_robot_world
from .dataset.synchronization import DeliveryWait, has_buffered
from .test_aircraft_belief_creation import aircraft_catalog, semdt_cas
from .test_scripted_aircraft import aircraft_frames
from .test_scripted_tetris import ControlledClock

if TYPE_CHECKING:
    from uuid import UUID

    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo.cas import CAS
    from robokudo.pipeline import Pipeline
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
    from semantic_digital_twin.world import World


# %% Robot world served to two processes


@pytest.fixture
def world_server(spinning_node: Node) -> Iterator[FetchWorldServer]:
    """
    Serve a synchronized robot world with root ``map``, as Giskard does.
    """
    world = create_robot_world(root_frame=RobotWorldFrame.MAP)
    synchronizer = WorldSynchronizer(_world=world, node=spinning_node)
    server = FetchWorldServer(node=spinning_node, world=world)
    yield server
    server.close()
    synchronizer.close()


@pytest.fixture
def aircraft_engine(
    world_server: FetchWorldServer,
    spinning_node: Node,
    aircraft_catalog: AircraftModelCatalog,
    aircraft_frames: AircraftFrames,
    semdt_cas: CAS,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[AnalysisEngine]:
    """
    An aircraft engine with one capture, set to run on a world fetched from the server.
    """
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY, str(aircraft_catalog.mesh_directory)
    )
    engine = AnalysisEngine(
        frames=aircraft_frames,
        clock=ControlledClock(),
        world_source=ProvidedWorldSource(
            world=fetch_world_from_service(node=spinning_node)
        ),
    )
    engine.scenario.timing.duration_nanoseconds = 0
    yield engine
    for synchronizer in WorldSynchronizer.all_callbacks_of_this_type_from_world(
        world_instance()
    ):
        synchronizer.close()


def build_pipeline(*, engine: AnalysisEngine, server: FetchWorldServer) -> Pipeline:
    """
    Install the engine's world, set up its pipeline, and wait until the server applied
    what the installation published.
    """
    pipeline = engine.implementation()
    pipeline.setup()
    position = WorldSynchronizer.of_world(world_instance()).latest_published_position
    assert DeliveryWait().until(
        lambda: WorldSynchronizer.of_world(server.world).has_applied(position)
    )
    return pipeline


@dataclass(kw_only=True)
class ReceivingWorld:
    """
    A copy of the robot's world in another process, kept in step by synchronization.
    """

    world: World
    """
    The fetched copy.
    """

    synchronizer: WorldSynchronizer
    """
    Synchronizer applying, or holding back, what RoboKudo publishes.
    """

    @classmethod
    def fetch(cls, *, node: Node, defer_incoming_updates: bool) -> ReceivingWorld:
        """
        Fetch the served robot world and synchronize it, as Coraplex does.
        """
        world = fetch_world_from_service(node=node)
        return cls(
            world=world,
            synchronizer=WorldSynchronizer(
                _world=world, node=node, defer_incoming_updates=defer_incoming_updates
            ),
        )


def accepted_parts(world: World) -> dict[UUID, AircraftPart]:
    """
    Index the aircraft parts of a world by the UUID of their body.
    """
    return {
        annotation.root.id: annotation
        for annotation in world.get_semantic_annotations_by_type(AircraftPart)
    }


# %% Synchronized beliefs


def test_receiving_world_gets_accepted_parts_with_uuids_types_and_poses(
    aircraft_engine: AnalysisEngine, spinning_node: Node, world_server: FetchWorldServer
) -> None:
    """
    Every part accepted into RoboKudo's world reaches a synchronized world.
    """
    pipeline = build_pipeline(engine=aircraft_engine, server=world_server)
    receiving = ReceivingWorld.fetch(node=spinning_node, defer_incoming_updates=False)
    AircraftBeliefCapture(pipeline).complete()
    position = WorldSynchronizer.of_world(world_instance()).latest_published_position

    assert DeliveryWait().until(lambda: receiving.synchronizer.has_applied(position))
    robokudo_parts = accepted_parts(world_instance())
    received_parts = accepted_parts(receiving.world)
    assert len(robokudo_parts) == len(AircraftModel)
    assert received_parts.keys() == robokudo_parts.keys()
    for identifier, part in robokudo_parts.items():
        assert type(received_parts[identifier]) is type(part)
        numpy.testing.assert_allclose(
            received_parts[identifier].root.global_transform.to_np(),
            part.root.global_transform.to_np(),
        )
    receiving.synchronizer.close()


def test_held_back_updates_reach_the_receiving_world_only_when_applied(
    aircraft_engine: AnalysisEngine, spinning_node: Node, world_server: FetchWorldServer
) -> None:
    """
    With deferred updates the receiving world stays unchanged until it applies them.
    """
    pipeline = build_pipeline(engine=aircraft_engine, server=world_server)
    receiving = ReceivingWorld.fetch(node=spinning_node, defer_incoming_updates=True)
    bodies_before = list(receiving.world.bodies)
    AircraftBeliefCapture(pipeline).complete()
    position = WorldSynchronizer.of_world(world_instance()).latest_published_position

    assert DeliveryWait().until(
        lambda: has_buffered(synchronizer=receiving.synchronizer, position=position)
    )
    assert list(receiving.world.bodies) == bodies_before
    assert not receiving.synchronizer.has_applied(position)

    receiving.synchronizer.apply_missed_messages()

    assert receiving.synchronizer.has_applied(position)
    assert (
        accepted_parts(receiving.world).keys()
        == accepted_parts(world_instance()).keys()
    )
    receiving.synchronizer.close()
