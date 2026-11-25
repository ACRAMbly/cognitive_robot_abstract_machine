# %% Imports

from __future__ import annotations

from collections.abc import Iterator
from uuid import uuid4

import pytest
from typing_extensions import TYPE_CHECKING

from krrood.adapters.json_serializer import from_json, to_json
from robokudo.cas import CAS
from robokudo.types.scene import ObjectHypothesis
from robokudo.world import get_object_belief_states, world_instance
from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.aircraft.types import AircraftPart
from robokudo_acrambly.playback.motion import VisibilityBoundary
from robokudo_acrambly.shared_world.accepted_capture import (
    AcceptedCapture,
    AcceptedCapturePublisher,
    AcceptedCaptureReceiver,
)
from semantic_digital_twin.adapters.ros.messages import MetaData, StreamPosition
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer

from .dataset.aircraft_beliefs import AircraftBeliefCapture
from .dataset.synchronization import DeliveryWait, has_buffered
from .test_aircraft_belief_creation import aircraft_catalog, semdt_cas
from .test_aircraft_world_synchronization import (
    ReceivingWorld,
    aircraft_engine,
    build_pipeline,
    world_server,
)
from .test_scripted_aircraft import aircraft_frames

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
        AnalysisEngine,
    )
    from acrambly_temp.stand_in_world_server import StandInWorldServer


# %% Message format


def test_accepted_capture_round_trips_through_json() -> None:
    """
    The message survives serialization with its time, parts and stream position.
    """
    capture = AcceptedCapture(
        capture_time=1_700_000_000_123_456_789,
        belief_ids=[uuid4(), uuid4()],
        stream_position=StreamPosition(
            origin=MetaData(node_name="robokudo", process_id=7), sequence_number=3
        ),
    )

    assert from_json(to_json(capture)) == capture
    assert AcceptedCapture.from_message(capture.to_message()) == capture


# %% Accepted beliefs of a capture


def test_parts_in_view_but_not_accepted_are_not_listed(semdt_cas: CAS) -> None:
    """
    Only beliefs whose latest hypothesis belongs to the capture count as accepted.
    """
    unaccepted = ObjectHypothesis()
    semdt_cas.annotations.append(unaccepted)

    assert AcceptedCapturePublisher.find_accepted_belief_ids(cas=semdt_cas) == []


# %% Receiving accepted captures


@pytest.fixture
def receiver(spinning_node: Node) -> Iterator[AcceptedCaptureReceiver]:
    """
    Receive accepted captures on the shared node, as the Coraplex demo will.
    """
    receiver = AcceptedCaptureReceiver(node=spinning_node)
    yield receiver
    receiver.close()


def test_receiver_records_the_capture_and_its_poses_count_only_once_applied(
    aircraft_engine: AnalysisEngine,
    world_server: StandInWorldServer,
    receiver: AcceptedCaptureReceiver,
    spinning_node: Node,
) -> None:
    """
    The receiver records time and parts, and held-back poses count once applied.
    """
    pipeline = build_pipeline(engine=aircraft_engine, server=world_server)
    receiving = ReceivingWorld.fetch(node=spinning_node, defer_incoming_updates=True)
    AircraftBeliefCapture(pipeline).complete()
    belief_ids = set(get_object_belief_states())
    position = WorldSynchronizer.of_world(world_instance()).latest_published_position

    assert DeliveryWait().until(lambda: receiver.part_captures.keys() == belief_ids)
    assert DeliveryWait().until(
        lambda: has_buffered(synchronizer=receiving.synchronizer, position=position)
    )
    assert len(belief_ids) == len(AircraftModel)
    for belief_id in belief_ids:
        part_capture = receiver.find_part_capture(belief_id=belief_id)
        assert part_capture.capture_time == pipeline.cas.data_timestamp
        assert part_capture.stream_position == position
        assert not part_capture.is_applied(synchronizer=receiving.synchronizer)

    receiving.synchronizer.apply_missed_messages()

    assert all(
        receiver.find_part_capture(belief_id=belief_id).is_applied(
            synchronizer=receiving.synchronizer
        )
        for belief_id in belief_ids
    )
    receiving.synchronizer.close()


def test_a_part_absent_from_a_capture_keeps_its_earlier_capture(
    aircraft_engine: AnalysisEngine,
    world_server: StandInWorldServer,
    receiver: AcceptedCaptureReceiver,
) -> None:
    """
    Parts hidden during partial absence keep the capture before it.
    """
    timing = aircraft_engine.scenario.timing
    timing.duration_nanoseconds = VisibilityBoundary.PARTIAL_ABSENCE_START
    timing.sample_period_nanoseconds = (
        VisibilityBoundary.TOTAL_ABSENCE_START
        - VisibilityBoundary.PARTIAL_ABSENCE_START
    )
    pipeline = build_pipeline(engine=aircraft_engine, server=world_server)
    clock = aircraft_engine.clock
    start = clock.timestamp_nanoseconds
    capture_times = []
    for offset in range(
        0, timing.duration_nanoseconds + 1, timing.sample_period_nanoseconds
    ):
        clock.timestamp_nanoseconds = start + offset
        AircraftBeliefCapture(pipeline).complete()
        capture_times.append(pipeline.cas.data_timestamp)
    final_capture_time, earlier_capture_time = capture_times[-1], capture_times[-2]
    models_by_belief = {
        annotation.root.id: next(
            model
            for model in AircraftModel
            if model.value.semantic_type is type(annotation)
        )
        for annotation in world_instance().get_semantic_annotations_by_type(
            AircraftPart
        )
    }
    absent_models = {
        part.model
        for part in aircraft_engine.scenario.parts
        if not part.is_visible(VisibilityBoundary.PARTIAL_ABSENCE_START)
    }

    assert DeliveryWait().until(
        lambda: {capture.capture_time for capture in receiver.part_captures.values()}
        == {final_capture_time, earlier_capture_time}
    )
    assert absent_models
    for belief_id, model in models_by_belief.items():
        expected = (
            earlier_capture_time if model in absent_models else final_capture_time
        )
        assert receiver.find_part_capture(belief_id=belief_id).capture_time == expected
