# %% Imports

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack
from enum import IntEnum
from time import monotonic

import pytest
import rclpy
from py_trees.common import Status
from typing_extensions import TYPE_CHECKING
from visualization_msgs.msg import MarkerArray

from robokudo import world
from robokudo_acrambly.aircraft_observations.configuration import AircraftEnvironment
from robokudo_acrambly.aircraft_observations.inspection import AircraftBeliefInspection
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)

from .dataset.aircraft_beliefs import AircraftBeliefCapture
from .test_aircraft_belief_creation import aircraft_catalog, semdt_cas
from .test_scripted_aircraft import aircraft_frames
from .test_scripted_tetris import ControlledClock

if TYPE_CHECKING:
    from rclpy.node import Node
    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames


# %% Inspection lifecycle


class InspectionLimit(IntEnum):
    """Bounds for local inspection message delivery."""

    DELIVERY_SECONDS = 5
    """Maximum wait for the final latched geometry message."""


@pytest.fixture
def inspection(
    aircraft_catalog: AircraftModelCatalog,
    aircraft_frames: AircraftFrames,
    semdt_cas: CAS,
    monkeypatch: pytest.MonkeyPatch,
    node: Node,
) -> Iterator[AircraftBeliefInspection]:
    """Inspect a single completed aircraft capture in an isolated belief world."""
    world.clear_world()
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY, str(aircraft_catalog.mesh_directory)
    )
    engine = AnalysisEngine(frames=aircraft_frames, clock=ControlledClock())
    engine.scenario.timing.duration_nanoseconds = 0
    with AircraftBeliefInspection(node=node, engine=engine) as inspection:
        yield inspection


def test_inspection_publishes_geometry_from_the_local_belief_world(
    inspection: AircraftBeliefInspection,
) -> None:
    """Every accepted belief supplies its geometry to the inspection markers."""
    AircraftBeliefCapture(inspection.pipeline).complete()

    assert {marker.ns for marker in inspection.markers.markers.markers} == {
        str(belief.body.name) for belief in world.get_object_belief_states().values()
    }
    assert len(inspection.markers.markers.markers) == len(
        inspection.engine.scenario.parts
    )


def test_finished_inspection_retains_beliefs_and_refreshes_transforms(
    inspection: AircraftBeliefInspection,
) -> None:
    """Idle inspection retains the accepted capture and publishes fresh TF stamps."""
    AircraftBeliefCapture(inspection.pipeline).complete()
    final_capture = inspection.pipeline.cas
    final_markers = inspection.markers.markers
    inspection.tick()
    previous_stamp = (
        inspection.markers.tf_publisher.tf_model_callback.tf_message.transforms[
            0
        ].header.stamp
    )
    inspection.tick()

    assert inspection.pipeline.status is Status.FAILURE
    assert inspection.pipeline.cas is final_capture
    assert inspection.markers.markers is final_markers
    assert inspection.node.context.ok()
    current_stamp = (
        inspection.markers.tf_publisher.tf_model_callback.tf_message.transforms[
            0
        ].header.stamp
    )
    assert (current_stamp.sec, current_stamp.nanosec) > (
        previous_stamp.sec,
        previous_stamp.nanosec,
    )


def test_late_subscriber_receives_final_belief_geometry(
    inspection: AircraftBeliefInspection,
) -> None:
    """An RViz subscriber opened after playback receives the completed belief world."""
    deadline = monotonic() + InspectionLimit.DELIVERY_SECONDS
    while inspection.pipeline.status is not Status.FAILURE and monotonic() < deadline:
        rclpy.spin_once(inspection.node, timeout_sec=inspection.tick_period_seconds)
    assert inspection.pipeline.status is Status.FAILURE

    received: list[MarkerArray] = []
    with ExitStack() as resources:
        subscription = inspection.node.create_subscription(
            MarkerArray,
            inspection.markers.topic_name,
            received.append,
            inspection.markers.qos_profile,
        )
        resources.callback(inspection.node.destroy_subscription, subscription)
        deadline = monotonic() + InspectionLimit.DELIVERY_SECONDS
        expected = inspection.markers.markers
        while (not received or received[-1] != expected) and monotonic() < deadline:
            rclpy.spin_once(inspection.node, timeout_sec=inspection.tick_period_seconds)

    assert received[-1] == expected
