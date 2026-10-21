"""Accept scripted CAD observations and publish native world changes."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum

from py_trees.common import Status
from rclpy.node import Node
from typing_extensions import ClassVar

from robokudo import world as perception
from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.scripted_observations import ScriptedObservationAnnotator
from robokudo.annotators.semantic_world_connector import SemanticDigitalTwinConnector
from robokudo.descriptors.scripted_tetris_observations import create_tetris_scenario
from robokudo.io.scripted_observations import PlaybackTiming, ScriptedScenario
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import Classification
from robokudo.utils.hypothesis_comparators import ObjectHypothesisComparator
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body

# %% launch contract


class SynchronizedTetrisIdentifier(StrEnum):
    """Shared names for the synchronized demonstration."""

    ENGINE = "scripted_tetris_sync"
    """Synchronization analysis engine module."""
    PIPELINE = "SynchronizedTetrisPipeline"
    """Output map entry for accepted observations."""
    WORLD = "map"
    """Root frame displayed in RViz."""
    TOPIC = "/scripted_tetris/world_sync"
    """Dedicated native synchronization stream."""


@dataclass
class ReceiverUnavailableError(RuntimeError):
    """Initial changes cannot be delivered to a receiving subscription."""

    topic: str
    """Stream requiring a receiver before source startup."""

    def __str__(self) -> str:
        """Explain the required startup order."""
        return f"Start the scripted Tetris receiver before the source on {self.topic}."


@dataclass(eq=False, kw_only=True)
class SynchronizedObservationAnnotator(ScriptedObservationAnnotator):
    """Supply world coordinates on every newly emitted CAS."""

    world_T_camera: HomogeneousTransformationMatrix
    """Fixed camera pose in the receiving world's root frame."""

    def update(self) -> Status:
        """Emit an observation with its camera-to-world transform."""
        status = super().update()
        if status is Status.SUCCESS:
            observation = self.get_cas()
            observation.world_frame = SynchronizedTetrisIdentifier.WORLD
            observation.camera_to_world_transform = self.world_T_camera
        return status


# %% analysis engine ownership


@dataclass
class AnalysisEngine(AnalysisEngineInterface):
    """Own synchronization for a scripted perception pipeline."""

    node: Node
    """Borrowed application node; its owner spins and destroys it."""
    required_peer_count: ClassVar[int] = 2
    """Native endpoints belonging to the source and its required receiver."""
    synchronization_topic: str = SynchronizedTetrisIdentifier.TOPIC
    """Dedicated topic shared with the receiving world."""
    receiver_timeout: timedelta = timedelta(seconds=5)
    """Maximum wait for a matched receiving subscription."""
    clock: Callable[[], float] = time.monotonic
    """Elapsed playback time source."""
    timing: PlaybackTiming = field(default_factory=PlaybackTiming)
    """Capture timeline for scripted observations."""
    scenario: ScriptedScenario = field(default_factory=create_tetris_scenario)
    """Finite observation sequence to accept."""
    repeat: bool = False
    """Whether to restart the observation sequence."""
    world_T_camera: HomogeneousTransformationMatrix = field(
        default_factory=HomogeneousTransformationMatrix
    )
    """Fixed camera pose relative to the map frame."""
    synchronizer: WorldSynchronizer | None = field(default=None, init=False)
    """Application-owned source synchronizer."""
    pipeline: Pipeline | None = field(default=None, init=False)
    """Pipeline whose connector worker must finish before shutdown."""
    connector: SemanticDigitalTwinConnector | None = field(default=None, init=False)
    """Acceptance stage for scripted object hypotheses."""

    def name(self) -> str:
        """Return the engine's module name."""
        return SynchronizedTetrisIdentifier.ENGINE

    def implementation(self) -> Pipeline:
        """Attach synchronization before creating any world structure."""
        if self.pipeline is not None:
            return self.pipeline
        world = perception.world_instance()
        self.synchronizer = WorldSynchronizer(
            _world=world, node=self.node, topic_name=self.synchronization_topic
        )
        deadline = time.monotonic() + self.receiver_timeout.total_seconds()
        while (
            self.synchronizer.publisher.get_subscription_count()
            < self.required_peer_count
            or self.node.count_publishers(self.synchronization_topic)
            < self.required_peer_count
        ):
            if time.monotonic() >= deadline:
                self.shutdown()
                raise ReceiverUnavailableError(self.synchronization_topic)
            time.sleep(self.synchronizer.discovery_poll_interval.total_seconds())
        source = SynchronizedObservationAnnotator(
            scenario=self.scenario,
            timing=self.timing,
            clock=self.clock,
            repeat=self.repeat,
            world_T_camera=self.world_T_camera,
        )
        root = Body(name=PrefixedName(SynchronizedTetrisIdentifier.WORLD))
        camera = Body(name=PrefixedName(source.camera.frame))
        with world.modify_world():
            world.add_connection(
                FixedConnection(
                    parent=root,
                    child=camera,
                    parent_T_connection_expression=self.world_T_camera,
                )
            )
        self.connector = SemanticDigitalTwinConnector()
        self.connector.object_comparator = ObjectHypothesisComparator(
            self.connector.get_cas
        ).with_comparator_for(Classification, weight=1.0)
        self.pipeline = Pipeline(
            SynchronizedTetrisIdentifier.PIPELINE, children=[source, self.connector]
        )
        return self.pipeline

    def shutdown(self) -> None:
        """Finish accepted updates before releasing synchronization resources."""
        if self.connector is not None:
            worker = self.connector.compute_worker_thread
            if worker is not None:
                worker.join()
        if self.synchronizer is not None:
            self.synchronizer.close()
            self.synchronizer = None
