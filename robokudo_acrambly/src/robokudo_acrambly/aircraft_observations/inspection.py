# %% Imports

from __future__ import annotations

from contextlib import AbstractContextManager, ExitStack, suppress
from dataclasses import dataclass, field
from enum import StrEnum

import rclpy
from py_trees.blackboard import Blackboard
from py_trees.common import Status
from py_trees.trees import BehaviourTree
from typing_extensions import TYPE_CHECKING, Self

from robokudo.annotators.core import ThreadedAnnotator
from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
from robokudo.world import world_instance
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)
from semantic_digital_twin.adapters.ros.node_registry import ROSNodeRegistry
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)

if TYPE_CHECKING:
    from types import TracebackType

    from rclpy.node import Node
    from rclpy.timer import Timer

    from robokudo.pipeline import Pipeline


# %% Belief world inspection


class InspectionNode(StrEnum):
    """ROS node names for aircraft belief inspection."""

    AIRCRAFT = "aircraft_belief_inspection"
    """Node hosting the aircraft pipeline and world publishers."""


class InspectionBlackboard(StrEnum):
    """Blackboard entries required by the aircraft pipeline."""

    OUTPUT_BUFFER = "annotator_output_pipeline_map_buffer"
    """Per-annotator output shared with the observation source."""


@dataclass
class AircraftBeliefInspection(AbstractContextManager):
    """Publish live aircraft beliefs and preserve the final world until closed."""

    node: Node
    """Application-owned ROS node serving the inspection session."""

    engine: AnalysisEngine = field(default_factory=AnalysisEngine)
    """Configured source and consumer of aircraft observations."""

    tick_period_seconds: float = 0.05
    """Interval between pipeline ticks and idle transform publications."""

    pipeline: Pipeline = field(init=False)
    """Pipeline accepting observations into the local belief world."""

    markers: VizMarkerPublisher = field(init=False)
    """Geometry publisher attached to the local belief world."""

    timer: Timer = field(init=False)
    """Timer driving playback and continued final-state inspection."""

    resources: ExitStack = field(init=False, repr=False)
    """Resources released before the application destroys its ROS node."""

    def __enter__(self) -> Self:
        """Initialize playback and world publication on the supplied node."""
        with ExitStack() as resources:
            Blackboard().set(
                InspectionBlackboard.OUTPUT_BUFFER, AnnotatorOutputPerPipelineMap()
            )
            self.pipeline = self.engine.implementation()
            tree = BehaviourTree(self.pipeline)
            resources.callback(tree.shutdown)
            tree.setup(node=self.node)
            self.markers = VizMarkerPublisher(_world=world_instance(), node=self.node)
            resources.callback(self.markers.stop)
            self.timer = self.node.create_timer(self.tick_period_seconds, self.tick)
            resources.callback(self.node.destroy_timer, self.timer)
            self.resources = resources.pop_all()
        self.node.get_logger().info(
            f"Aircraft beliefs: {self.markers.topic_name}; "
            f"RViz fixed frame: {world_instance().root.name}. "
            "Playback runs once; Ctrl-C stops inspection."
        )
        return self

    def tick(self) -> None:
        """Advance playback and refresh transforms while its world is stable."""
        if self.pipeline.status is not Status.FAILURE:
            self.pipeline.tick_once()
            if self.pipeline.status is Status.FAILURE:
                self.node.get_logger().info(
                    "Playback finished; the final belief world remains available. "
                    "Ctrl-C stops inspection."
                )
        if self.pipeline.status is Status.RUNNING:
            return
        self.markers.tf_publisher.on_state_change()

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> bool | None:
        """Finish outstanding belief updates before stopping world publishers."""
        self.timer.cancel()
        for annotator in self.pipeline.get_annotators():
            if (
                isinstance(annotator, ThreadedAnnotator)
                and annotator.compute_worker_thread is not None
            ):
                annotator.compute_worker_thread.join()
        return self.resources.__exit__(exception_type, exception, traceback)


# %% Inspection command


def main() -> None:
    """Serve the configured aircraft belief world until interrupted."""
    with ExitStack() as resources:
        rclpy.init()
        resources.callback(rclpy.try_shutdown)
        node = rclpy.create_node(InspectionNode.AIRCRAFT)
        resources.callback(node.destroy_node)
        registry = ROSNodeRegistry()
        registry.register(node)
        resources.callback(registry.clear, node)
        with AircraftBeliefInspection(node=node):
            rclpy.spin(node)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        main()
