"""Launch the static synchronized Tetris source or receiving RViz world."""

from __future__ import annotations

import argparse
import signal
from contextlib import ExitStack
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum
from types import FrameType, TracebackType

import rclpy
from py_trees.blackboard import Blackboard
from py_trees.common import Status
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from typing_extensions import Self

from robokudo import world as perception
from robokudo.descriptors.analysis_engines.scripted_tetris_sync import (
    AnalysisEngine,
    SynchronizedTetrisIdentifier,
)
from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
from robokudo.descriptors.scripted_tetris_observations import create_tetris_scenario
from robokudo.io.scripted_observations import (
    InvalidScriptedObservationError,
    ScriptedIdentifier,
    ScriptedScenario,
)
from robokudo.synchronized_tetris import SynchronizedTetrisReceiver
from semantic_digital_twin.adapters.ros.node_registry import ROSNodeRegistry
from semantic_digital_twin.world import World

# %% process lifecycle


class ProcessRole(StrEnum):
    """Processes participating in the static demonstration."""

    SOURCE = "source"
    """Accept the initial observation and retain its native stream."""
    RECEIVER = "receiver"
    """Receive changes and publish geometry and transforms."""


class LaunchArgument(StrEnum):
    """Command-line controls for the demonstration."""

    ROLE = "role"
    """Process role positional argument."""
    TOPIC = "--topic"
    """Native synchronization topic override."""
    RECEIVER_TIMEOUT = "--receiver-timeout"
    """Seconds allowed for receiver discovery."""
    NAMESPACE = "--namespace"
    """ROS namespace isolating visualization topics between demonstrations."""


@dataclass
class SynchronizedTetrisApplication:
    """Own ROS resources for one source or receiving process."""

    role: ProcessRole
    """Whether this process accepts observations or displays replicated beliefs."""
    synchronization_topic: str = SynchronizedTetrisIdentifier.TOPIC
    """Dedicated native stream shared by both processes."""
    receiver_timeout: timedelta = timedelta(seconds=5)
    """Maximum source startup wait for the required receiver."""
    namespace: str = ""
    """ROS namespace for application nodes and visualization topics."""
    executor_interval: timedelta = timedelta(milliseconds=20)
    """Maximum time spent waiting for ROS work between stop checks."""
    context: Context = field(default_factory=Context, init=False)
    """ROS context belonging exclusively to this application."""
    node: Node = field(init=False)
    """Application-owned node shared with its perception stages."""
    executor: SingleThreadedExecutor = field(init=False)
    """Executor spun by the application thread."""
    resources: ExitStack = field(default_factory=ExitStack, init=False)
    """Cleanup actions for acquired ROS resources."""
    engine: AnalysisEngine | None = field(default=None, init=False)
    """Source engine whose worker and synchronizer must be stopped."""
    receiver: SynchronizedTetrisReceiver | None = field(default=None, init=False)
    """Receiving world and its visualization resources."""
    stopping: bool = field(default=False, init=False)
    """Whether a termination signal has requested an orderly exit."""
    closed: bool = field(default=False, init=False)
    """Whether owned resources have been released."""

    def __enter__(self) -> Self:
        """Acquire the context, node, registration, and executor."""
        with ExitStack() as resources:
            rclpy.init(context=self.context)
            resources.callback(self.context.try_shutdown)
            self.node = Node(
                f"scripted_tetris_{self.role}",
                context=self.context,
                namespace=self.namespace,
            )
            resources.callback(self.node.destroy_node)
            registry = ROSNodeRegistry()
            if registry.has_node():
                resources.callback(registry.register, registry.get())
            registry.register(self.node)
            resources.callback(registry.clear, self.node)
            self.executor = SingleThreadedExecutor(context=self.context)
            resources.callback(self.executor.shutdown)
            self.executor.add_node(self.node)
            resources.callback(self.executor.remove_node, self.node)
            self.resources = resources.pop_all()
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Release owned resources even when startup or processing fails."""
        self.close()

    def run(self) -> None:
        """Receive continuously or accept one initial observation and hold it."""
        if self.role is ProcessRole.RECEIVER:
            self.receiver = SynchronizedTetrisReceiver(
                node=self.node,
                synchronization_topic=self.synchronization_topic,
            )
            self.node.get_logger().info(
                f"Receiver ready on {self.synchronization_topic}"
            )
            while not self.stopping and self.context.ok():
                self.executor.spin_once(
                    timeout_sec=self.executor_interval.total_seconds()
                )
            return

        perception.set_world(World())
        perception.get_object_belief_states().clear()
        self.engine = AnalysisEngine(
            node=self.node,
            synchronization_topic=self.synchronization_topic,
            receiver_timeout=self.receiver_timeout,
            scenario=ScriptedScenario(create_tetris_scenario().observations[:1]),
        )
        Blackboard().set(
            ScriptedIdentifier.OUTPUT_BUFFER, AnnotatorOutputPerPipelineMap()
        )
        pipeline = self.engine.implementation()
        pipeline.setup()
        accepted = False
        while not self.stopping and self.context.ok():
            self.executor.spin_once(timeout_sec=self.executor_interval.total_seconds())
            if accepted:
                continue
            pipeline.tick_once()
            if pipeline.status is Status.FAILURE:
                raise InvalidScriptedObservationError(
                    "The initial scripted observation was not accepted."
                )
            if pipeline.status is Status.SUCCESS:
                accepted = True
                self.node.get_logger().info(
                    f"Accepted {len(perception.get_object_belief_states())} CAD beliefs; "
                    f"stream position: {self.engine.synchronizer.latest_published_position}"
                )

    def request_stop(self, signum: int, stack_frame: FrameType | None) -> None:
        """Request orderly resource cleanup after a termination signal."""
        self.stopping = True

    def close(self) -> None:
        """Stop world callbacks before releasing their ROS node and context."""
        if self.closed:
            return
        self.closed = True
        with self.resources:
            if self.engine is not None:
                self.engine.shutdown()
            if self.receiver is not None:
                self.receiver.close()


# %% command-line entry point


def main() -> None:
    """Run the selected process until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        LaunchArgument.ROLE, type=ProcessRole, choices=list(ProcessRole)
    )
    parser.add_argument(
        LaunchArgument.TOPIC, default=SynchronizedTetrisIdentifier.TOPIC
    )
    parser.add_argument(LaunchArgument.RECEIVER_TIMEOUT, type=float, default=5.0)
    parser.add_argument(LaunchArgument.NAMESPACE, default="")
    arguments = parser.parse_args()
    application = SynchronizedTetrisApplication(
        role=arguments.role,
        synchronization_topic=arguments.topic,
        receiver_timeout=timedelta(seconds=arguments.receiver_timeout),
        namespace=arguments.namespace,
    )
    with application, ExitStack() as handlers:
        for termination in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(termination, application.request_stop)
            handlers.callback(signal.signal, termination, previous)
        application.run()


if __name__ == "__main__":
    main()
