"""Tests for orderly RoboKudo runtime shutdown."""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from robokudo.scripts import main


@dataclass
class ContextShutdownExecutor:
    """Raise the normal executor signal for a stopped ROS context."""

    spin_count: int = 0
    """Number of executor spin attempts."""

    def spin(self) -> None:
        """Simulate context shutdown during executor spinning."""
        self.spin_count += 1
        raise ExternalShutdownException


@dataclass
class OrderedShutdownTree:
    """Record behavior-tree shutdown order."""

    events: list[str]
    """Shared shutdown event sequence."""

    def shutdown(self) -> None:
        """Record behavior-tree shutdown."""
        self.events.append('tree')


@dataclass
class OrderedShutdownExecutor:
    """Record executor shutdown order."""

    name: str
    """Executor event name."""

    events: list[str]
    """Shared shutdown event sequence."""

    def shutdown(self) -> None:
        """Record executor shutdown."""
        self.events.append(self.name)


@dataclass
class OrderedShutdownThread:
    """Record executor-thread join order."""

    name: str
    """Thread event name."""

    events: list[str]
    """Shared shutdown event sequence."""

    def join(self) -> None:
        """Record thread joining."""
        self.events.append(self.name)


@dataclass
class OrderedShutdownNode:
    """Record ROS-node destruction order."""

    name: str
    """Node event name."""

    events: list[str]
    """Shared shutdown event sequence."""

    def destroy_node(self) -> None:
        """Record node destruction."""
        self.events.append(self.name)


def test_executor_spin_accepts_external_context_shutdown() -> None:
    """Treat executor context shutdown as normal termination."""
    executor = ContextShutdownExecutor()

    main.spin_executor(executor)

    assert executor.spin_count == 1


def test_runtime_shutdown_finalizes_tree_before_ros_context(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Finalize recorders before executors, nodes, and the ROS context."""
    events: list[str] = []
    monkeypatch.setattr(main.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(main.rclpy, 'shutdown', lambda: events.append('rclpy'))
    resources = main.RuntimeResources(
        analysis_tree=OrderedShutdownTree(events),
        executors=(
            OrderedShutdownExecutor('executor_main', events),
            OrderedShutdownExecutor('executor_query', events),
        ),
        threads=(
            OrderedShutdownThread('thread_main', events),
            OrderedShutdownThread('thread_query', events),
        ),
        nodes=(
            OrderedShutdownNode('node_main', events),
            OrderedShutdownNode('node_query', events),
        ),
    )

    resources.shutdown()

    assert events == [
        'tree',
        'executor_main',
        'executor_query',
        'thread_main',
        'thread_query',
        'node_main',
        'node_query',
        'rclpy',
    ]


def test_rclpy_initialization_leaves_sigint_for_keyboard_interrupt(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Retain ROS SIGTERM handling without consuming Ctrl+C."""
    initialization_arguments: dict[str, object] = {}

    def capture_initialization(**arguments: object) -> None:
        """Capture rclpy initialization arguments."""
        initialization_arguments.update(arguments)

    monkeypatch.setattr(main.rclpy, 'init', capture_initialization)

    main.initialize_rclpy(['main', '_ae=storage'])

    assert initialization_arguments == {
        'args': ['main', '_ae=storage'],
        'signal_handler_options': SignalHandlerOptions.SIGTERM,
    }
