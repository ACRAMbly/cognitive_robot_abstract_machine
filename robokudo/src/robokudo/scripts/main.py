#!/usr/bin/env python3

from __future__ import annotations

import argparse
from contextlib import suppress
from dataclasses import dataclass
import logging
import os
import sys
import time
import traceback
from threading import Thread

# For time measurements or additional logic
from timeit import default_timer as timer

import rclpy
import rclpy.impl.logging_severity
import rclpy.logging
from py_trees.blackboard import Blackboard
from py_trees.common import Status
from rclpy.executors import (
    Executor,
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from typing_extensions import TYPE_CHECKING

# RoboKudo imports
from robokudo.annotators.query import QueryActionServer
from robokudo.defs import LOGGING_IDENTIFIER_MAIN_EXECUTABLE, PACKAGE_NAME
from robokudo.garden import grow_tree
from robokudo.identifier import BBIdentifier
from robokudo.io.ros import init_node
from robokudo.utils.logging_configuration import configure_logging
from robokudo.utils.module_loader import ModuleLoader
from robokudo.utils.tree import setup_with_descendants_rk

if TYPE_CHECKING:
    from py_trees_ros.trees import BehaviourTree
    from rclpy.node import Node

# Silence some TensorFlow GPU logs if needed
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"


def initialize_rclpy(arguments: list[str]) -> None:
    """Initialize ROS while leaving Ctrl+C handling to the main thread.

    :param arguments: Command-line arguments forwarded to ROS.
    """
    rclpy.init(
        args=arguments,
        signal_handler_options=SignalHandlerOptions.SIGTERM,
    )


def spin_executor(executor: Executor) -> None:
    """Spin an executor until interruption or ROS context shutdown.

    :param executor: ROS executor to spin.
    """
    with suppress(KeyboardInterrupt, ExternalShutdownException):
        executor.spin()


@dataclass(frozen=True, slots=True)
class RuntimeResources:
    """Own resources that require ordered application shutdown."""

    analysis_tree: BehaviourTree
    """Behavior tree containing runtime annotators and recorders."""

    executors: tuple[Executor, ...]
    """ROS executors serving application nodes."""

    threads: tuple[Thread, ...]
    """Threads spinning the ROS executors."""

    nodes: tuple[Node, ...]
    """ROS nodes owned by the application."""

    def shutdown(self) -> None:
        """Release resources from data producers toward the ROS context."""
        self.analysis_tree.shutdown()
        for executor in self.executors:
            executor.shutdown()
        for thread in self.threads:
            thread.join()
        for node in self.nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_ae(
    ae_name: str,
    ae_root: BehaviourTree,
    tickrate: int = 20,
) -> None:
    """Run an Analysis Engine (AE) by periodically ticking the Behavior Tree."""
    logger = logging.getLogger(LOGGING_IDENTIFIER_MAIN_EXECUTABLE)
    logger.info(f"Running AE named '{ae_name}'...")

    blackboard = Blackboard()
    blackboard.set("CAS", None)
    tick_count = 0

    def tick_tree() -> None:
        nonlocal tick_count
        try:
            logger.debug(f"--------- Tick {tick_count} ---------")
            start = timer()
            ae_root.tick()
            end = timer()
            logger.debug(f"Tick took {end - start:.4f} seconds")
            if (
                ae_root.root.children
                and ae_root.root.children[0].status == Status.FAILURE
            ):
                # If your top-level child fails, maybe shut down
                rclpy.shutdown()
            tick_count += 1
        except Exception as e:
            logger.error(f"Exception: {e}")
            logger.error("Traceback:\n" + traceback.format_exc())

    interval = 1.0 / tickrate
    next_tick = time.monotonic()

    while rclpy.ok():
        current_time = time.monotonic()
        elapsed = current_time - next_tick

        if elapsed >= interval:
            tick_tree()
            next_tick = current_time
        else:
            time.sleep(interval - elapsed)


def main() -> None:
    """
    Entry point for the RoboKudo system, setting up ROS, parsing arguments,
    loading the requested Analysis Engine, and spinning the ROS executors.
    """
    # 1. Parse CLI arguments (prefix_chars='_'):
    parser = argparse.ArgumentParser(prefix_chars="_")
    parser.add_argument(
        "_ae",
        dest="ae",
        type=str,
        nargs="?",
        const=1,
        default="demo",
        help="Analysis Engine to run (module name in descriptors/analysis_engines/).",
    )
    parser.add_argument(
        "_ros_pkg",
        dest="ros_pkg",
        type=str,
        nargs="?",
        const=1,
        default=PACKAGE_NAME,
        help="ROS package name containing the AE (default: robokudo).",
    )
    parser.add_argument(
        "_headless", action="store_true", help="If set, runs without a GUI."
    )
    parser.set_defaults(headless=False)
    parser.add_argument(
        "_nodesuffix",
        dest="nodesuffix",
        type=str,
        nargs="?",
        const=1,
        default="",
        help="A suffix to add to the ROS node name.",
    )
    parser.add_argument(
        "_tickrate",
        dest="tickrate",
        type=int,
        nargs="?",
        const=1,
        default=5,
        help="Rate (Hz) to tick the Behavior Tree.",
    )
    parser.add_argument(
        "_debugmode",
        action="store_true",
        help="If set, the rcply root logger will be set to DEBUG log level which will yield many ROS-related debug messages.",
    )
    parser.set_defaults(debugmode=False)
    args = parser.parse_args()

    # 2. Initialize RCL
    initialize_rclpy(sys.argv)

    if args.debugmode:
        rclpy.logging.set_logger_level(
            "", rclpy.impl.logging_severity.LoggingSeverity.DEBUG
        )

    # 3. Logging setup
    logger = logging.getLogger(LOGGING_IDENTIFIER_MAIN_EXECUTABLE)

    log_cfg_file = ModuleLoader.get_module_path(PACKAGE_NAME) / "logging_levels.yaml"
    configure_logging(logging_config_file_name=str(log_cfg_file))

    # 4. Create a main ROS node
    node_name = PACKAGE_NAME + args.nodesuffix
    node1 = init_node(
        node_name,
        parameter_overrides=[
            Parameter(
                "default_snapshot_stream", rclpy.parameter.Parameter.Type.BOOL, True
            ),
            Parameter(
                "default_snapshot_period", rclpy.parameter.Parameter.Type.DOUBLE, 2.0
            ),
        ],
    )
    logger.info(f"Created node: {node_name}")

    # 5. Create any action servers or supporting nodes
    query_action_server = QueryActionServer(name="query")
    blackboard = Blackboard()
    blackboard.set(BBIdentifier.QUERY_SERVER, query_action_server)
    blackboard.set(
        BBIdentifier.QUERY_SERVER_IN_PIPELINE, False
    )  # Ownership in Pipeline has to be declared first

    # 6. Start executors in separate threads
    executor_main = SingleThreadedExecutor()
    executor_asrv = (
        MultiThreadedExecutor()
    )  # Necessary to handle long-running goals AND incoming preempts

    executor_main.add_node(node1)
    executor_asrv.add_node(query_action_server)

    thread_main = Thread(target=spin_executor, args=(executor_main,), daemon=True)
    thread_asrv = Thread(target=spin_executor, args=(executor_asrv,), daemon=True)
    thread_main.start()
    thread_asrv.start()

    # 7. Dynamically load the requested Analysis Engine (AE) using the **refactored** ModuleLoader
    loader = ModuleLoader()
    logger.info(f"Loading AE '{args.ae}' from package '{args.ros_pkg}'...")
    loaded_ae = loader.load_ae(ros_pkg_name=args.ros_pkg, module_name=args.ae)

    # 8. Build your Behavior Tree from the loaded AE
    #    (Assuming loaded_ae.implementation() returns a py_trees root or something similar)
    ae_root = grow_tree(
        loaded_ae.implementation(), node=node1, include_gui=not args.headless
    )

    # If you have a custom version of `setup_with_descendants`, call it:
    setup_with_descendants_rk(ae_root)
    runtime_resources = RuntimeResources(
        analysis_tree=ae_root,
        executors=(executor_main, executor_asrv),
        threads=(thread_main, thread_asrv),
        nodes=(node1, query_action_server),
    )

    try:
        # 9. Start ticking the Behavior Tree
        run_ae(ae_name=args.ae, ae_root=ae_root, tickrate=args.tickrate)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received; shutting down.")
    finally:
        runtime_resources.shutdown()


if __name__ == "__main__":
    main()
