"""
The demo against a controller running in its own process.

The in-process integration test shares interpreter state -- notably the
:class:`GiskardBlackboard` borg and the ``rospy`` module globals -- with the controller
it talks to. On the robot the controller is a separate process, so nothing is shared and
every exchange has to survive the middleware. This test reproduces that topology.
"""

import os
import signal
import subprocess
import sys
import threading
import time
from importlib.resources import files

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from experiments.real_stretch_apartment_demo import demo
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service

CONTROLLER_READY_TIMEOUT = 180.0
"""
How long to wait for the controller to advertise its world service.

Start-up parses the robot description and builds the collision model, which is slow on a
loaded machine; the wait polls rather than sleeping so it costs only what it needs.
"""

CONTROLLER_SHUTDOWN_TIMEOUT = 60.0
"""
How long to let the controller shut down after SIGINT before killing it.
"""

PLACEMENT_TOLERANCE = 0.2
"""
How far from the bedside table's centre the cereal box may land.

The measured placement error is about 0.1m, so this discriminates a successful place
from one that missed the furniture without being tight enough to chase controller noise.
"""


def wait_for_world_service(node, deadline: float) -> None:
    """
    Block until the controller advertises a world-fetch service.

    :param node: Node used to inspect the graph.
    :param deadline: Monotonic time after which to give up.
    :raises TimeoutError: If no such service appears in time.
    """
    while time.monotonic() < deadline:
        if any(
            name.endswith("fetch_world")
            for name, _ in node.get_service_names_and_types()
        ):
            return
        time.sleep(0.5)
    raise TimeoutError("controller never advertised a fetch_world service")


@pytest.fixture()
def controller_process():
    """
    A standalone Giskard controller running in its own process, as on the robot.
    """
    launcher = files("giskardpy").joinpath(
        "middleware/ros2/scripts/iai_robots/stretch/stretch_standalone.py"
    )
    # Output goes to a file rather than a pipe: an undrained pipe blocks the child once
    # it fills, and start_new_session gives the controller its own process group so it
    # can be signalled without hitting the test runner.
    log = open(os.devnull, "wb")
    process = subprocess.Popen(
        [sys.executable, str(launcher)],
        stdout=log,
        stderr=log,
        start_new_session=True,
        env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
    )
    try:
        rclpy.init()
        node = rclpy.create_node("process_boundary_probe")
        try:
            wait_for_world_service(node, time.monotonic() + CONTROLLER_READY_TIMEOUT)
            if process.poll() is not None:
                pytest.fail(f"controller died during start-up, rc={process.returncode}")
            yield process
        finally:
            node.destroy_node()
    finally:
        if process.poll() is None:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            try:
                process.wait(timeout=CONTROLLER_SHUTDOWN_TIMEOUT)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        log.close()
        if rclpy.ok():
            rclpy.shutdown()


def test_demo_runs_against_a_controller_in_another_process(
    controller_process, monkeypatch
):
    """
    The demo drives a controller it shares no interpreter state with, so every exchange
    crosses a real process boundary: fetching the world, synchronizing the furniture it
    spawns, and executing each action.

    The result is read back by fetching the world from the controller again, which
    proves the furniture and the transported object landed in the controller's own
    process rather than only in the demo's copy.
    """
    monkeypatch.setenv("STRETCH_DEMO_EXECUTION", "REAL")

    demo.main()

    # The service response is delivered by an executor, so the node reading the result
    # back has to be spun; a bare node would block until the timeout.
    node = rclpy.create_node("process_boundary_result")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True, name="result-spinner")
    spinner.start()
    try:
        controller_world = fetch_world_from_service(node, timeout_seconds=60)
    finally:
        executor.shutdown()
        node.destroy_node()

    cereal = controller_world.get_body_by_name("cheeze_it.obj")
    bedside_table = controller_world.get_body_by_name("bedside_table.dae")
    assert cereal.parent_connection.parent is not controller_world.get_body_by_name(
        "shelf_layer2"
    )
    np.testing.assert_allclose(
        controller_world.compute_forward_kinematics(controller_world.root, cereal)
        .to_position()
        .to_np()[:2],
        controller_world.compute_forward_kinematics(
            controller_world.root, bedside_table
        )
        .to_position()
        .to_np()[:2],
        atol=PLACEMENT_TOLERANCE,
    )
