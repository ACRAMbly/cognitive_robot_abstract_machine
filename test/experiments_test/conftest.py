"""
Fixtures for running demonstrations against a controller in its own process.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from importlib.resources import files
from pathlib import Path

import pytest

from experiments.demonstration import RobotDemonstrationRosSession

# %% standalone controller process

CONTROLLER_READY_TIMEOUT = 180.0
"""
How long to wait for a controller to advertise its world service.

Start-up parses the robot description and builds the collision model, which is slow on a
loaded machine; the wait polls rather than sleeping so it costs only what it needs.
"""

CONTROLLER_SHUTDOWN_TIMEOUT = 60.0
"""
How long to let a controller shut down after SIGINT before killing it.
"""


@dataclass
class StandaloneControllerProcess:
    """
    A standalone Giskard controller running in its own process, as on the robot.

    Nothing is shared with the test's interpreter, so every exchange with the controller
    has to survive the middleware.
    """

    launcher_path: Path
    """
    Standalone controller script to run.
    """

    ready_timeout: float = CONTROLLER_READY_TIMEOUT
    """
    How long to wait for the controller to advertise its world service.
    """

    shutdown_timeout: float = CONTROLLER_SHUTDOWN_TIMEOUT
    """
    How long to let the controller shut down after SIGINT before killing it.
    """

    process: subprocess.Popen | None = field(init=False, default=None)
    """
    The controller process, once started.
    """

    session: RobotDemonstrationRosSession | None = field(init=False, default=None)
    """
    Session watching the controller's services.

    It owns the ROS context for the whole test, so a demonstration running against this
    controller finds a context it did not create and leaves it alone.
    """

    def start(self) -> None:
        """
        Launch the controller and block until it serves its world.
        """
        # Output is discarded rather than piped: an undrained pipe blocks the child once
        # it fills. start_new_session gives the controller its own process group, so it
        # can be signalled without hitting the test runner.
        self.process = subprocess.Popen(
            [sys.executable, str(self.launcher_path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        )
        self.session = RobotDemonstrationRosSession.start("controller_process_probe")
        self.wait_until_ready()

    def wait_until_ready(self) -> None:
        """
        Block until the controller advertises a world-fetch service.

        :raises TimeoutError: If no such service appears in time.
        """
        deadline = time.monotonic() + self.ready_timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                pytest.fail(
                    f"controller died during start-up, rc={self.process.returncode}"
                )
            if any(
                name.endswith("fetch_world")
                for name, _ in self.session.node.get_service_names_and_types()
            ):
                return
            time.sleep(0.5)
        raise TimeoutError("controller never advertised a fetch_world service")

    def stop(self) -> None:
        """
        Interrupt the controller, killing it if it outstays its shutdown budget.
        """
        if self.process.poll() is None:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            try:
                self.process.wait(timeout=self.shutdown_timeout)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.session.stop()

    def __enter__(self) -> StandaloneControllerProcess:
        self.start()
        return self

    def __exit__(self, exception_type, exception, traceback) -> None:
        self.stop()


@pytest.fixture()
def stretch_controller_process():
    """
    A standalone Giskard controller for the Stretch, running in its own process.
    """
    launcher = files("giskardpy").joinpath(
        "middleware/ros2/scripts/iai_robots/stretch/stretch_standalone.py"
    )
    with StandaloneControllerProcess(launcher_path=Path(str(launcher))) as controller:
        yield controller
