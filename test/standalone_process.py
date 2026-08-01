"""
Running a component in its own process, so tests can exercise a real process boundary.

A component started this way shares no interpreter state with the test, so every exchange
with it has to survive the middleware rather than a Python reference.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import pytest
from typing_extensions import Callable, List

READY_TIMEOUT = 180.0
"""
How long to wait for a launched component to announce itself.

Start-up can be slow on a loaded machine (a controller parses its robot description and
builds a collision model), so the wait polls rather than sleeping and costs only what it
needs.
"""

SHUTDOWN_TIMEOUT = 60.0
"""
How long to let a component shut down after SIGINT before killing it.
"""


@dataclass
class StandaloneProcess:
    """
    A component launched as its own process and stopped again when the test is done.
    """

    launcher_path: Path
    """
    Script to run.
    """

    is_ready: Callable[[], bool]
    """
    Whether the launched component is up yet.

    Supplied by the caller because only it knows what the component announces and which
    node to look from.
    """

    arguments: List[str] = field(default_factory=list)
    """
    Command-line arguments passed to the script.
    """

    ready_timeout: float = READY_TIMEOUT
    """
    How long to wait for :attr:`is_ready` to hold.
    """

    shutdown_timeout: float = SHUTDOWN_TIMEOUT
    """
    How long to let the process shut down after SIGINT before killing it.
    """

    process: subprocess.Popen | None = field(init=False, default=None)
    """
    The launched process, once started.
    """

    def start(self) -> None:
        """
        Launch the process and block until it is ready.
        """
        # Output is discarded rather than piped: an undrained pipe blocks the child once
        # it fills. start_new_session gives the process its own group, so it can be
        # signalled without hitting the test runner.
        self.process = subprocess.Popen(
            [sys.executable, str(self.launcher_path), *self.arguments],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        )
        self.wait_until_ready()

    def wait_until_ready(self) -> None:
        """
        Block until the launched component announces itself.

        :raises TimeoutError: If it does not within :attr:`ready_timeout`.
        """
        deadline = time.monotonic() + self.ready_timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                pytest.fail(
                    f"{self.launcher_path.name} died during start-up, "
                    f"rc={self.process.returncode}"
                )
            if self.is_ready():
                return
            time.sleep(0.5)
        raise TimeoutError(f"{self.launcher_path.name} never became ready")

    def stop(self) -> None:
        """
        Interrupt the process, killing it if it outstays its shutdown budget.
        """
        if self.process.poll() is not None:
            return
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        try:
            self.process.wait(timeout=self.shutdown_timeout)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()

    def __enter__(self) -> StandaloneProcess:
        self.start()
        return self

    def __exit__(self, exception_type, exception, traceback) -> None:
        self.stop()
