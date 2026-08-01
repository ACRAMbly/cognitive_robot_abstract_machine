"""
Measure how long the control loop of Giskard needs per cycle.

Every measurement runs in a subprocess of its own, so that worlds, symbol graphs and ros
nodes of one scenario cannot slow down the next one. Call the script without arguments to
measure everything::

    python giskardpy/scripts/benchmark_control_loop.py

.. note::
    The scenarios need the ``iai_pr2_description``, ``iai_kitchen`` and ``iai_apartment``
    packages on the ros package path.
"""

from __future__ import annotations

import argparse
import cProfile
import json
import pstats
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List

from giskardpy.middleware.ros2.utils.control_loop_benchmark import (
    BENCHMARK_SCENARIOS,
    IsolatedBenchmarkSession,
    ScenarioRunner,
)

# %% one measurement


@dataclass
class BenchmarkRun:
    """
    One measurement of one scenario.
    """

    scenario_name: str
    """
    Which motion was measured.
    """

    debug_mode: bool
    """
    Whether the post goal plotters recorded the motion while it was measured.
    """

    repetition: int
    """
    Which of the repetitions of this configuration this is, counted from zero.
    """

    profile: Dict[str, Any]
    """
    What :class:`~giskardpy.middleware.ros2.control_loop_profiler.CallTreeProfile`
    measured, as plain data.
    """

    @property
    def cycle_samples(self) -> Dict[str, Any]:
        """
        The measurements of the whole control cycle.

        :raises ControlCycleMissingFromProfileError: If the profile holds no control
            cycle.
        """
        for phase in self.profile["phases"]:
            if phase["path"] == ["control_cycle"]:
                return phase
        raise ControlCycleMissingFromProfileError(self.scenario_name)

    def summary_row(self) -> str:
        """
        One line of the table comparing all runs.
        """
        cycle = self.cycle_samples
        mode = "debug" if self.debug_mode else "plain"
        return (
            f"{self.scenario_name:<36}{mode:>8}{self.repetition:>5}"
            f"{self.profile['control_cycles']:>9}"
            f"{cycle['inclusive_mean'] * 1000:>11.2f}"
            f"{cycle['inclusive_p95'] * 1000:>11.2f}"
            f"{cycle['inclusive_maximum'] * 1000:>11.2f}"
            f"{self.profile['budget_utilization']:>10.0%}"
            f"{self.profile['cycles_per_second']:>10.1f}"
            f"{self.profile['compile_duration'] * 1000:>12.1f}"
        )


class ControlCycleMissingFromProfileError(Exception):
    """
    Raised when a stored profile holds no measurement of a whole control cycle.
    """

    def __init__(self, scenario_name: str):
        super().__init__(
            f'The profile of "{scenario_name}" holds no control cycle, so the motion '
            f"never ran."
        )


SUMMARY_HEADER = (
    f"{'scenario':<36}{'mode':>8}{'rep':>5}{'cycles':>9}{'mean ms':>11}"
    f"{'p95 ms':>11}{'max ms':>11}{'budget':>10}{'Hz':>10}{'compile ms':>12}"
)
"""
Header of the table comparing all runs.
"""


# %% driving the measurements


@dataclass
class BenchmarkSweep:
    """
    Runs every requested configuration in a subprocess and collects the results.
    """

    scenario_names: List[str]
    """
    The motions that are measured.
    """

    debug_modes: List[bool]
    """
    Whether to measure with the post goal plotters recording, without, or both.
    """

    repeats: int
    """
    How often every configuration is measured.
    """

    target_frequency: float
    """
    Frequency the controller is discretized for, in hertz.
    """

    runs: List[BenchmarkRun] = field(default_factory=list)
    """
    The measurements taken so far.
    """

    def execute(self) -> None:
        """
        Measure every configuration and remember what came back.
        """
        for scenario_name in self.scenario_names:
            for debug_mode in self.debug_modes:
                for repetition in range(self.repeats):
                    self.runs.append(
                        self._measure_in_subprocess(
                            scenario_name, debug_mode, repetition
                        )
                    )

    def _measure_in_subprocess(
        self, scenario_name: str, debug_mode: bool, repetition: int
    ) -> BenchmarkRun:
        """
        Run one measurement in a process of its own and read its result.

        :raises MeasurementProcessFailedError: If the measuring process did not finish.
        """
        with tempfile.NamedTemporaryFile(suffix=".json") as result_file:
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--scenario",
                scenario_name,
                "--target-frequency",
                str(self.target_frequency),
                "--write-result-to",
                result_file.name,
            ]
            if debug_mode:
                command.append("--debug-mode")
            print(f"measuring {scenario_name} (debug={debug_mode}, run {repetition})")
            completed = subprocess.run(command)
            if completed.returncode != 0:
                raise MeasurementProcessFailedError(scenario_name, completed.returncode)
            return BenchmarkRun(
                scenario_name=scenario_name,
                debug_mode=debug_mode,
                repetition=repetition,
                profile=json.loads(Path(result_file.name).read_text()),
            )

    def format_summary(self) -> str:
        """
        Render every run as one line, so the spread between them is visible.
        """
        lines = [SUMMARY_HEADER, "-" * len(SUMMARY_HEADER)]
        lines.extend(run.summary_row() for run in self.runs)
        return "\n".join(lines)


class MeasurementProcessFailedError(Exception):
    """
    Raised when the subprocess measuring a scenario did not finish successfully.
    """

    def __init__(self, scenario_name: str, return_code: int):
        super().__init__(
            f'Measuring "{scenario_name}" failed with return code {return_code}.'
        )


# %% command line


def measure_one_scenario(arguments: argparse.Namespace) -> None:
    """
    Measure a single scenario and write the profile to the requested file.
    """
    scenario = BENCHMARK_SCENARIOS[arguments.scenario]()
    python_profiler = None if arguments.profile_to is None else cProfile.Profile()
    runner = ScenarioRunner(
        debug_mode=arguments.debug_mode,
        target_frequency=arguments.target_frequency,
        python_profiler=python_profiler,
    )
    with IsolatedBenchmarkSession():
        profile = runner.run(scenario)
        if python_profiler is not None:
            statistics = pstats.Stats(python_profiler)
            statistics.dump_stats(arguments.profile_to)
            statistics.sort_stats("tottime").print_stats(40)
    print(profile.format_report())
    if arguments.write_result_to is not None:
        Path(arguments.write_result_to).write_text(
            json.dumps(profile.to_dict(), indent=2)
        )


def run_sweep(arguments: argparse.Namespace) -> None:
    """
    Measure every requested configuration and print the comparison.
    """
    debug_modes = {"debug": [True], "plain": [False], "both": [False, True]}[
        arguments.plotters
    ]
    sweep = BenchmarkSweep(
        scenario_names=arguments.scenarios,
        debug_modes=debug_modes,
        repeats=arguments.repeats,
        target_frequency=arguments.target_frequency,
    )
    sweep.execute()
    print()
    print(sweep.format_summary())
    if arguments.write_result_to is not None:
        Path(arguments.write_result_to).write_text(
            json.dumps([run.profile for run in sweep.runs], indent=2)
        )


def parse_arguments() -> argparse.Namespace:
    """
    Describe what the script can measure.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=list(BENCHMARK_SCENARIOS),
        choices=list(BENCHMARK_SCENARIOS),
        help="which motions to measure",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=1,
        help="how often to measure every configuration",
    )
    parser.add_argument(
        "--plotters",
        default="both",
        choices=["debug", "plain", "both"],
        help="whether the post goal plotters record while measuring",
    )
    parser.add_argument(
        "--target-frequency",
        type=float,
        default=20.0,
        help="frequency the controller is discretized for, in hertz",
    )
    parser.add_argument(
        "--write-result-to", default=None, help="file the measurements are stored in"
    )
    parser.add_argument(
        "--scenario",
        default=None,
        choices=list(BENCHMARK_SCENARIOS),
        help="measure only this scenario, in this process",
    )
    parser.add_argument(
        "--debug-mode",
        action="store_true",
        help="let the post goal plotters record, only together with --scenario",
    )
    parser.add_argument(
        "--profile-to",
        default=None,
        help="file the python profile is stored in, only together with --scenario",
    )
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    if arguments.scenario is not None:
        measure_one_scenario(arguments)
        return
    run_sweep(arguments)


if __name__ == "__main__":
    main()
