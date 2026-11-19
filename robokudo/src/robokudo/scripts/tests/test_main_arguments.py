"""Tests for the RoboKudo command-line arguments."""

from __future__ import annotations

from pathlib import Path

from robokudo.scripts import main


def test_output_directory_argument_is_parsed_as_path() -> None:
    """Accept the recording directory as a command-line argument."""
    arguments = main.create_argument_parser().parse_args(
        ['_ae=storage_red', '_output_directory=/data/run_01']
    )

    assert arguments.output_directory == Path('/data/run_01')


def test_output_directory_argument_is_optional() -> None:
    """Leave the recording directory unset when it is not given."""
    arguments = main.create_argument_parser().parse_args(['_ae=storage_red'])

    assert arguments.output_directory is None
