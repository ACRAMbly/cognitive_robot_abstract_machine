from __future__ import annotations

import json
import subprocess
import sys
import tomllib
from pathlib import Path

from packaging.requirements import Requirement

import krrood.symbol_graph.symbol_graph

PACKAGE_DIRECTORY = Path(__file__).parents[3] / "krrood"
"""
The directory holding krrood's ``pyproject.toml``.
"""

PROBE = Path(__file__).parents[1] / "dataset" / "loaded_top_level_packages.py"
"""
The script that imports a module in an interpreter of its own, so that what it loads is
not already loaded by the test run.
"""

# %% optional dependencies


def test_importing_the_symbol_graph_loads_no_optional_dependency():
    """
    Every :class:`~krrood.symbol_graph.symbol_graph.Symbol` subclass imports the symbol
    graph, so importing it must not load a package krrood declares only as optional.
    """
    declaration = tomllib.loads((PACKAGE_DIRECTORY / "pyproject.toml").read_text())
    project = declaration["project"]
    required = {Requirement(r).name for r in project["dependencies"]}
    optional = {
        Requirement(requirement).name
        for extra in project["optional-dependencies"].values()
        for requirement in extra
    } - required
    probe = subprocess.run(
        [
            sys.executable,
            "-I",
            PROBE,
            krrood.symbol_graph.symbol_graph.__name__,
        ],
        capture_output=True,
        text=True,
        check=True,
    )

    assert set(json.loads(probe.stdout)) & optional == set()
