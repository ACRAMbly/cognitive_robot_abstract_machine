"""
Tests that RoboKudo modules can be imported on their own, in any order.
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest

IMPORT_SCRIPT = Path(__file__).parent / "scripts" / "import_module.py"
"""
Script importing the module named by its argument.
"""


def _import_in_fresh_interpreter(module_name: str) -> subprocess.CompletedProcess:
    """
    Import the module in a new interpreter that sees the same packages as this one.
    """
    environment = os.environ | {
        "PYTHONPATH": os.pathsep.join(path for path in sys.path if path)
    }
    return subprocess.run(
        [sys.executable, str(IMPORT_SCRIPT), module_name],
        env=environment,
        capture_output=True,
        text=True,
    )


@pytest.mark.parametrize(
    "module_name",
    ["robokudo.annotators.outputs", "robokudo.annotators.core", "robokudo.pipeline"],
)
def test_module_imports_before_any_other_robokudo_module(module_name):
    completed = _import_in_fresh_interpreter(module_name)

    assert completed.returncode == 0, completed.stderr
