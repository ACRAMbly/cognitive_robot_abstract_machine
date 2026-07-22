"""
Regenerates krrood's own committed verbalization-surface snapshot module.

Run this after an intentional wording change makes
``test_verbalization_surfaces.py``'s ``test_every_declared_surface_matches_what_its_class_renders``
print a new sentence, then review and commit the resulting diff.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "test"))

import krrood
from krrood.entity_query_language.testing.surface_generation import (
    VerbalizationSurfaceGenerator,
)
from krrood.entity_query_language.testing.surface_verification import (
    SymbolicSurfaceSnapshot,
)
from krrood_test.test_eql.test_verbalization.snapshot_config import (
    KRROOD_OPERAND_OVERRIDES,
)

SURFACES_MODULE_PATH = (
    REPOSITORY_ROOT
    / "test"
    / "krrood_test"
    / "test_eql"
    / "test_verbalization"
    / "verbalization_surfaces.py"
)


def regenerate() -> None:
    """
    Write the generated surfaces module to :data:`SURFACES_MODULE_PATH`.
    """
    snapshot = SymbolicSurfaceSnapshot(
        package=krrood, surfaces=(), operand_overrides=KRROOD_OPERAND_OVERRIDES
    )
    VerbalizationSurfaceGenerator(snapshot=snapshot).write(SURFACES_MODULE_PATH)


if __name__ == "__main__":
    regenerate()
