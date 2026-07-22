"""
Operand overrides for krrood's own :class:`SymbolicSurfaceSnapshot`.

Kept separate from :mod:`test_verbalization_surfaces` so both that test and
``generate_verbalization_surfaces.py`` build their snapshot from the same overrides,
without the generation script reaching into a test module.
"""

from __future__ import annotations

from krrood.entity_query_language.predicate import HasType, HasTypes
from krrood.entity_query_language.testing.surface_verification import (
    OperandOverridesDict,
    OverriddenOperand,
)

KRROOD_OPERAND_OVERRIDES: OperandOverridesDict = {
    HasType: [OverriddenOperand("types_", int)],
    HasTypes: [OverriddenOperand("types_", (int, str))],
}
"""
``HasType`` / ``HasTypes`` read the type(s) by name, so they get a concrete type rather
than a symbolic operand; every other field defaults to a placeholder variable of its
annotated type.
"""
