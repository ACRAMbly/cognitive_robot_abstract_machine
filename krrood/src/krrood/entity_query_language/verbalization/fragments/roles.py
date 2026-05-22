from __future__ import annotations

from enum import Enum
from typing import Optional


class SemanticRole(Enum):
    KEYWORD = "keyword"        # If, Then, Find, Where, Such that
    VARIABLE = "variable"      # Robot, Employee 1
    AGGREGATION = "aggregation"  # sum of, number of, average of
    OPERATOR = "operator"      # is greater than, equals
    LOGICAL = "logical"        # and, or, not, for all, there exists
    LITERAL = "literal"        # 42, "hello", True
    ATTRIBUTE = "attribute"    # battery, tasks, name
    PLAIN = "plain"            # neutral connecting text


# Hex colours taken directly from QueryGraph.ColorLegend
ROLE_COLORS: dict[SemanticRole, Optional[str]] = {
    SemanticRole.KEYWORD:     "#eded18",        # ConclusionSelector yellow
    SemanticRole.VARIABLE:    "cornflowerblue",
    SemanticRole.AGGREGATION: "#F54927",         # Aggregator red-orange
    SemanticRole.OPERATOR:    "#ff7f0e",         # Comparator orange
    SemanticRole.LOGICAL:     "#2ca02c",         # LogicalOperator green
    SemanticRole.LITERAL:     "#949292",         # Literal gray
    SemanticRole.ATTRIBUTE:   "#8FC7B8",         # MappedVariable teal
    SemanticRole.PLAIN:       None,
}


def _build_role_map() -> dict[type, SemanticRole]:
    from krrood.entity_query_language.operators.core_logical_operators import LogicalOperator
    from krrood.entity_query_language.operators.aggregators import Aggregator
    from krrood.entity_query_language.operators.comparator import Comparator
    from krrood.entity_query_language.core.mapped_variable import MappedVariable
    from krrood.entity_query_language.core.variable import Variable, Literal
    from krrood.entity_query_language.query.query import Entity, SetOf
    return {
        LogicalOperator: SemanticRole.LOGICAL,
        Aggregator:      SemanticRole.AGGREGATION,
        Comparator:      SemanticRole.OPERATOR,
        MappedVariable:  SemanticRole.ATTRIBUTE,
        Literal:         SemanticRole.LITERAL,   # before Variable in MRO traversal
        Variable:        SemanticRole.VARIABLE,
        Entity:          SemanticRole.VARIABLE,
        SetOf:           SemanticRole.VARIABLE,
    }


_role_map: Optional[dict[type, SemanticRole]] = None


def role_for(expr) -> SemanticRole:
    """Return the SemanticRole for an EQL expression instance, using MRO for inheritance."""
    global _role_map
    if _role_map is None:
        _role_map = _build_role_map()
    for cls in type(expr).__mro__:
        if cls in _role_map:
            return _role_map[cls]
    return SemanticRole.PLAIN
