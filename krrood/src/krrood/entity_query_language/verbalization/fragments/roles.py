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
    from krrood.entity_query_language.core.variable import Variable, Literal, InstantiatedVariable
    from krrood.entity_query_language.core.mapped_variable import Attribute, Index, Call, FlatVariable
    from krrood.entity_query_language.operators.comparator import Comparator
    from krrood.entity_query_language.operators.core_logical_operators import AND, OR, Not
    from krrood.entity_query_language.operators.logical_quantifiers import ForAll, Exists
    from krrood.entity_query_language.operators.aggregators import (
        Average, Count, CountAll, Max, Min, Mode, MultiMode, Sum,
    )
    from krrood.entity_query_language.query.query import Entity, SetOf
    return {
        Variable:             SemanticRole.VARIABLE,
        InstantiatedVariable: SemanticRole.VARIABLE,
        Entity:               SemanticRole.VARIABLE,
        SetOf:                SemanticRole.VARIABLE,
        Literal:              SemanticRole.LITERAL,
        Attribute:            SemanticRole.ATTRIBUTE,
        Index:                SemanticRole.ATTRIBUTE,
        Call:                 SemanticRole.ATTRIBUTE,
        FlatVariable:         SemanticRole.ATTRIBUTE,
        Comparator:           SemanticRole.OPERATOR,
        AND:                  SemanticRole.LOGICAL,
        OR:                   SemanticRole.LOGICAL,
        Not:                  SemanticRole.LOGICAL,
        ForAll:               SemanticRole.LOGICAL,
        Exists:               SemanticRole.LOGICAL,
        Count:                SemanticRole.AGGREGATION,
        CountAll:             SemanticRole.AGGREGATION,
        Sum:                  SemanticRole.AGGREGATION,
        Average:              SemanticRole.AGGREGATION,
        Max:                  SemanticRole.AGGREGATION,
        Min:                  SemanticRole.AGGREGATION,
        Mode:                 SemanticRole.AGGREGATION,
        MultiMode:            SemanticRole.AGGREGATION,
    }


_role_map: Optional[dict[type, SemanticRole]] = None


def role_for(expr) -> SemanticRole:
    """Return the SemanticRole for an EQL expression instance (built once, cached)."""
    global _role_map
    if _role_map is None:
        _role_map = _build_role_map()
    return _role_map.get(type(expr), SemanticRole.PLAIN)
