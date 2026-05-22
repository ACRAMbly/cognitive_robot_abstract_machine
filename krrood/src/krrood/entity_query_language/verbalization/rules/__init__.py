from __future__ import annotations

from krrood.entity_query_language.verbalization.rules.logical import (
    LogicalRule, AndRule, OrRule, NotRule, NotComparatorRule, NotBoolAttrRule,
)
from krrood.entity_query_language.verbalization.rules.quantifiers import (
    QuantifierRule, ForAllRule, ExistsRule,
)
from krrood.entity_query_language.verbalization.rules.comparator import ComparatorRule
from krrood.entity_query_language.verbalization.rules.aggregators import AggregatorRule, CountAllRule
from krrood.entity_query_language.verbalization.rules.variables import (
    VariableRule, LiteralRule, ExternallySetVariableRule,
    InstantiatedVariableRule, InstantiatedVerbalizableRule,
)
from krrood.entity_query_language.verbalization.rules.chains import MappedVariableRule, FlatVariableRule
from krrood.entity_query_language.verbalization.rules.query import (
    EntityRule, SetOfRule, ResultQuantifierRule, FilterRule, GroupedByRule, OrderedByRule,
)

ALL_RULES: list[type] = [
    # logical — specific patterns first, generic Not last
    NotComparatorRule, NotBoolAttrRule, NotRule,
    AndRule, OrRule,
    # quantifiers
    ForAllRule, ExistsRule,
    # comparator
    ComparatorRule,
    # instantiated variables — template form before natural form
    InstantiatedVerbalizableRule, InstantiatedVariableRule,
    # plain variables — Literal before Variable
    LiteralRule, ExternallySetVariableRule, VariableRule,
    # aggregators — CountAll before generic Aggregator
    CountAllRule, AggregatorRule,
    # query structures
    EntityRule, SetOfRule,
    ResultQuantifierRule,
    FilterRule,
    GroupedByRule, OrderedByRule,
    # mapped chains — FlatVariable before MappedVariable (via subclass depth)
    FlatVariableRule, MappedVariableRule,
]
