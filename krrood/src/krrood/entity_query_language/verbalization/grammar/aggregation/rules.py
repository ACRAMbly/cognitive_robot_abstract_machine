from __future__ import annotations

from krrood.entity_query_language.operators.aggregators import Aggregator
from krrood.entity_query_language.verbalization.fragments.base import (
    Fragment,
    NounPhrase,
)
from krrood.entity_query_language.verbalization.fragments.features import (
    Definiteness,
    Number,
)
from krrood.entity_query_language.verbalization.grammar.aggregation.kinds import (
    AGGREGATION_KIND,
)
from krrood.entity_query_language.verbalization.grammar.framework.phrase_rule import (
    PhraseRule,
    RuleContext,
)
from krrood.entity_query_language.verbalization.vocabulary.english import Prepositions
from krrood.entity_query_language.verbalization.vocabulary.words import ChildForm


class AggregatorRule(PhraseRule):
    """*"the <aggregation> <plural child>"* (or *"the <aggregation> of <child>"*).

    >>> verbalize_expression(max(variable(Robot, []).battery))
    'the maximum of the battery of a Robot'
    """

    construct = Aggregator
    name = "aggregator"

    def build(self, node: Aggregator, context: RuleContext) -> Fragment:
        aggregation_kind = AGGREGATION_KIND[type(node)]
        aggregation_word = aggregation_kind.value
        aggregation_fragment = aggregation_kind.as_fragment()

        if aggregation_word.child_form is ChildForm.NONE:
            return aggregation_fragment  # childless aggregate, e.g. "count of all"
        if aggregation_word.child_form == ChildForm.SINGULAR_OF:
            child_fragment = context.child(node._child_)
            modifiers = [Prepositions.OF.as_fragment(), child_fragment]
        else:
            child_fragment = context.child(node._child_, number=Number.PLURAL)
            modifiers = [child_fragment]
        return NounPhrase(
            head=aggregation_fragment,
            definiteness=Definiteness.DEFINITE,
            modifiers=modifiers,
        )
