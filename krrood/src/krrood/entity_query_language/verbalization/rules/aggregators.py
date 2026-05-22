from __future__ import annotations

from typing import TYPE_CHECKING

from krrood.entity_query_language.operators.aggregators import (
    Aggregator, Count, CountAll, Sum, Average, Max, Min, Mode, MultiMode,
)
from krrood.entity_query_language.verbalization.chain_utils import verbalize_plural
from krrood.entity_query_language.verbalization.fragments.base import PhraseFragment, VerbFragment
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule
from krrood.entity_query_language.verbalization.utils import _str
from krrood.entity_query_language.verbalization.vocabulary.english import Aggregations, Articles

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


def _phrase(*parts: VerbFragment, sep: str = " ") -> PhraseFragment:
    return PhraseFragment(parts=list(parts), separator=sep)


_AGGREGATION_KIND: dict[type, Aggregations] = {
    Count:      Aggregations.COUNT,
    Sum:        Aggregations.SUM,
    Average:    Aggregations.AVERAGE,
    Max:        Aggregations.MAX,
    Min:        Aggregations.MIN,
    Mode:       Aggregations.MODE,
    MultiMode:  Aggregations.MULTI_MODE,
}


class AggregatorRule(VerbalizationRule):
    """Handles any Aggregator subtype via _AGGREGATION_KIND lookup."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Aggregator)

    @classmethod
    def transform(cls, expr: Aggregator, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        child_frag = verbalize_plural(expr._child_, ctx, delegate.build)
        agg_frag = _AGGREGATION_KIND[type(expr)].as_fragment()
        if expr._id_ in ctx.seen:
            return _phrase(Articles.THE.as_fragment(), agg_frag, child_frag)
        ctx.seen[expr._id_] = _str(_phrase(agg_frag, child_frag))
        return _phrase(agg_frag, child_frag)


class CountAllRule(AggregatorRule):
    """CountAll has no child; renders directly as 'the total count'."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, CountAll)

    @classmethod
    def transform(cls, expr: CountAll, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return Aggregations.COUNT_ALL.as_fragment()
