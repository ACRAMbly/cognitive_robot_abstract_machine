from __future__ import annotations

from typing import TYPE_CHECKING

from krrood.entity_query_language.core.base_expressions import Filter
from krrood.entity_query_language.query.operations import GroupedBy, OrderedBy
from krrood.entity_query_language.query.quantifiers import ResultQuantifier
from krrood.entity_query_language.query.query import Entity, SetOf
from krrood.entity_query_language.verbalization.fragments.base import PhraseFragment, VerbFragment
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule
from krrood.entity_query_language.verbalization.vocabulary.english import Keywords, SortDirections

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


def _word(text: str) -> VerbFragment:
    from krrood.entity_query_language.verbalization.fragments.base import WordFragment
    return WordFragment(text=text)


def _phrase(*parts: VerbFragment, sep: str = " ") -> PhraseFragment:
    return PhraseFragment(parts=list(parts), separator=sep)


class EntityRule(VerbalizationRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Entity)

    @classmethod
    def transform(cls, expr: Entity, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return delegate._entity.verbalize_query(expr, ctx)


class SetOfRule(VerbalizationRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, SetOf)

    @classmethod
    def transform(cls, expr: SetOf, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return delegate._entity.verbalize_set_of(expr, ctx)


class ResultQuantifierRule(VerbalizationRule):
    """An, The, and ResultQuantifier are transparent wrappers; delegates to child."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, ResultQuantifier)

    @classmethod
    def transform(cls, expr: ResultQuantifier, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return delegate.build(expr._child_, ctx)


class FilterRule(VerbalizationRule):
    """Where and Having both delegate to their condition expression."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Filter)

    @classmethod
    def transform(cls, expr: Filter, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return delegate.build(expr.condition, ctx)


class GroupedByRule(VerbalizationRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, GroupedBy)

    @classmethod
    def transform(cls, expr: GroupedBy, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        if expr.variables_to_group_by:
            groups = [delegate.verbalize(v, ctx) for v in expr.variables_to_group_by]
            return _phrase(Keywords.GROUPED_BY.as_fragment(), _word(", ".join(groups)))
        return Keywords.GROUPED.as_fragment()


class OrderedByRule(VerbalizationRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, OrderedBy)

    @classmethod
    def transform(cls, expr: OrderedBy, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        direction = SortDirections.DESCENDING.text if expr.descending else SortDirections.ASCENDING.text
        return _phrase(
            Keywords.ORDERED_BY.as_fragment(),
            _word(f"{delegate.verbalize(expr.variable, ctx)} ({direction})"),
        )
