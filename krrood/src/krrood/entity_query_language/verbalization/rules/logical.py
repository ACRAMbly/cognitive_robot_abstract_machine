from __future__ import annotations

from typing import TYPE_CHECKING

from krrood.entity_query_language.core.mapped_variable import Attribute, MappedVariable
from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.operators.core_logical_operators import AND, OR, Not, LogicalOperator
from krrood.entity_query_language.verbalization.fragments.base import PhraseFragment, RoleFragment, VerbFragment
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule
from krrood.entity_query_language.verbalization.vocabulary.english import Conjunctions, Logicals, Operators

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


def _word(text: str) -> VerbFragment:
    from krrood.entity_query_language.verbalization.fragments.base import WordFragment
    return WordFragment(text=text)


def _phrase(*parts: VerbFragment, sep: str = " ") -> PhraseFragment:
    return PhraseFragment(parts=list(parts), separator=sep)


def _join_with(fragments: list[VerbFragment], separator: str) -> VerbFragment:
    if not fragments:
        return _word("")
    if len(fragments) == 1:
        return fragments[0]
    result: list[VerbFragment] = []
    for i, frag in enumerate(fragments):
        result.append(frag)
        if i < len(fragments) - 1:
            result.append(_word(separator))
    return PhraseFragment(parts=result, separator="")


def _oxford_and(fragments: list[VerbFragment], conjunction: VerbFragment) -> VerbFragment:
    if len(fragments) == 1:
        return fragments[0]
    head = fragments[:-1]
    tail = fragments[-1]
    parts: list[VerbFragment] = []
    for f in head:
        parts.append(f)
        parts.append(_word(", "))
    parts.append(PhraseFragment(parts=[conjunction, tail], separator=" "))
    return PhraseFragment(parts=parts, separator="")


def _is_bool_attr_chain(expr) -> bool:
    if not isinstance(expr, MappedVariable):
        return False
    from krrood.entity_query_language.verbalization.chain_utils import walk_chain
    chain, _ = walk_chain(expr)
    return bool(chain) and isinstance(chain[-1], Attribute) and chain[-1]._type_ is bool


class LogicalRule(VerbalizationRule):
    """Abstract base: catches any LogicalOperator; concrete subclasses handle specific types."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, LogicalOperator)


class AndRule(LogicalRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, AND)

    @classmethod
    def transform(cls, expr: AND, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        parts = [delegate.build(c, ctx) for c in ctx.flatten_same_type(expr, AND)]
        if len(parts) == 1:
            return parts[0]
        return _oxford_and(parts, Conjunctions.AND.as_fragment())


class OrRule(LogicalRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, OR)

    @classmethod
    def transform(cls, expr: OR, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        parts = [delegate.build(c, ctx) for c in ctx.flatten_same_type(expr, OR)]
        if len(parts) == 1:
            return parts[0]
        head_with_comma = PhraseFragment(
            parts=[_join_with(parts[:-1], ", "), _word(",")], separator=""
        )
        return _phrase(Logicals.EITHER.as_fragment(), head_with_comma, Conjunctions.OR.as_fragment(), parts[-1])


class NotRule(LogicalRule):
    """Generic Not: wraps child in 'not (...)'."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Not)

    @classmethod
    def transform(cls, expr: Not, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        child_frag = delegate.build(expr._child_, ctx)
        return _phrase(
            Logicals.NOT.as_fragment(),
            PhraseFragment(parts=[_word("("), child_frag, _word(")")], separator=""),
        )


class NotComparatorRule(NotRule):
    """Not wrapping a Comparator: negates the comparison operator inline."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Not) and isinstance(expr._child_, Comparator)

    @classmethod
    def transform(cls, expr: Not, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        child = expr._child_
        left = delegate.build(child.left, ctx)
        right = delegate.build(child.right, ctx)
        is_temporal = delegate._chain.is_temporal(child.left) or delegate._chain.is_temporal(child.right)
        try:
            op_frag = Operators.from_callable(child.operation).select(
                negated=True, compact=ctx.compact_predicates, temporal=is_temporal
            ).as_fragment()
        except KeyError:
            op_frag = RoleFragment.for_operator(f"not {child._name_}")
        return _phrase(left, op_frag, right)


class NotBoolAttrRule(NotRule):
    """Not wrapping a bool Attribute chain: produces '<nav> is not <attr>'."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Not) and _is_bool_attr_chain(expr._child_)

    @classmethod
    def transform(cls, expr: Not, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return delegate._chain.verbalize_mapped_negated(expr._child_, ctx)
