from __future__ import annotations

from typing import TYPE_CHECKING

from krrood.entity_query_language.operators.logical_quantifiers import Exists, ForAll, QuantifiedConditional
from krrood.entity_query_language.verbalization.chain_utils import verbalize_plural
from krrood.entity_query_language.verbalization.fragments.base import PhraseFragment, VerbFragment
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule
from krrood.entity_query_language.verbalization.vocabulary.english import Keywords, Logicals

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


def _word(text: str) -> VerbFragment:
    from krrood.entity_query_language.verbalization.fragments.base import WordFragment
    return WordFragment(text=text)


def _phrase(*parts: VerbFragment, sep: str = " ") -> PhraseFragment:
    return PhraseFragment(parts=list(parts), separator=sep)


class QuantifierRule(VerbalizationRule):
    """Abstract base: catches ForAll and Exists."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, QuantifiedConditional)


class ForAllRule(QuantifierRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, ForAll)

    @classmethod
    def transform(cls, expr: ForAll, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        var_frag = verbalize_plural(expr.variable, ctx, delegate.build)
        cond_frag = delegate.build(expr.condition, ctx)
        return _phrase(Logicals.FOR_ALL.as_fragment(), var_frag, _word(","), cond_frag)


class ExistsRule(QuantifierRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Exists)

    @classmethod
    def transform(cls, expr: Exists, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        var_frag = delegate.build(expr.variable, ctx)
        cond_frag = delegate.build(expr.condition, ctx)
        return _phrase(
            Logicals.THERE_EXISTS.as_fragment(),
            var_frag,
            Keywords.SUCH_THAT.as_fragment(),
            cond_frag,
        )
