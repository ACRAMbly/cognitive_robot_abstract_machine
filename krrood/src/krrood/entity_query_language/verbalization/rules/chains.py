from __future__ import annotations

from typing import TYPE_CHECKING

from krrood.entity_query_language.core.mapped_variable import FlatVariable, MappedVariable
from krrood.entity_query_language.verbalization.fragments.base import VerbFragment
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


class MappedVariableRule(VerbalizationRule):
    """Handles all MappedVariable chains (Attribute, Index, Call); FlatVariable handled by subclass."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, MappedVariable)

    @classmethod
    def transform(cls, expr: MappedVariable, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return delegate._chain.verbalize_mapped(expr, ctx)


class FlatVariableRule(MappedVariableRule):
    """FlatVariable is a MappedVariable that unwraps to its child; tried before MappedVariableRule."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, FlatVariable)

    @classmethod
    def transform(cls, expr: FlatVariable, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return delegate._chain.verbalize_flat(expr, ctx)
