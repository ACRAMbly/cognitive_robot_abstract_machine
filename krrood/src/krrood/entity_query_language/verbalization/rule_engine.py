from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from krrood.entity_query_language.core.base_expressions import SymbolicExpression
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.fragments.base import VerbFragment
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


class VerbalizationRule(ABC):
    """
    Abstract base for a declarative verbalization rule.

    Subclass to declare when a rule fires (`applies`) and what fragment it produces
    (`transform`).  The RuleEngine sorts registered rule classes by inheritance depth
    so that more-specific subclasses are always tried before their parents — no priority
    integers needed.
    """

    @classmethod
    @abstractmethod
    def applies(cls, expr: SymbolicExpression, ctx: VerbalizationContext) -> bool:
        """Return True if this rule can handle `expr`."""

    @classmethod
    @abstractmethod
    def transform(
        cls,
        expr: SymbolicExpression,
        ctx: VerbalizationContext,
        delegate: EQLVerbalizer,
    ) -> VerbFragment:
        """Build and return the VerbFragment for `expr`."""


def _inheritance_depth(cls: type) -> int:
    try:
        return cls.__mro__.index(VerbalizationRule)
    except ValueError:
        return 0


class RuleEngine:
    """Applies the first matching VerbalizationRule to an expression, deepest subclass first."""

    def __init__(self, rule_classes: list[type[VerbalizationRule]]) -> None:
        self._rules = sorted(rule_classes, key=_inheritance_depth, reverse=True)

    def build(
        self,
        expr: SymbolicExpression,
        ctx: VerbalizationContext,
        delegate: EQLVerbalizer,
    ) -> VerbFragment:
        var_id = getattr(expr, "_id_", None)
        if var_id is not None and var_id in ctx.binding_overrides:
            return ctx.binding_overrides[var_id]
        for rule_cls in self._rules:
            if rule_cls.applies(expr, ctx):
                return rule_cls.transform(expr, ctx, delegate)
        from krrood.entity_query_language.verbalization.fragments.base import WordFragment
        return WordFragment(text=expr._name_)
