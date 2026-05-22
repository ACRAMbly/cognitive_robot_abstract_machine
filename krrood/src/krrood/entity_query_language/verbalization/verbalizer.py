from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.verbalization.chain_verbalizer import ChainVerbalizer
from krrood.entity_query_language.verbalization.context import VerbalizationContext
from krrood.entity_query_language.verbalization.entity_verbalizer import EntityVerbalizer
from krrood.entity_query_language.verbalization.fragments.base import VerbFragment
from krrood.entity_query_language.verbalization.rule_engine import RuleEngine
from krrood.entity_query_language.verbalization.rule_verbalizer import RuleVerbalizer
from krrood.entity_query_language.verbalization.rules import ALL_RULES
from krrood.entity_query_language.verbalization.utils import _str


@dataclass
class EQLVerbalizer:
    """
    Coordinator that maps an EQL expression tree to a VerbFragment tree.

    Dispatches via a RuleEngine of VerbalizationRule classes (see verbalization/rules/).
    Each rule declares its guard in `applies()` and its rendering in `transform()`.
    More-specific subclasses are tried before their parents (depth-first priority).

    Use verbalize_expression() for the simple string API, or build a
    VerbalizationPipeline to choose format and colour scheme.
    """

    _chain: ChainVerbalizer = field(init=False, repr=False)
    _entity: EntityVerbalizer = field(init=False, repr=False)
    _rule: RuleVerbalizer = field(init=False, repr=False)
    _engine: RuleEngine = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rule = RuleVerbalizer(delegate=self)
        self._entity = EntityVerbalizer(delegate=self)
        self._chain = ChainVerbalizer(
            delegate=self,
            entity_inline_fn=self._entity.as_inline_noun,
        )
        self._engine = RuleEngine(ALL_RULES)

    def build(
        self,
        expr: SymbolicExpression,
        ctx: Optional[VerbalizationContext] = None,
    ) -> VerbFragment:
        if ctx is None:
            ctx = VerbalizationContext.from_expression(expr)
        return self._engine.build(expr, ctx, self)

    def verbalize(
        self,
        expr: SymbolicExpression,
        ctx: Optional[VerbalizationContext] = None,
    ) -> str:
        return _str(self.build(expr, ctx))


_default_verbalizer = EQLVerbalizer()


def verbalize_expression(expr) -> str:
    """Verbalize any EQL expression into a human-readable English phrase (plain text)."""
    from krrood.entity_query_language.query.query import Query
    if isinstance(expr, Query):
        expr.build()
    return _default_verbalizer.verbalize(expr)
