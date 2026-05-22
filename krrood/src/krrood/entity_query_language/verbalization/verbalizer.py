from __future__ import annotations

import datetime as _dt
import re
from dataclasses import dataclass, field
from typing import Callable, Optional

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.core.mapped_variable import Attribute, Call, FlatVariable, Index, MappedVariable
from krrood.entity_query_language.core.variable import ExternallySetVariable, InstantiatedVariable, Literal, Variable
from krrood.entity_query_language.operators.aggregators import (
    Average, Count, CountAll, Max, Min, Mode, MultiMode, Sum,
)
from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.operators.core_logical_operators import AND, OR, Not
from krrood.entity_query_language.operators.logical_quantifiers import Exists, ForAll
from krrood.entity_query_language.predicate import Verbalizable
from krrood.entity_query_language.query.operations import GroupedBy, Having, OrderedBy, Where
from krrood.entity_query_language.query.quantifiers import An, ResultQuantifier, The
from krrood.entity_query_language.query.query import Entity, SetOf, Query
from krrood.entity_query_language.verbalization.chain_utils import verbalize_plural
from krrood.entity_query_language.verbalization.chain_verbalizer import ChainVerbalizer
from krrood.entity_query_language.verbalization.context import ArticleSelection, VerbalizationContext
from krrood.entity_query_language.verbalization.entity_verbalizer import EntityVerbalizer
from krrood.entity_query_language.verbalization.fragments.base import (
    BlockFragment,
    PhraseFragment,
    RoleFragment,
    VerbFragment,
    WordFragment,
)
from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef
from krrood.entity_query_language.verbalization.rule_verbalizer import RuleVerbalizer
from krrood.entity_query_language.verbalization.utils import (
    _apply_binding_aliases,
    _camel_to_words,
    _ensure_plural,
    _str,
    inflect_engine,
)
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Aggregations,
    Articles,
    Conjunctions,
    Copulas,
    ExistentialPhrase,
    FallbackNouns,
    GroupKeyPhrases,
    Keywords,
    Logicals,
    Operators,
    Prepositions,
    SortDirections,
)

# ── Small fragment helpers ──────────────────────────────────────────────────────

def _word(text: str) -> WordFragment:
    return WordFragment(text=text)


def _role(
    text: str,
    role: SemanticRole,
    source_ref: Optional[SourceRef] = None,
) -> RoleFragment:
    return RoleFragment(text=text, role=role, source_ref=source_ref)


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


def _oxford_and(fragments: list[VerbFragment], conjunction: WordFragment) -> VerbFragment:
    """Join with Oxford comma: a, b, and c."""
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



@dataclass
class EQLVerbalizer:
    """
    Coordinator that maps an EQL expression tree to a VerbFragment tree.

    Dispatches to focused subsystem verbalizers (ChainVerbalizer, EntityVerbalizer,
    RuleVerbalizer) and handles the remaining short-form expression types directly.

    Use verbalize_expression() for the simple string API, or build a
    VerbalizationPipeline to choose format and colour scheme.
    """

    _chain: ChainVerbalizer = field(init=False, repr=False)
    _entity: EntityVerbalizer = field(init=False, repr=False)
    _rule: RuleVerbalizer = field(init=False, repr=False)
    _dispatch: dict[type, Callable] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rule = RuleVerbalizer(delegate=self)
        self._entity = EntityVerbalizer(delegate=self)
        self._chain = ChainVerbalizer(
            delegate=self,
            entity_inline_fn=self._entity.as_inline_noun,
        )
        self._dispatch = {
            Variable:                self._v_Variable_,
            ExternallySetVariable:   self._v_ExternallySetVariable_,
            Literal:                 self._v_Literal_,
            Attribute:            self._chain.verbalize_mapped,
            Index:                self._chain.verbalize_mapped,
            Call:                 self._chain.verbalize_mapped,
            FlatVariable:         self._chain.verbalize_flat,
            InstantiatedVariable: self._v_InstantiatedVariable_,
            AND:                  self._v_AND_,
            OR:                   self._v_OR_,
            Not:                  self._v_Not_,
            ForAll:               self._v_ForAll_,
            Exists:               self._v_Exists_,
            Comparator:           self._v_Comparator_,
            Count:                self._v_Count_,
            CountAll:             self._v_CountAll_,
            Sum:                  self._v_Sum_,
            Average:              self._v_Average_,
            Max:                  self._v_Max_,
            Min:                  self._v_Min_,
            Mode:                 self._v_Mode_,
            MultiMode:            self._v_MultiMode_,
            Entity:               self._entity.verbalize_query,
            SetOf:                self._entity.verbalize_set_of,
            An:                   self._v_An_,
            The:                  self._v_The_,
            ResultQuantifier:     self._v_ResultQuantifier_,
            Where:                self._v_Where_,
            Having:               self._v_Having_,
            GroupedBy:            self._v_GroupedBy_,
            OrderedBy:            self._v_OrderedBy_,
        }

    # ── Dispatcher ─────────────────────────────────────────────────────────────

    def build(
        self,
        expr: SymbolicExpression,
        ctx: Optional[VerbalizationContext] = None,
    ) -> VerbFragment:
        if ctx is None:
            ctx = VerbalizationContext.from_expression(expr)
        return self._dispatch.get(type(expr), self._v_default_)(expr, ctx)

    def verbalize(
        self,
        expr: SymbolicExpression,
        ctx: Optional[VerbalizationContext] = None,
    ) -> str:
        return _str(self.build(expr, ctx))

    # ── Leaves ─────────────────────────────────────────────────────────────────

    def _v_Variable_(self, expr: Variable, ctx: VerbalizationContext) -> VerbFragment:
        article, label = ctx.noun_for_parts(expr)
        label_frag = RoleFragment.for_variable(label, expr)
        if article == ArticleSelection.NONE:
            return label_frag
        if article == ArticleSelection.DEFINITE:
            return _phrase(Articles.THE.as_fragment(), label_frag)
        return _phrase(Articles.indefinite(label), label_frag)

    def _v_Literal_(self, expr: Literal, ctx: VerbalizationContext) -> VerbFragment:
        return _role(ctx.type_name_of_value(expr._value_), SemanticRole.LITERAL)

    def _v_ExternallySetVariable_(self, expr: ExternallySetVariable, ctx: VerbalizationContext) -> VerbFragment:
        type_name = expr._type_.__name__ if getattr(expr, "_type_", None) else "variable"
        return _phrase(Articles.indefinite(type_name), _role(type_name, SemanticRole.VARIABLE))

    # ── Instantiated variables (predicates / inference) ────────────────────────

    def _v_InstantiatedVariable_(
        self, expr: InstantiatedVariable, ctx: VerbalizationContext
    ) -> VerbFragment:
        try:
            if isinstance(expr._type_, type) and issubclass(expr._type_, Verbalizable):
                template = expr._type_._verbalization_template_()
                return self._verbalize_template_(expr, ctx, template)
        except NotImplementedError:
            pass
        return self._verbalize_instantiated_natural_(expr, ctx)

    def _verbalize_template_(
        self, expr: InstantiatedVariable, ctx: VerbalizationContext, template: str
    ) -> VerbFragment:
        kwargs = {name: self.verbalize(child, ctx) for name, child in expr._child_vars_.items()}
        return _word(template.format(**kwargs))

    def _verbalize_predicate_no_template_(
        self, expr: InstantiatedVariable, ctx: VerbalizationContext
    ) -> VerbFragment:
        type_name = getattr(expr._type_, "__name__", str(expr._type_))
        if len(expr._child_vars_) == 2:
            items = list(expr._child_vars_.items())
            left, right = items[0][1], items[1][1]
            predicate_text = _camel_to_words(type_name)
            return _phrase(self.build(left, ctx), _word(predicate_text), self.build(right, ctx))
        if expr._child_vars_:
            args_str = ", ".join(
                f"{name}={self.verbalize(child, ctx)}" for name, child in expr._child_vars_.items()
            )
            return _phrase(
                Articles.indefinite(type_name),
                RoleFragment.for_variable(type_name, expr),
                _word(f"({args_str})"),
            )
        return _phrase(Articles.indefinite(type_name), RoleFragment.for_variable(type_name, expr))

    def _verbalize_instantiated_natural_(
        self, expr: InstantiatedVariable, ctx: VerbalizationContext
    ) -> VerbFragment:
        type_name = getattr(expr._type_, "__name__", str(expr._type_))

        if expr._id_ in ctx.seen:
            return _phrase(Articles.THE.as_fragment(), _role(ctx.seen[expr._id_], SemanticRole.VARIABLE))
        ctx.seen[expr._id_] = type_name

        ctx.push_constraint_frame()

        _the = Articles.THE.text
        _is = Copulas.IS.text
        _are = Copulas.ARE.text
        _where = Keywords.WHERE.text
        _such_that = Keywords.SUCH_THAT.text
        _and = Conjunctions.AND.text
        _of = Prepositions.OF.text

        binding_parts: list[str] = []
        binding_alias_map: dict[str, str] = {}
        for field_name, child_expr in expr._child_vars_.items():
            field_ref = f"{_the} {field_name} {_of} {_the} {type_name}"
            if inflect_engine.singular_noun(field_name):
                plural_value = _str(verbalize_plural(child_expr, ctx, self.build))
                binding_parts.append(f"{field_ref} {_are} {plural_value}")
            else:
                value_text = self.verbalize(child_expr, ctx)
                binding_parts.append(f"{field_ref} {_is} {value_text}")
                _the_pat = re.escape(_the)
                definite_value = re.sub(r"\b(a|an) ([A-Z])", rf"{_the} \2", value_text)
                if re.search(rf"\b{_the_pat} [A-Z]", definite_value) and definite_value not in binding_alias_map:
                    binding_alias_map[definite_value] = field_ref

        constraints = ctx.pop_constraint_frame()
        ctx.binding_aliases.update(binding_alias_map)
        if constraints and binding_alias_map:
            constraints = [_apply_binding_aliases(c, binding_alias_map) for c in constraints]

        result_parts: list[VerbFragment] = [
            _phrase(Articles.indefinite(type_name), RoleFragment.for_variable(type_name, expr))
        ]
        if binding_parts:
            result_parts.append(_word(f", {_where} " + f" {_and} ".join(binding_parts)))
        if constraints:
            result_parts.append(_word(f", {_such_that} " + f" {_and} ".join(constraints)))
        return PhraseFragment(parts=result_parts, separator="")

    # ── Logical operators ──────────────────────────────────────────────────────

    def _v_AND_(self, expr: AND, ctx: VerbalizationContext) -> VerbFragment:
        parts = [self.build(c, ctx) for c in ctx.flatten_same_type(expr, AND)]
        if len(parts) == 1:
            return parts[0]
        return _oxford_and(parts, Conjunctions.AND.as_fragment())

    def _v_OR_(self, expr: OR, ctx: VerbalizationContext) -> VerbFragment:
        parts = [self.build(c, ctx) for c in ctx.flatten_same_type(expr, OR)]
        if len(parts) == 1:
            return parts[0]
        head_with_comma = PhraseFragment(
            parts=[_join_with(parts[:-1], ", "), _word(",")], separator=""
        )
        return _phrase(Logicals.EITHER.as_fragment(), head_with_comma, Conjunctions.OR.as_fragment(), parts[-1])

    def _v_Not_(self, expr: Not, ctx: VerbalizationContext) -> VerbFragment:
        child = expr._child_
        if isinstance(child, Comparator):
            left = self.build(child.left, ctx)
            right = self.build(child.right, ctx)
            is_temporal = self._chain.is_temporal(child.left) or self._chain.is_temporal(child.right)
            try:
                op_frag = Operators.from_callable(child.operation).select(
                    negated=True, compact=ctx.compact_predicates, temporal=is_temporal
                ).as_fragment()
            except KeyError:
                op_frag = RoleFragment.for_operator(f"not {child._name_}")
            return _phrase(left, op_frag, right)
        if isinstance(child, MappedVariable):
            from krrood.entity_query_language.verbalization.chain_utils import walk_chain
            chain, _ = walk_chain(child)
            if isinstance(chain[-1], Attribute) and chain[-1]._type_ is bool:
                return self._chain.verbalize_mapped_negated(child, ctx)
        return _phrase(Logicals.NOT.as_fragment(), PhraseFragment(parts=[_word("("), self.build(child, ctx), _word(")")], separator=""))

    # ── Quantifiers ────────────────────────────────────────────────────────────

    def _v_ForAll_(self, expr: ForAll, ctx: VerbalizationContext) -> VerbFragment:
        var_frag = verbalize_plural(expr.variable, ctx, self.build)
        cond_frag = self.build(expr.condition, ctx)
        return _phrase(Logicals.FOR_ALL.as_fragment(), var_frag, _word(","), cond_frag)

    def _v_Exists_(self, expr: Exists, ctx: VerbalizationContext) -> VerbFragment:
        var_frag = self.build(expr.variable, ctx)
        cond_frag = self.build(expr.condition, ctx)
        return _phrase(
            Logicals.THERE_EXISTS.as_fragment(),
            var_frag,
            Keywords.SUCH_THAT.as_fragment(),
            cond_frag,
        )

    # ── Comparator ─────────────────────────────────────────────────────────────

    def _v_Comparator_(self, expr: Comparator, ctx: VerbalizationContext) -> VerbFragment:
        left = self.build(expr.left, ctx)
        right = self.build(expr.right, ctx)
        is_temporal = self._chain.is_temporal(expr.left) or self._chain.is_temporal(expr.right)
        try:
            op_frag = Operators.from_callable(expr.operation).select(
                compact=ctx.compact_predicates, temporal=is_temporal
            ).as_fragment()
        except KeyError:
            op_frag = RoleFragment.for_operator(expr._name_)
        return _phrase(left, op_frag, right)

    # ── Aggregators ────────────────────────────────────────────────────────────

    def _v_Count_(self, expr: Count, ctx: VerbalizationContext) -> VerbFragment:
        return self._verbalize_aggregator_(expr, ctx, Aggregations.COUNT)

    def _v_CountAll_(self, expr: CountAll, ctx: VerbalizationContext) -> VerbFragment:
        return Aggregations.COUNT_ALL.as_fragment()

    def _v_Sum_(self, expr: Sum, ctx: VerbalizationContext) -> VerbFragment:
        return self._verbalize_aggregator_(expr, ctx, Aggregations.SUM)

    def _v_Average_(self, expr: Average, ctx: VerbalizationContext) -> VerbFragment:
        return self._verbalize_aggregator_(expr, ctx, Aggregations.AVERAGE)

    def _v_Max_(self, expr: Max, ctx: VerbalizationContext) -> VerbFragment:
        return self._verbalize_aggregator_(expr, ctx, Aggregations.MAX)

    def _v_Min_(self, expr: Min, ctx: VerbalizationContext) -> VerbFragment:
        return self._verbalize_aggregator_(expr, ctx, Aggregations.MIN)

    def _v_Mode_(self, expr: Mode, ctx: VerbalizationContext) -> VerbFragment:
        return self._verbalize_aggregator_(expr, ctx, Aggregations.MODE)

    def _v_MultiMode_(self, expr: MultiMode, ctx: VerbalizationContext) -> VerbFragment:
        return self._verbalize_aggregator_(expr, ctx, Aggregations.MULTI_MODE)

    def _verbalize_aggregator_(self, expr, ctx: VerbalizationContext, agg: Aggregations) -> VerbFragment:
        child_frag = verbalize_plural(expr._child_, ctx, self.build)
        agg_frag = agg.as_fragment()
        if expr._id_ in ctx.seen:
            return _phrase(Articles.THE.as_fragment(), agg_frag, child_frag)
        ctx.seen[expr._id_] = _str(_phrase(agg_frag, child_frag))
        return _phrase(agg_frag, child_frag)

    # ── Result quantifiers (transparent wrappers) ──────────────────────────────

    def _v_An_(self, expr: An, ctx: VerbalizationContext) -> VerbFragment:
        return self.build(expr._child_, ctx)

    def _v_The_(self, expr: The, ctx: VerbalizationContext) -> VerbFragment:
        return self.build(expr._child_, ctx)

    def _v_ResultQuantifier_(self, expr: ResultQuantifier, ctx: VerbalizationContext) -> VerbFragment:
        return self.build(expr._child_, ctx)

    # ── Filter wrappers ────────────────────────────────────────────────────────

    def _v_Where_(self, expr: Where, ctx: VerbalizationContext) -> VerbFragment:
        return self.build(expr.condition, ctx)

    def _v_Having_(self, expr: Having, ctx: VerbalizationContext) -> VerbFragment:
        return self.build(expr.condition, ctx)

    def _v_GroupedBy_(self, expr: GroupedBy, ctx: VerbalizationContext) -> VerbFragment:
        if expr.variables_to_group_by:
            groups = [self.verbalize(v, ctx) for v in expr.variables_to_group_by]
            return _phrase(Keywords.GROUPED_BY.as_fragment(), _word(", ".join(groups)))
        return Keywords.GROUPED.as_fragment()

    def _v_OrderedBy_(self, expr: OrderedBy, ctx: VerbalizationContext) -> VerbFragment:
        direction = SortDirections.DESCENDING.text if expr.descending else SortDirections.ASCENDING.text
        return _phrase(
            Keywords.ORDERED_BY.as_fragment(),
            _word(f"{self.verbalize(expr.variable, ctx)} ({direction})"),
        )

    # ── Fallback ───────────────────────────────────────────────────────────────

    def _v_default_(self, expr: SymbolicExpression, ctx: VerbalizationContext) -> VerbFragment:
        return _word(expr._name_)


_default_verbalizer = EQLVerbalizer()


def verbalize_expression(expr) -> str:
    """Verbalize any EQL expression into a human-readable English phrase (plain text)."""
    if isinstance(expr, Query):
        expr.build()
    return _default_verbalizer.verbalize(expr)
