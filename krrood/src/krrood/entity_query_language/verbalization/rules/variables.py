from __future__ import annotations

import re
from typing import TYPE_CHECKING

from krrood.entity_query_language.core.variable import ExternallySetVariable, InstantiatedVariable, Literal, Variable
from krrood.entity_query_language.predicate import Verbalizable
from krrood.entity_query_language.verbalization.chain_utils import verbalize_plural
from krrood.entity_query_language.verbalization.context import ArticleSelection
from krrood.entity_query_language.verbalization.fragments.base import PhraseFragment, RoleFragment, VerbFragment, WordFragment
from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule
from krrood.entity_query_language.verbalization.utils import _apply_binding_aliases, _str, inflect_engine
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Articles, Conjunctions, Copulas, Keywords, Prepositions,
)

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


def _word(text: str) -> WordFragment:
    return WordFragment(text=text)


def _phrase(*parts: VerbFragment, sep: str = " ") -> PhraseFragment:
    return PhraseFragment(parts=list(parts), separator=sep)


def _role(text: str, role: SemanticRole, source_ref=None) -> RoleFragment:
    return RoleFragment(text=text, role=role, source_ref=source_ref)


class VariableRule(VerbalizationRule):
    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Variable)

    @classmethod
    def transform(cls, expr: Variable, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        article, label = ctx.noun_for_parts(expr)
        label_frag = RoleFragment.for_variable(label, expr)
        if article == ArticleSelection.NONE:
            return label_frag
        if article == ArticleSelection.DEFINITE:
            return _phrase(Articles.THE.as_fragment(), label_frag)
        return _phrase(Articles.indefinite(label), label_frag)


class LiteralRule(VariableRule):
    """Literal is a subclass of Variable; renders as a plain semantic-role fragment."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, Literal)

    @classmethod
    def transform(cls, expr: Literal, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return _role(ctx.type_name_of_value(expr._value_), SemanticRole.LITERAL)


class ExternallySetVariableRule(VerbalizationRule):
    """ExternallySetVariable is a sibling of Variable (both inherit CanHaveDomainSource)."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, ExternallySetVariable)

    @classmethod
    def transform(cls, expr: ExternallySetVariable, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        type_name = expr._type_.__name__ if getattr(expr, "_type_", None) else "variable"
        return _phrase(Articles.indefinite(type_name), _role(type_name, SemanticRole.VARIABLE))


class InstantiatedVariableRule(VerbalizationRule):
    """InstantiatedVariable natural form: 'a TypeName where the field of the TypeName is …'"""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, InstantiatedVariable)

    @classmethod
    def transform(cls, expr: InstantiatedVariable, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        return _verbalize_instantiated_natural(expr, ctx, delegate)


class InstantiatedVerbalizableRule(InstantiatedVariableRule):
    """InstantiatedVariable whose type provides a _verbalization_template_(); uses it directly."""

    @classmethod
    def applies(cls, expr, ctx: VerbalizationContext) -> bool:
        return isinstance(expr, InstantiatedVariable) and _has_verbalization_template(expr)

    @classmethod
    def transform(cls, expr: InstantiatedVariable, ctx: VerbalizationContext, delegate: EQLVerbalizer) -> VerbFragment:
        template = expr._type_._verbalization_template_()
        kwargs = {name: delegate.verbalize(child, ctx) for name, child in expr._child_vars_.items()}
        return _word(template.format(**kwargs))


# ── Module-level helpers ───────────────────────────────────────────────────────

def _has_verbalization_template(expr: InstantiatedVariable) -> bool:
    try:
        if isinstance(expr._type_, type) and issubclass(expr._type_, Verbalizable):
            expr._type_._verbalization_template_()
            return True
    except NotImplementedError:
        pass
    return False


def _verbalize_instantiated_natural(
    expr: InstantiatedVariable,
    ctx: VerbalizationContext,
    delegate: EQLVerbalizer,
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
            plural_value = _str(verbalize_plural(child_expr, ctx, delegate.build))
            binding_parts.append(f"{field_ref} {_are} {plural_value}")
        else:
            value_text = delegate.verbalize(child_expr, ctx)
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
