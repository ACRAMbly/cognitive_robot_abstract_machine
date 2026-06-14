from __future__ import annotations

from krrood.entity_query_language.core.mapped_variable import FlatVariable
from krrood.entity_query_language.core.variable import (
    ExternallySetVariable,
    Literal,
    Variable,
)
from krrood.entity_query_language.verbalization.fragments.base import (
    Fragment,
    NounPhrase,
    RoleFragment,
)
from krrood.entity_query_language.verbalization.fragments.features import (
    Definiteness,
    Number,
)
from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.grammar.framework.phrase_rule import (
    PhraseRule,
    RuleContext,
)
from krrood.entity_query_language.verbalization.vocabulary.english import FallbackNouns


class VariableRule(PhraseRule):
    """*"a/an Robot"* (first mention), *"the Robot"* (subsequent), or *"Robot N"* (numbered).

    >>> verbalize_expression(variable(Robot, []))
    'a Robot'
    """

    construct = Variable
    name = "variable"

    def build(self, node: Variable, context: RuleContext) -> Fragment:
        if context.number is Number.PLURAL:
            return self._plural(node, context)
        noun_form = context.refer.noun_for_parts(node)
        return NounPhrase(
            head=RoleFragment.for_variable(noun_form.label, node),
            definiteness=noun_form.definiteness,
            referent_id=node._id_,
        )

    @staticmethod
    def _plural(node: Variable, context: RuleContext) -> Fragment:
        """Bare plural variable noun phrase (*"Robots"*); the determiner phase drops the article and
        the morphology pass inflects the head.

        A numbered label (*"Robot 2"*) is surface-final — kept singular and bare; a plain type
        name is a plural indefinite noun phrase (the concord table renders it bare-then-pluralised).
        """
        numbered = context.refer.numbered_label(node)
        return NounPhrase(
            head=RoleFragment.for_variable(numbered.text, node),
            number=Number.SINGULAR if numbered.is_numbered else Number.PLURAL,
            definiteness=(
                Definiteness.BARE if numbered.is_numbered else Definiteness.INDEFINITE
            ),
            referent_id=node._id_,
        )


class LiteralRule(PhraseRule):
    """A literal value (e.g. ``42``, ``"hello"``, ``True``)."""

    construct = Literal
    name = "literal"

    def build(self, node: Literal, context: RuleContext) -> Fragment:
        return RoleFragment(
            text=context.services.type_name_of_value(node._value_),
            role=SemanticRole.LITERAL,
        )


class ExternalVariableRule(PhraseRule):
    """*"a/an TypeName"* for an opaque externally-set variable (no coreference)."""

    construct = ExternallySetVariable
    name = "external-variable"

    def build(self, node: ExternallySetVariable, context: RuleContext) -> Fragment:
        type_name = (
            node._type_.__name__
            if getattr(node, "_type_", None)
            else FallbackNouns.VARIABLE.text
        )
        return NounPhrase(head=RoleFragment(text=type_name, role=SemanticRole.VARIABLE))


class FlatVariableRule(PhraseRule):
    """A transparent SetOf wrapper → unwrap to its child (forwarding the requested number)."""

    construct = FlatVariable
    name = "flat-variable"

    def build(self, node: FlatVariable, context: RuleContext) -> Fragment:
        return context.child(node._child_, number=context.number)
