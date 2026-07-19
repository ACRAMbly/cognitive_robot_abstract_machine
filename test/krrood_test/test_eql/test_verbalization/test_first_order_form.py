"""
Tests for the first-order (value-agnostic) rendering `surface_verification.py` provides,
and how it pairs with the ordinary value-using form a bound expression already renders
through `verbalize_expression`.

The first-order form names every operand from its declared field type alone (no
constructed instance or bound literal in hand); the value-using form names a real, bound
expression's operands the ordinary way. Both go through the very same rendering pipeline
and the very same operand-naming resolution (`referring.operand_head_noun`), so an
abstract declared field type is expanded into its concrete alternatives identically in
either form.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from krrood.entity_query_language.factories import variable
from krrood.entity_query_language.predicate import Predicate
from krrood.entity_query_language.verbalization.pipeline import verbalize_expression
from krrood.entity_query_language.verbalization.surface_verification import (
    first_order_form,
    OverriddenOperand,
    placeholder_operands,
)
from krrood.entity_query_language.verbalization.vocabulary.parts_of_speech import (
    Adjective,
    clause,
    Copula,
    Noun,
)

# %% mimic domain


class Igniter:
    """
    A stand-in operand type whose class name is unremarkable.
    """


@dataclass(eq=False)
class Kindled(Predicate):
    """
    A two-operand predicate used to exercise the first-order/value-using pairing; only
    `fuel` appears in the fragment, so `catalyst` proves an override without affecting
    wording.
    """

    fuel: Igniter
    catalyst: object

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(Noun(fields["fuel"]), Copula(), Adjective("lit"))


class Fastener(ABC):
    """
    A stand-in abstract operand type with a small, nameable family of concrete
    alternatives.
    """

    @abstractmethod
    def grip(self) -> float: ...


class Bolt(Fastener):
    def grip(self) -> float:
        return 0.0


class Screw(Fastener):
    def grip(self) -> float:
        return 0.0


@dataclass(eq=False)
class Fastened(Predicate):
    """
    A single-operand predicate whose field is typed with an abstract base.
    """

    item: Fastener

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(Noun(fields["item"]), Copula(), Adjective("secure"))


# %% placeholder_operands


def test_placeholder_operands_uses_the_field_type_by_default():
    operands = placeholder_operands(Kindled)
    assert operands["fuel"]._type_ is Igniter
    assert operands["catalyst"]._type_ is object


def test_placeholder_operands_uses_a_registered_override():
    operands = placeholder_operands(Kindled, [OverriddenOperand("catalyst", "ash")])
    assert operands["catalyst"] == "ash"


# %% first_order_form (the value-agnostic form)


def test_first_order_form_verbalizes_from_declared_field_types():
    assert first_order_form(Kindled) == "an Igniter is lit"


def test_first_order_form_respects_a_registered_override():
    """
    An overridden field's concrete value participates in the sentence read only through
    the fragment the class chose to build -- here `catalyst` never appears in the
    wording, so the override changes nothing observable, matching the un-overridden
    rendering.
    """
    assert first_order_form(
        Kindled, [OverriddenOperand("catalyst", "ash")]
    ) == first_order_form(Kindled)


def test_first_order_form_expands_an_abstract_declared_field_type():
    """
    The first-order form threads an abstract field's placeholder variable through the
    same operand-naming resolution as any bound variable, so it is expanded into its
    concrete alternatives exactly like a real query would be.
    """
    assert first_order_form(Fastened) == "a Bolt or Screw is secure"


# %% first-order form and value-using form are the same pipeline


def test_first_order_form_and_value_using_form_agree_when_types_match():
    """
    The value-using form (`verbalize_expression` on a real, bound instance) and the
    first-order form differ only in where the operand came from -- a real referent vs.

    a placeholder built from the declared field type -- not in how it is named, since a
    bound instance's type is always concrete and resolves through the very same
    `referring.operand_head_noun` call.
    """
    bound_instance = Kindled(variable(Igniter, []), object())
    assert first_order_form(Kindled) == verbalize_expression(bound_instance)
