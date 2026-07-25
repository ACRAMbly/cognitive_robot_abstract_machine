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

import krrood
from krrood.entity_query_language.factories import variable
from krrood.entity_query_language.predicate import Predicate
from krrood.entity_query_language.testing.surface_verification import (
    first_order_form,
    OverriddenOperand,
    placeholder_operands,
    SymbolicSurfaceSnapshot,
)
from krrood.entity_query_language.verbalization.pipeline import verbalize_expression
from krrood.entity_query_language.verbalization.vocabulary.english import Prepositions
from krrood.entity_query_language.verbalization.vocabulary.parts_of_speech import (
    Adjective,
    clause,
    Copula,
    Noun,
)

# %% mimic domain


@dataclass
class Igniter:
    """
    A stand-in operand type whose class name is unremarkable.
    """


@dataclass(eq=False)
class Kindled(Predicate):
    """
    A two-operand predicate used to exercise the first-order/value-using pairing and the
    snapshot's operand-override mechanism -- both fields appear in the fragment, so an
    override's effect (or the lack of one) is directly observable in the rendered
    sentence rather than merely asserted.
    """

    fuel: Igniter
    catalyst: object

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(
            Noun(fields["fuel"]),
            Copula(),
            Adjective("lit"),
            Prepositions.WITH,
            Noun(fields["catalyst"]),
        )


@dataclass
class Fastener(ABC):
    """
    A stand-in abstract operand type with a small, nameable family of concrete
    alternatives.
    """

    @abstractmethod
    def grip(self) -> float: ...


@dataclass
class Bolt(Fastener):
    def grip(self) -> float:
        return 0.0


@dataclass
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


# %% placeholder_operands and first_order_form take nothing but the class itself


def test_placeholder_operands_uses_the_field_type_by_default():
    operands = placeholder_operands(Kindled)
    assert operands["fuel"]._type_ is Igniter
    assert operands["catalyst"]._type_ is object


def test_first_order_form_verbalizes_from_declared_field_types():
    assert first_order_form(Kindled) == "an Igniter is lit with a catalyst"


def test_first_order_form_expands_an_abstract_declared_field_type():
    """
    The first-order form threads an abstract field's placeholder variable through the
    same operand-naming resolution as any bound variable, so it is expanded into its
    concrete alternatives exactly like a real query would be.
    """
    assert first_order_form(Fastened) == "a Bolt or a Screw is secure"


# %% first-order form and value-using form are the same pipeline


def test_first_order_form_and_value_using_form_agree_when_types_match():
    """
    The value-using form (`verbalize_expression` on a real, bound instance) and the
    first-order form differ only in where the operand came from -- a real referent vs.

    a placeholder built from the declared field type -- not in how it is named, since a
    bound instance's type is always concrete and resolves through the very same
    `referring.operand_head_noun` call. Both operands here are equivalent, still-unbound
    variables (an `Igniter` and an untyped `object`), matching what `first_order_form`
    itself builds, so the two renderings agree exactly.
    """
    bound_instance = Kindled(variable(Igniter, []), variable(object, []))
    assert first_order_form(Kindled) == verbalize_expression(bound_instance)


# %% SymbolicSurfaceSnapshot layers operand_overrides on top -- a snapshot-testing concern,
# %% not a first_order_form one, since a value-agnostic rendering needs nothing external


def test_snapshot_placeholder_operands_lets_a_registered_override_overwrite_a_field():
    snapshot = SymbolicSurfaceSnapshot(
        package=krrood,
        surfaces=(),
        operand_overrides={Kindled: [OverriddenOperand("catalyst", "ash")]},
    )
    assert snapshot.placeholder_operands(Kindled)["catalyst"] == "ash"


def test_snapshot_rendered_surface_respects_a_registered_override():
    """
    The overridden value appears in the rendered sentence in place of the un-overridden
    default's field-name fallback ("a catalyst") -- the whole point of an override, made
    directly observable rather than merely asserted.
    """
    snapshot = SymbolicSurfaceSnapshot(
        package=krrood,
        surfaces=(),
        operand_overrides={Kindled: [OverriddenOperand("catalyst", "ash")]},
    )
    assert snapshot.rendered_surface(Kindled) == "an Igniter is lit with 'ash'"
    assert snapshot.rendered_surface(Kindled) != first_order_form(Kindled)
