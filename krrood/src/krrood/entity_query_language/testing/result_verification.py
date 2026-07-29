"""
Exhaustive verbalization-result verification for any package, and the first-order
(value-agnostic) rendering it builds on.

Point a :class:`VerbalizationResultsOfPackage` at a package and a committed list of
:class:`VerbalizationResult` entries. Its three ``assert_*`` methods, used as the bodies of three
tests, check that every concrete symbolic callable the package defines (1) implements its own
verbalization fragment, (2) has a declared result, and (3) renders exactly its declared sentence —
so a new predicate or function, or a changed shared result builder, cannot slip through unreviewed.

The same three-line test works for any package that defines
:class:`~krrood.entity_query_language.predicate.SymbolicCallable` subclasses (krrood itself,
``semantic_digital_twin``, ``coraplex``, …): the discovery, placeholder operands, and rendering all
live here.

The snapshot's per-class rendering is itself a reusable capability, not a test-only trick:
:func:`first_order_form` verbalizes a symbolic callable *value-agnostically* — from its declared
field types alone, with no constructed instance or bound literal in hand — the sibling of the
ordinary *value-using* form :func:`~…pipeline.verbalize_expression` already produces for a real,
bound expression. It takes nothing but the class itself, needs nothing external, and never
consults a class's own :meth:`~krrood.entity_query_language.predicate.SymbolicCallable.
_example_operands_` override — that hook exists purely to make a committed :class:`VerbalizationResult`
snapshot read more realistically and is consulted only by :class:`VerbalizationResultsOfPackage`,
never by ordinary rendering. That snapshot also layers its own
:attr:`~VerbalizationResultsOfPackage.operand_overrides` on top — an example value scoped to one
snapshot (a test-only mimic class, say) rather than a class-level truth.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from dataclasses import fields as dataclass_fields
from types import ModuleType

from typing_extensions import Any, Dict, Sequence, Tuple, Type

from krrood.class_diagrams.class_diagram import WrappedClass
from krrood.class_diagrams.utils import class_implements_own_method
from krrood.class_diagrams.wrapped_field import WrappedField
from krrood.entity_query_language.factories import variable
from krrood.entity_query_language.predicate import SymbolicCallable, Verbalizable
from krrood.entity_query_language.verbalization.pipeline import verbalize_expression
from krrood.ormatic.utils import classes_of_package
from krrood.utils import module_and_class_name


def placeholder_operands(cls: Type[SymbolicCallable]) -> Dict[str, Any]:
    """
    One placeholder operand per init dataclass field, from the field's declared type
    endpoint alone — a fresh variable of that type (``object`` when the endpoint is not
    a plain class), so the result reads the operand as *"a <TypeName>"*.

    :param cls: The symbolic callable to build operands for.
    :return: The operand to pass for each init field, keyed by field name.

    >>> operands = placeholder_operands(IsReachable)
    >>> operands["location"]._type_, operands["body"]._type_
    (<class 'object'>, <class 'object'>)
    """
    wrapped_class = WrappedClass(clazz=cls)
    operands: Dict[str, Any] = {}
    for field_ in dataclass_fields(cls):
        if not field_.init:
            continue
        endpoint = WrappedField(wrapped_class, field_).type_endpoint
        placeholder_type = (
            endpoint if isinstance(endpoint, type) and endpoint is not Any else object
        )
        operands[field_.name] = variable(placeholder_type, [])
    return operands


def first_order_form(cls: Type[SymbolicCallable]) -> str:
    """
    Verbalize *cls* value-agnostically — the *first-order form*: every operand named
    from its declared field type alone (:func:`placeholder_operands`), with no
    constructed instance or bound literal in hand.

    :param cls: The symbolic callable to render.
    :return: The sentence *cls* renders with placeholder operands.

    >>> first_order_form(IsReachable)
    'a location is reachable for a body'
    """
    return verbalize_expression(cls(**placeholder_operands(cls)))


@dataclass(frozen=True)
class VerbalizationResult:
    """
    One symbolic callable and the sentence it verbalizes to — a committed snapshot
    entry.
    """

    callable_class: Type[SymbolicCallable]
    """
    The symbolic function or predicate whose result this records.
    """

    sentence: str
    """
    The approved sentence it renders with the snapshot's placeholder operands.
    """


@dataclass(frozen=True)
class VerbalizationResultsOfPackage:
    """
    Exhaustive verbalization-result check for the symbolic callables a package defines.

    Discovers every concrete :class:`~krrood.entity_query_language.predicate.SymbolicCallable` in
    :attr:`package`, renders each with placeholder operands, and checks the rendering against the
    committed :attr:`results`. Use the three ``assert_*`` methods as the bodies of three tests.
    """

    package: ModuleType
    """
    The package whose symbolic callables are discovered and checked.
    """

    results: Sequence[VerbalizationResult]
    """
    The committed expected results, one per covered class.
    """

    operand_overrides: Dict[Type[SymbolicCallable], Dict[str, Any]] = field(
        default_factory=dict
    )
    """
    Example values scoped to this snapshot alone, keyed by the class and then the field
    name, consulted after each class's own ``_example_operands_`` -- for an example
    specific to this snapshot (a test-only mimic class, say) rather than a class-level
    truth.
    """

    def discovered_callables(self) -> Tuple[Type[SymbolicCallable], ...]:
        """:return: every concrete symbolic callable the package defines (abstract only in its
        verbalization fragment, if at all), sorted by qualified name."""
        discovered = {
            cls
            for cls in classes_of_package(self.package, recursive=True)
            if isinstance(cls, type)
            and issubclass(cls, SymbolicCallable)
            and set(cls.__abstractmethods__) <= {"_verbalization_fragment_"}
        }
        return tuple(sorted(discovered, key=module_and_class_name))

    @staticmethod
    def has_fragment(cls: Type[SymbolicCallable]) -> bool:
        """
        :param cls: The symbolic callable to check.
        :return: whether *cls* decided its result by implementing its own fragment.
        """
        return class_implements_own_method(
            cls._verbalization_fragment_, Verbalizable._verbalization_fragment_
        )

    def placeholder_operands(self, cls: Type[SymbolicCallable]) -> Dict[str, Any]:
        """
        :param cls: The symbolic callable to build operands for.
        :return: :func:`placeholder_operands` for *cls*, with *cls*'s own
            :meth:`~krrood.entity_query_language.predicate.SymbolicCallable._example_operands_`
            applied, then this snapshot's registered :attr:`operand_overrides` overwriting the
            fields they name.
        """
        operands = cls._example_operands_(placeholder_operands(cls))
        operands.update(self.operand_overrides.get(cls, {}))
        return operands

    def rendered_result(self, cls: Type[SymbolicCallable]) -> str:
        """
        :param cls: The symbolic callable to render.
        :return: the sentence *cls* renders with this snapshot's (possibly overridden) operands.
        """
        return verbalize_expression(cls(**self.placeholder_operands(cls)))

    def assert_results_cover_every_callable(self) -> None:
        """
        Assert the declared results are exactly the discovered callables — a new class
        with no entry, or an entry for a class that no longer exists, is a red test.
        """
        discovered = {
            module_and_class_name(cls)
            for cls in self.discovered_callables()
            if self.has_fragment(cls)
        }
        declared = {
            module_and_class_name(result.callable_class) for result in self.results
        }
        missing = sorted(discovered - declared)
        stale = sorted(declared - discovered)
        assert discovered == declared, (
            f"Declared results are out of sync. Discovered classes with no entry (add one): "
            f"{missing}. Entries whose class is no longer discovered (remove them): {stale}."
        )

    def assert_declared_results_render_as_stated(self) -> None:
        """
        Assert every declared sentence matches what its class renders, so any wording
        change is re-approved by updating the entry and reviewing the diff.
        """
        mismatches = {
            module_and_class_name(result.callable_class): self.rendered_result(
                result.callable_class
            )
            for result in self.results
            if self.has_fragment(result.callable_class)
            and self.rendered_result(result.callable_class) != result.sentence
        }
        assert not mismatches, (
            "Verbalization results changed. Update the sentence for each of these in the snapshot "
            f"module: {mismatches}."
        )
