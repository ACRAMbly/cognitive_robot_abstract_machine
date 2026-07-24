"""
Exhaustive verbalization-surface verification for any package, and the first-order
(value-agnostic) rendering it builds on.

Point a :class:`SymbolicSurfaceSnapshot` at a package and a committed list of
:class:`VerbalizationSurface` entries. Its three ``assert_*`` methods, used as the bodies of three
tests, check that every concrete symbolic callable the package defines (1) implements its own
verbalization fragment, (2) has a declared surface, and (3) renders exactly its declared sentence —
so a new predicate or function, or a changed shared surface builder, cannot slip through unreviewed.

The same three-line test works for any package that defines
:class:`~krrood.entity_query_language.predicate.SymbolicCallable` subclasses (krrood itself,
``semantic_digital_twin``, ``coraplex``, …): the discovery, placeholder operands, and rendering all
live here.

The snapshot's per-class rendering is itself a reusable capability, not a test-only trick:
:func:`first_order_form` verbalizes a symbolic callable *value-agnostically* — from its declared
field types alone, with no constructed instance or bound literal in hand — the sibling of the
ordinary *value-using* form :func:`~…pipeline.verbalize_expression` already produces for a real,
bound expression. It takes nothing but the class itself; a caller wanting the first-order form of
one class (documentation, introspection) can call it directly. :class:`SymbolicSurfaceSnapshot`
builds on the same :func:`placeholder_operands` but layers its own :attr:`~SymbolicSurfaceSnapshot.
operand_overrides` on top — supplying a concrete value for a field whose fragment reads it
directly rather than as a symbolic operand is a snapshot-testing concern, not a first-order-form
one, so it stays out of the general functions' signatures.
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


@dataclass(frozen=True)
class OverriddenOperand:
    """
    One dataclass field's concrete VALUE for a symbolic callable whose fragment reads
    that field directly rather than treating it as a symbolic operand.

    A ``Type`` field, for example, cannot be resolved by annotation alone — it may be a
    symbolic operand in one class and a named value in another — so its value is stated
    here per class.
    """

    name: str
    """
    The dataclass field name being overridden.
    """

    value: Any
    """
    The concrete value to pass for that field.
    """

OperandOverridesDict = Dict[Type[SymbolicCallable], Sequence[OverriddenOperand]]
"""
A mapping from the symbolic callable class to its operand overrides
"""


def placeholder_operands(cls: Type[SymbolicCallable]) -> Dict[str, Any]:
    """
    One placeholder operand per init dataclass field, from the field's declared type
    endpoint alone — a fresh variable of that type (``object`` when the endpoint is not
    a plain class), so the surface reads the operand as *"a <TypeName>"*.

    This is genuinely value-agnostic: it needs nothing beyond *cls* itself. A field whose
    fragment reads its raw VALUE rather than a symbolic operand (a ``Type`` field, say)
    still gets a placeholder variable here; a caller that must supply a real value for
    such a field instead (:class:`SymbolicSurfaceSnapshot` does, via its
    :attr:`~SymbolicSurfaceSnapshot.operand_overrides`) overwrites that entry in the
    returned dict rather than this function taking on that concern itself.

    :param cls: The symbolic callable to build operands for.
    :return: The operand to pass for each init field, keyed by field name.
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
    constructed instance or bound literal in hand. The sibling of the ordinary *value-
    using* form a real, bound expression already renders through
    :func:`~…pipeline.verbalize_expression`.

    :param cls: The symbolic callable to render.
    :return: The sentence *cls* renders with placeholder operands.
    """
    return verbalize_expression(cls(**placeholder_operands(cls)))


@dataclass(frozen=True)
class VerbalizationSurface:
    """
    One symbolic callable and the sentence it verbalizes to — a committed snapshot
    entry.
    """

    callable_class: Type[SymbolicCallable]
    """
    The symbolic function or predicate whose surface this records.
    """

    sentence: str
    """
    The approved sentence it renders with the snapshot's placeholder operands.
    """


@dataclass(frozen=True)
class SymbolicSurfaceSnapshot:
    """
    Exhaustive verbalization-surface check for the symbolic callables a package defines.

    Discovers every concrete :class:`~krrood.entity_query_language.predicate.SymbolicCallable` in
    :attr:`package`, renders each with placeholder operands, and checks the rendering against the
    committed :attr:`surfaces`. Use the three ``assert_*`` methods as the bodies of three tests.
    """

    package: ModuleType
    """
    The package whose symbolic callables are discovered and checked.
    """

    surfaces: Sequence[VerbalizationSurface]
    """
    The committed expected surfaces, one per covered class.
    """

    operand_overrides: OperandOverridesDict = field(default_factory=dict)
    """
    Concrete field overrides for classes whose fragment reads a field's raw VALUE rather
    than treating it as a symbolic operand, keyed by the class.
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
        :return: whether *cls* decided its surface by implementing its own fragment.
        """
        return class_implements_own_method(
            cls._verbalization_fragment_, Verbalizable._verbalization_fragment_
        )

    def placeholder_operands(self, cls: Type[SymbolicCallable]) -> Dict[str, Any]:
        """
        :param cls: The symbolic callable to build operands for.
        :return: :func:`placeholder_operands` for *cls*, with this snapshot's registered
            :attr:`operand_overrides` overwriting the fields they name — the snapshot's own
            concern, so the general :func:`placeholder_operands` stays override-free.
        """
        operands = placeholder_operands(cls)
        operands.update(
            {
                override.name: override.value
                for override in self.operand_overrides.get(cls, ())
            }
        )
        return operands

    def rendered_surface(self, cls: Type[SymbolicCallable]) -> str:
        """
        :param cls: The symbolic callable to render.
        :return: The sentence *cls* renders with this snapshot's (possibly overridden) operands.
        """
        return verbalize_expression(cls(**self.placeholder_operands(cls)))

    def assert_surfaces_cover_every_callable(self) -> None:
        """
        Assert the declared surfaces are exactly the discovered callables — a new class
        with no entry, or an entry for a class that no longer exists, is a red test.
        """
        discovered = {
            module_and_class_name(cls)
            for cls in self.discovered_callables()
            if self.has_fragment(cls)
        }
        declared = {
            module_and_class_name(surface.callable_class) for surface in self.surfaces
        }
        missing = sorted(discovered - declared)
        stale = sorted(declared - discovered)
        assert discovered == declared, (
            f"Declared surfaces are out of sync. Discovered classes with no entry (add one): "
            f"{missing}. Entries whose class is no longer discovered (remove them): {stale}."
        )

    def assert_declared_surfaces_render_as_stated(self) -> None:
        """
        Assert every declared sentence matches what its class renders, so any wording
        change is re-approved by updating the entry and reviewing the diff.
        """
        mismatches = {
            module_and_class_name(surface.callable_class): self.rendered_surface(
                surface.callable_class
            )
            for surface in self.surfaces
            if self.has_fragment(surface.callable_class)
               and self.rendered_surface(surface.callable_class) != surface.sentence
        }
        assert not mismatches, (
            "Verbalization surfaces changed. Update the sentence for each of these in the snapshot "
            f"module: {mismatches}."
        )
