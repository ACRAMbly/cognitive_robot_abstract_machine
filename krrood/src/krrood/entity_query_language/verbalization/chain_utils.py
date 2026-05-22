"""
Utilities for walking MappedVariable chains, building path parts, and
pluralising chain expressions.

These are pure utilities shared by multiple verbalizer subsystems.  They must
not import from the subsystem files to avoid circular dependencies.
"""
from __future__ import annotations

from typing import Callable, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from krrood.entity_query_language.core.mapped_variable import MappedVariable
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.fragments.base import VerbFragment
    from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef


def walk_chain(expr) -> tuple[list, object]:
    """Walk a MappedVariable chain outward-first; return (chain, root)."""
    from krrood.entity_query_language.core.mapped_variable import MappedVariable

    chain: list[MappedVariable] = []
    current = expr
    while isinstance(current, MappedVariable):
        chain.append(current)
        current = current._child_
    chain.reverse()
    return chain, current


def chain_root(expr) -> object:
    """Return the non-MappedVariable root of a chain (skip walking the whole list)."""
    from krrood.entity_query_language.core.mapped_variable import MappedVariable

    current = expr
    while isinstance(current, MappedVariable):
        current = current._child_
    return current


def build_path_parts(chain: list) -> list[tuple[str, Optional[SourceRef]]]:
    """
    Convert a walked chain into ``(display_name, SourceRef | None)`` pairs.

    Consecutive ``Attribute → Index`` nodes are merged into ``"attr[key]"`` pairs;
    standalone ``Index`` nodes appear as ``"[key]"`` pairs.
    """
    from krrood.entity_query_language.core.mapped_variable import Attribute, Index, Call, FlatVariable
    from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef

    parts: list[tuple[str, Optional[SourceRef]]] = []
    i = 0
    while i < len(chain):
        node = chain[i]
        if isinstance(node, Attribute):
            name = node._attribute_name_
            owner = node._owner_class_
            ref: Optional[SourceRef] = SourceRef.for_attribute(owner, name)
            while i + 1 < len(chain) and isinstance(chain[i + 1], Index):
                i += 1
                name += f"[{repr(chain[i]._key_)}]"
                ref = None  # composite indexed access has no clean single-line anchor
            parts.append((name, ref))
        elif isinstance(node, Index):
            parts.append((f"[{repr(node._key_)}]", None))
        elif isinstance(node, Call):
            parts.append(("()", None))
        elif isinstance(node, FlatVariable):
            pass
        i += 1
    return parts


def verbalize_plural(expr, ctx: VerbalizationContext, build_fn: Callable) -> VerbFragment:
    """
    Return a plural :class:`VerbFragment` for *expr*.

    *build_fn* is the main dispatcher (``EQLVerbalizer.build``) used as a
    fallback when the expression type has no special plural form.
    """
    from krrood.entity_query_language.core.mapped_variable import Attribute, FlatVariable, MappedVariable
    from krrood.entity_query_language.core.variable import Variable
    from krrood.entity_query_language.verbalization.fragments.base import PhraseFragment, RoleFragment
    from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
    from krrood.entity_query_language.verbalization.utils import _ensure_plural, inflect_engine
    from krrood.entity_query_language.verbalization.vocabulary.english import Prepositions

    if isinstance(expr, FlatVariable):
        return verbalize_plural(expr._child_, ctx, build_fn)

    if isinstance(expr, Variable):
        type_name = expr._type_.__name__
        label = ctx.disambiguation_map.get(expr._id_, type_name)
        ctx.seen[expr._id_] = label
        plural = label if label != type_name else inflect_engine.plural(type_name)
        return RoleFragment.for_variable(plural, expr)

    if isinstance(expr, Attribute):
        chain, root = walk_chain(expr)
        if isinstance(root, Variable) and len(chain) == 1 and isinstance(chain[0], Attribute):
            type_name = root._type_.__name__
            label = ctx.disambiguation_map.get(root._id_, type_name)
            ctx.seen[root._id_] = label
            root_plural = label if label != type_name else inflect_engine.plural(type_name)
            attr_name = chain[0]._attribute_name_
            attr_plural = _ensure_plural(attr_name)
            owner = chain[0]._owner_class_
            return PhraseFragment(
                parts=[
                    RoleFragment.for_attribute(attr_plural, owner, attr_name),
                    Prepositions.OF.as_fragment(),
                    RoleFragment.for_variable(root_plural, root),
                ],
                separator=" ",
            )

    return build_fn(expr, ctx)
