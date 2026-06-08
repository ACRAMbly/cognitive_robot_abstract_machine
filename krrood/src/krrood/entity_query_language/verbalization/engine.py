"""
The verbalization engine — a single catamorphism over the EQL expression tree.

:func:`fold` is the *only* place the EQL tree is recursed: it dispatches a node to
the most-specific :class:`~krrood.entity_query_language.verbalization.grammar.phrase_rule.PhraseRule`
(via :func:`~krrood.entity_query_language.verbalization.grammar.phrase_rule.select`)
and applies its ``build``, handing the rule a :class:`~krrood.entity_query_language.verbalization.grammar.phrase_rule.Ctx`
whose ``child`` re-enters the fold.  Rules therefore never recurse by hand.

This is the F-algebra / catamorphism over the source (EQL) algebra; the grammar
is the algebra (Meijer, Fokkinga & Paterson 1991, "Functional Programming with
Bananas, Lenses, Envelopes and Barbed Wire"; Bird & de Moor 1997, "Algebra of
Programming").  Compare the fold over the *output* tree,
:func:`~krrood.entity_query_language.verbalization.fragments.base.fold_fragment`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING
from typing_extensions import Optional, Sequence

from krrood.entity_query_language.verbalization.fragments.base import (
    VerbFragment,
    WordFragment,
)
from krrood.entity_query_language.verbalization.grammar.phrase_rule import (
    Ctx,
    PhraseRule,
    select,
)
from krrood.entity_query_language.verbalization.grammar.registry import ALL_PHRASE_RULES

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext


def fold(
    node,
    context: "VerbalizationContext",
    rules: Optional[Sequence[PhraseRule]] = None,
) -> VerbFragment:
    """
    Verbalize *node* by dispatching to the matching grammar rule and recursing.

    Order of resolution (matching the previous engine):

    1. **Binding-override short-circuit** — if ``node._id_`` has a pre-built
       substitute in :attr:`BindingScope.binding_overrides`, return it before any
       dispatch (used for InstantiatedVariable field references).
    2. :func:`select` the most-specific rule and apply its ``build`` with a fresh
       :class:`Ctx` whose ``child`` re-enters :func:`fold`.
    3. **Fallback** — no rule applies → a plain :class:`WordFragment` of ``node._name_``.

    :param node: Any EQL expression.
    :param context: The verbalization context (services + render config).
    :param rules: Grammar to dispatch over; defaults to ``ALL_PHRASE_RULES``.
    :return: The fragment for *node*.
    :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
    """
    rules = ALL_PHRASE_RULES if rules is None else rules

    node_id = getattr(node, "_id_", None)
    if node_id is not None:
        override = context.binding.binding_overrides.get(node_id)
        if override is not None:
            return override

    ctx = Ctx(
        child=lambda child_node: fold(child_node, context, rules), context=context
    )

    rule = select(node, rules)
    if rule is None:
        return WordFragment(text=node._name_)
    return rule.build(node, ctx)
