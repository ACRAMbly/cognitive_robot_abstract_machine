"""
The grammar as first-class data — a :class:`PhraseRule` per EQL construct, the
dispatch primitive :func:`select`, and the per-node :class:`Ctx` handed to a rule.

This realises the **rule-to-rule** mapping of Montague grammar: each construct of
the source algebra (an EQL :class:`~krrood.entity_query_language.core.base_expressions.SymbolicExpression`)
has one clause describing how it composes into the target (English) algebra.  A
clause is *data* — a :class:`PhraseRule` value — not a method on a class, so the
grammar is itself queryable (see
:mod:`~krrood.entity_query_language.verbalization.grammar.registry`).

References:

* Montague, R. (1970), "Universal Grammar", *Theoria* 36 — syntax algebra →
  semantics algebra as a homomorphism.
* Bach, E. (1976) — the *rule-to-rule hypothesis* (one syntactic rule ↔ one
  semantic rule).
* Stanford Encyclopedia of Philosophy, "Montague Semantics" / "Compositionality".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING
from typing_extensions import Any, Callable, List, Optional, Sequence, TypeVar

from krrood.entity_query_language.verbalization.fragments.base import VerbFragment

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.microplanning.binding_scope import (
        BindingScope,
    )
    from krrood.entity_query_language.verbalization.microplanning.config import (
        RenderConfig,
    )
    from krrood.entity_query_language.verbalization.microplanning.referring import (
        ReferringExpressions,
    )

_T = TypeVar("_T")


def _always(node: Any) -> bool:
    """Default guard — a rule with no extra precondition beyond its ``construct``."""
    return True


@dataclass
class Ctx:
    """
    Per-node context handed to a :attr:`PhraseRule.build`.

    Bundles the single recursion entry (:attr:`child`, the fold continuation) with
    the microplanning services, so a ``build`` never recurses by hand and never
    reaches for cross-cutting state directly.
    """

    child: Callable[[Any], VerbFragment]
    """Recurse on a sub-expression — the fold continuation bound to this pass."""

    context: "VerbalizationContext"
    """The owning verbalization context (services accessed via the properties below)."""

    @property
    def refer(self) -> "ReferringExpressions":
        """Referring-expression service (articles, coreference, pronouns)."""
        return self.context.referring

    @property
    def scope(self) -> "BindingScope":
        """Binding-scope service (deferred constraints + field overrides)."""
        return self.context.binding

    @property
    def config(self) -> "RenderConfig":
        """Render-mode flags (query depth, compact predicates)."""
        return self.context.config


@dataclass(frozen=True)
class PhraseRule:
    """
    One Montague rule-to-rule clause, as data: *for this construct, build this phrase.*

    :param construct: The EQL node class this clause handles (the ``isinstance`` gate).
    :param build: ``build(node, ctx) -> VerbFragment`` — how the construct composes
        (delegating recursion to ``ctx.child`` and sub-decisions to ``ctx`` services).
    :param when: Extra precondition beyond ``construct`` (the non-``isinstance`` part
        of the old ``applies``); the default :func:`_always` means "no extra guard".
    :param name: Stable identifier for querying / tracing the grammar.
    :param tiebreak: Explicit ordering for the rare case of two rules with the same
        ``construct`` that are *both* guarded and overlap (e.g. inference vs. top-level
        entity); higher wins.
    """

    construct: type
    build: Callable[[Any, Ctx], VerbFragment]
    when: Callable[[Any], bool] = _always
    name: str = ""
    tiebreak: int = 0


def _mro_depth(cls: type) -> int:
    """Specificity of a construct: deeper in the MRO ⇒ more specific (``Literal`` > ``Variable``)."""
    return len(cls.__mro__)


def most_specific(candidates: Sequence[_T], key: Callable[[_T], tuple]) -> Optional[_T]:
    """
    Return the single most-specific candidate by *key*, or ``None`` when empty.

    The shared selection primitive — used both by :func:`select` over the grammar
    and by the subject-restriction registries, so first-match-by-specificity is
    written once.

    :param candidates: Items already filtered to those that apply.
    :param key: Specificity key; the maximum wins.
    :return: The most specific candidate, or ``None``.
    """
    return max(candidates, key=key, default=None)


def select(node, rules: Sequence[PhraseRule]) -> Optional[PhraseRule]:
    """
    Return the most-specific :class:`PhraseRule` whose ``construct`` and ``when``
    match *node*, or ``None`` when none apply.

    Specificity key, highest wins: ``(construct MRO depth, guarded over unguarded,
    explicit tiebreak)``.  This reproduces the previous engine's MRO-depth ordering
    and ``applies`` guards without a class hierarchy.

    :param node: The EQL expression being dispatched.
    :param rules: The grammar (e.g. ``ALL_PHRASE_RULES``).
    :return: The chosen rule, or ``None`` (caller supplies the fallback).
    """
    candidates = [
        rule for rule in rules if isinstance(node, rule.construct) and rule.when(node)
    ]
    return most_specific(
        candidates,
        key=lambda rule: (
            _mro_depth(rule.construct),
            rule.when is not _always,
            rule.tiebreak,
        ),
    )
