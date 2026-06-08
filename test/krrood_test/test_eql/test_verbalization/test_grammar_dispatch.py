"""
Standalone unit tests for the grammar dispatch primitive (``select``) and the
EQL-tree fold (``engine.fold``) — exercised with synthetic constructs/rules so
the dispatch mechanics are validated independently of the real grammar.
"""

from __future__ import annotations

from krrood.entity_query_language.verbalization.context import VerbalizationContext
from krrood.entity_query_language.verbalization.engine import fold
from krrood.entity_query_language.verbalization.fragments.base import (
    flatten_fragment_to_plain_text,
    PhraseFragment,
    WordFragment,
)
from krrood.entity_query_language.verbalization.grammar.phrase_rule import (
    PhraseRule,
    select,
)


# Synthetic construct hierarchy (deeper class == more specific).
class Base:
    _id_ = None
    _name_ = "base"


class Mid(Base):
    pass


class Leaf(Mid):
    pass


class Other:
    _id_ = None
    _name_ = "other"


def _rule(construct, name, **kw):
    return PhraseRule(
        construct=construct, build=lambda n, c: WordFragment(name), name=name, **kw
    )


# ── select: specificity ──────────────────────────────────────────────────────


def test_select_prefers_deeper_construct():
    rules = [_rule(Base, "base"), _rule(Mid, "mid"), _rule(Leaf, "leaf")]
    assert select(Leaf(), rules).name == "leaf"
    assert select(Mid(), rules).name == "mid"
    assert select(Base(), rules).name == "base"


def test_select_guarded_beats_unguarded_same_construct():
    rules = [
        _rule(Mid, "plain"),
        _rule(Mid, "guarded", when=lambda n: True),
    ]
    assert select(Mid(), rules).name == "guarded"


def test_select_tiebreak_breaks_same_construct_both_guarded():
    rules = [
        _rule(Mid, "low", when=lambda n: True, tiebreak=0),
        _rule(Mid, "high", when=lambda n: True, tiebreak=5),
    ]
    assert select(Mid(), rules).name == "high"


def test_select_guard_can_exclude():
    rules = [_rule(Mid, "only-even", when=lambda n: getattr(n, "ok", False))]
    node = Mid()
    assert select(node, rules) is None
    node.ok = True
    assert select(node, rules).name == "only-even"


def test_select_returns_none_when_nothing_matches():
    rules = [_rule(Mid, "mid")]
    assert select(Other(), rules) is None


# ── fold: dispatch, recursion, override, fallback ────────────────────────────


def test_fold_dispatches_to_selected_rule():
    rules = [_rule(Leaf, "leaf")]
    assert (
        flatten_fragment_to_plain_text(fold(Leaf(), VerbalizationContext(), rules))
        == "leaf"
    )


def test_fold_child_re_enters_the_fold():
    # A parent rule recurses into a child node via ctx.child.
    child = Other()
    parent = Mid()
    rules = [
        PhraseRule(
            Mid,
            name="parent",
            build=lambda n, c: PhraseFragment([WordFragment("p"), c.child(child)]),
        ),
        PhraseRule(Other, name="child", build=lambda n, c: WordFragment("c")),
    ]
    assert (
        flatten_fragment_to_plain_text(fold(parent, VerbalizationContext(), rules))
        == "p c"
    )


def test_fold_binding_override_short_circuits_before_dispatch():
    node = Mid()
    node._id_ = "k"
    context = VerbalizationContext()
    context.binding.binding_overrides["k"] = WordFragment("OVERRIDE")
    # No rules at all — the override must still win.
    assert flatten_fragment_to_plain_text(fold(node, context, [])) == "OVERRIDE"


def test_fold_falls_back_to_node_name():
    node = Mid()
    node._name_ = "fallback-name"
    assert (
        flatten_fragment_to_plain_text(fold(node, VerbalizationContext(), []))
        == "fallback-name"
    )
