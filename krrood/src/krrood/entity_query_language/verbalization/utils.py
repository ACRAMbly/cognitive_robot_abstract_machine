from __future__ import annotations

import re
from typing import TYPE_CHECKING

import inflect

inflect_engine = inflect.engine()

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.fragments.base import VerbFragment


def _str(fragment: "VerbFragment") -> str:
    """Flatten a VerbFragment to a plain string (no colours) for internal string ops."""
    from krrood.entity_query_language.verbalization.fragments.base import (
        BlockFragment, PhraseFragment, RoleFragment, WordFragment,
    )
    match fragment:
        case WordFragment(text=t):
            return t
        case RoleFragment(text=t):
            return t
        case PhraseFragment(parts=parts, separator=sep):
            return sep.join(_str(p) for p in parts)
        case BlockFragment(header=header, items=items):
            parts_text = ", ".join(_str(i) for i in items)
            if header is None:
                return parts_text
            return f"{_str(header)} {parts_text}" if parts_text else _str(header)
        case _:
            return ""


def _camel_to_words(name: str) -> str:
    """Convert a CamelCase class name to space-separated lowercase words.

    Examples: ``"HasRole"`` → ``"has role"``, ``"IsReachable"`` → ``"is reachable"``.
    """
    return re.sub(r"([A-Z])", r" \1", name).strip().lower()


def _ordinal(n: int) -> str:
    return inflect_engine.ordinal(inflect_engine.number_to_words(n + 1))


def _ensure_plural(word: str) -> str:
    """Return *word* in plural form, without double-pluralising already-plural words."""
    return word if inflect_engine.singular_noun(word) else inflect_engine.plural(word)


