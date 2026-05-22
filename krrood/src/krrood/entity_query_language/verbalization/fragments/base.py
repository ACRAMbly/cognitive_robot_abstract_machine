from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef

if TYPE_CHECKING:
    from krrood.entity_query_language.core.mapped_variable import Attribute


@dataclass
class VerbFragment:
    """Abstract base for all verbalized output fragments."""


@dataclass
class WordFragment(VerbFragment):
    """Plain neutral text: articles, connectives, punctuation."""
    text: str


@dataclass
class RoleFragment(VerbFragment):
    """Text carrying a semantic role — drives coloring and optional source hyperlinking."""
    text: str
    role: SemanticRole
    source_ref: Optional[SourceRef] = None

    @classmethod
    def for_variable(cls, label: str, expr) -> "RoleFragment":
        """Fragment for a Variable / InstantiatedVariable / Entity, linked to its type."""
        return cls(
            text=label,
            role=SemanticRole.VARIABLE,
            source_ref=SourceRef.for_type(getattr(expr, "_type_", None)),
        )

    @classmethod
    def for_attribute(cls, label: str, owner, attr_name: str) -> "RoleFragment":
        """Fragment for an attribute access, linked to its owner class and attribute name."""
        return cls(
            text=label,
            role=SemanticRole.ATTRIBUTE,
            source_ref=SourceRef.for_attribute(owner, attr_name),
        )

    @classmethod
    def for_operator(cls, label: str) -> "RoleFragment":
        """Fragment for an operator / copula (no source link needed)."""
        return cls(text=label, role=SemanticRole.OPERATOR)


@dataclass
class PhraseFragment(VerbFragment):
    """An inline sequence of fragments joined by a separator."""
    parts: list[VerbFragment]
    separator: str = " "

    @classmethod
    def joined(cls, parts: list[VerbFragment], separator: str = " ") -> "PhraseFragment":
        return cls(parts=parts, separator=separator)

    @classmethod
    def spaced(cls, *parts: VerbFragment) -> "PhraseFragment":
        return cls(parts=list(parts), separator=" ")


@dataclass
class BlockFragment(VerbFragment):
    """
    A named structural block with sub-items.

    ParagraphRenderer flattens this into prose.
    HierarchicalRenderer turns it into an indented bullet list.
    """
    header: Optional[VerbFragment]
    items: list[VerbFragment] = field(default_factory=list)
