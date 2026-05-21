from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

from krrood.entity_query_language.verbalization.fragments.base import (
    BlockFragment,
    PhraseFragment,
    RoleFragment,
    VerbFragment,
    WordFragment,
)
from krrood.entity_query_language.verbalization.rendering.formatter import (
    BulletStyle,
    Formatter,
    IndentSize,
    PlainFormatter,
)

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.rendering.source_link_resolver import SourceLinkResolver


@dataclass
class FragmentRenderer(ABC):
    """Converts a VerbFragment tree into a string."""

    _formatter: Formatter = field(default_factory=PlainFormatter)
    _link_resolver: Optional["SourceLinkResolver"] = field(default=None)

    @abstractmethod
    def render(self, fragment: VerbFragment) -> str:
        """
        Render a VerbFragment tree into a string.

        :param fragment: The root of the fragment tree.
        :return: The rendered string.
        """
        ...

    def _render_role(self, text: str, role, source_ref) -> str:
        """Colorize *text* and, when a resolver and source ref are present, wrap with a link."""
        colored = self._formatter.colorize(text, role)
        if source_ref is not None and self._link_resolver is not None:
            url = self._link_resolver.resolve(source_ref)
            if url is not None:
                return self._formatter.wrap_link(colored, url)
        return colored


@dataclass
class ParagraphRenderer(FragmentRenderer):
    """
    Flattens the fragment tree into a single prose string.

    BlockFragment headers and items are joined inline; nesting adds no
    visual structure — only content.
    """

    def render(self, fragment: VerbFragment) -> str:
        match fragment:
            case WordFragment(text=text):
                return text
            case RoleFragment(text=text, role=role, source_ref=ref):
                return self._render_role(text, role, ref)
            case PhraseFragment(parts=parts, separator=sep):
                rendered = [self.render(p) for p in parts]
                return sep.join(rendered)
            case BlockFragment(header=header, items=items):
                rendered_items = [self.render(i) for i in items]
                prose = ", ".join(rendered_items)
                if header is None:
                    return prose
                header_str = self.render(header)
                return f"{header_str}{self._formatter.space}{prose}" if prose else header_str
            case _:
                return ""


@dataclass
class HierarchicalRenderer(FragmentRenderer):
    """
    Renders BlockFragments as indented bullet lists.

    Each level of BlockFragment nesting adds one ``indent`` step.
    Non-block fragments are rendered inline using the same formatter.

    Example output (ANSI/plain)::

        If:
          - there's a Handle
          - there's a PrismaticConnection, whose child is …
        Then:
          - there's a Drawer
            - whose container is …
    """

    indent_size: IndentSize = field(default=IndentSize.TWO_SPACES)
    """
    The size of the indentation for each level of nesting.
    """
    bullet: BulletStyle = field(default=BulletStyle.DASH)
    """
    The bullet character to use for the list items.
    """

    def render(self, fragment: VerbFragment, depth: int = 0) -> str:
        match fragment:
            case BlockFragment(header=header, items=items):
                lines: list[str] = []
                if header is not None:
                    lines.append(self.formatted_indent * depth + self._inline(header))
                    depth = depth + 1
                for item in items:
                    lines.append(self._render_item(item, depth))
                return self._formatter.newline.join(lines)
            case _:
                return self.formatted_indent * depth + self._inline(fragment)

    @property
    def formatted_indent(self) -> str:
        """The indentation string, with spaces replaced by the formatter's space character."""
        return self.indent_size.value.replace(' ', self._formatter.space)

    def _render_item(self, fragment: VerbFragment, depth: int) -> str:
        """Render one item, prepending the bullet at its indentation level."""
        match fragment:
            case BlockFragment():
                return self.render(fragment, depth)
            case _:
                prefix = self.formatted_indent * depth + self.bullet.value + self._formatter.space
                return prefix + self._inline(fragment)

    def _inline(self, fragment: VerbFragment) -> str:
        """Render a non-block fragment as a flat inline string."""
        match fragment:
            case WordFragment(text=text):
                return text
            case RoleFragment(text=text, role=role, source_ref=ref):
                return self._render_role(text, role, ref)
            case PhraseFragment(parts=parts, separator=sep):
                return sep.join(self._inline(p) for p in parts)
            case BlockFragment():
                return self.render(fragment, 0)
            case _:
                return ""
