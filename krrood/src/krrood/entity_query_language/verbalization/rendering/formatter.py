from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import ClassVar

from krrood.entity_query_language.verbalization.fragments.roles import ROLE_COLORS, SemanticRole


class BulletStyle(Enum):
    DASH = "-"
    DOT = "•"
    ASTERISK = "*"


class IndentSize(Enum):
    TWO_SPACES = "  "
    FOUR_SPACES = "    "
    TAB = "\t"


@dataclass
class Formatter(ABC):
    """Single source of truth for all format-specific characters and color markup."""

    @abstractmethod
    def colorize(self, text: str, role: SemanticRole) -> str:
        """Wrap *text* in format-specific color markup for *role*."""
        ...

    @property
    @abstractmethod
    def space(self) -> str:
        """Inline word separator character(s)."""
        ...

    @property
    @abstractmethod
    def newline(self) -> str:
        """Line break character(s)."""
        ...


@dataclass
class PlainFormatter(Formatter):
    """No color markup; standard ASCII space and newline."""

    def colorize(self, text: str, role: SemanticRole) -> str:
        return text

    @property
    def space(self) -> str:
        return " "

    @property
    def newline(self) -> str:
        return "\n"


@dataclass
class ANSIFormatter(Formatter):
    """
    True-color ANSI escape sequences (24-bit, ``\\033[38;2;R;G;Bm``).

    Works in VS Code terminal, GNOME Terminal, iTerm2, Windows Terminal, and any
    other terminal that supports the ISO-8613-3 direct-color extension.
    """

    _RESET: ClassVar[str] = "\033[0m"
    _NAMED: ClassVar[dict[str, tuple[int, int, int]]] = {
        "cornflowerblue": (100, 149, 237),
    }

    def colorize(self, text: str, role: SemanticRole) -> str:
        color = ROLE_COLORS.get(role)
        if color is None:
            return text
        r, g, b = self._hex_to_rgb(color)
        return f"\033[38;2;{r};{g};{b}m{text}{self._RESET}"

    @property
    def space(self) -> str:
        return " "

    @property
    def newline(self) -> str:
        return "\n"

    def _hex_to_rgb(self, color: str) -> tuple[int, int, int]:
        if color.startswith("#"):
            h = color.lstrip("#")
            return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        return self._NAMED.get(color.lower(), (255, 255, 255))


@dataclass
class HTMLFormatter(Formatter):
    """
    HTML output: ``<span style="color: …">`` color tags, ``&nbsp;`` spaces, ``<br>`` newlines.

    Suitable for Jupyter notebooks, GitLab Markdown, and any renderer that
    passes through inline HTML.
    """

    def colorize(self, text: str, role: SemanticRole) -> str:
        color = ROLE_COLORS.get(role)
        if color is None:
            return text
        return f'<span style="color:{color}">{text}</span>'

    @property
    def space(self) -> str:
        return "&nbsp;"

    @property
    def newline(self) -> str:
        return "<br>"
