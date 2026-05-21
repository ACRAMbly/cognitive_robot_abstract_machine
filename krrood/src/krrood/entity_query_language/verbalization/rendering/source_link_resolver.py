from __future__ import annotations

import ast
import inspect
import logging
from dataclasses import dataclass
from typing import Optional, Protocol

from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef

_log = logging.getLogger(__name__)


def _find_attribute_line(cls: type, attr_name: str) -> Optional[int]:
    """Return the absolute file line of *attr_name* defined on *cls* via ``AnnAssign``.

    Walks the MRO so inherited dataclass fields are found on the defining class.
    Returns ``None`` when the attribute cannot be located.
    """
    for klass in cls.__mro__:
        if klass is object:
            continue
        try:
            source_lines, class_start = inspect.getsourcelines(klass)
        except (OSError, TypeError):
            continue
        source = "".join(source_lines)
        try:
            tree = ast.parse(source)
        except SyntaxError:
            continue
        # The first top-level ClassDef in the snippet is the class itself.
        for top_node in tree.body:
            if isinstance(top_node, ast.ClassDef):
                for item in top_node.body:
                    if (
                        isinstance(item, ast.AnnAssign)
                        and isinstance(item.target, ast.Name)
                        and item.target.id == attr_name
                    ):
                        return class_start + item.lineno - 1
                break  # Only the outer class — do not descend into inner classes.
    return None


class SourceLinkResolver(Protocol):
    """Maps a :class:`SourceRef` to a URL string, or ``None`` when unavailable."""

    def resolve(self, ref: SourceRef) -> Optional[str]:
        ...


@dataclass
class FileURLResolver:
    """Resolves source references to ``file://`` URLs pointing at the local source tree."""

    def resolve(self, ref: SourceRef) -> Optional[str]:
        try:
            path = inspect.getfile(ref.cls)
        except (TypeError, OSError):
            return None
        if ref.attribute is None:
            try:
                _, line = inspect.getsourcelines(ref.cls)
                return f"file://{path}#{line}"
            except (OSError, TypeError):
                return f"file://{path}"
        line = _find_attribute_line(ref.cls, ref.attribute)
        return f"file://{path}#{line}" if line is not None else f"file://{path}"


@dataclass
class IdeaURIResolver:
    """Resolves source references to the ``idea://`` URI scheme registered by JetBrains IDEs.

    Clicking an ``idea://`` link (in the browser or via an OSC 8 terminal hyperlink) is
    handled by the OS URI dispatcher, which routes it to the running JetBrains IDE.
    PyCharm registers this scheme during installation (reliably via JetBrains Toolbox;
    also registered by the standalone installer on most systems).

    The generated URL format is::

        idea://open?file=/abs/path/to/file.py&line=42

    This works for both HTML output (browser click) and ANSI OSC 8 terminal hyperlinks
    (terminal Ctrl+click → ``xdg-open``).  No HTTP server or running process is required
    beyond the IDE itself being installed.
    """

    def resolve(self, ref: SourceRef) -> Optional[str]:
        try:
            path = inspect.getfile(ref.cls)
        except (TypeError, OSError):
            return None
        if ref.attribute is None:
            try:
                _, line = inspect.getsourcelines(ref.cls)
            except (OSError, TypeError):
                line = 1
        else:
            line = _find_attribute_line(ref.cls, ref.attribute) or 1
        return f"idea://open?file={path}&line={line}"


# Alias kept for backward compatibility.
PyCharmResolver = IdeaURIResolver


@dataclass
class LocalBridgeResolver:
    """Resolves source references to the local verbalization bridge server.

    The bridge server translates plain HTTP GET requests into IDE navigation
    commands, so no URI scheme registration is required.  Start it once with::

        python -m krrood.entity_query_language.verbalization.rendering.bridge_server

    Generated URL format::

        http://localhost:PORT/open?file=/abs/path/file.py&line=42

    Works for both HTML output (browser click → fetch stays on page) and ANSI
    OSC 8 terminal hyperlinks (Ctrl+click → browser opens URL → bridge opens IDE).
    """

    port: int = 8765

    def resolve(self, ref: SourceRef) -> Optional[str]:
        try:
            path = inspect.getfile(ref.cls)
        except (TypeError, OSError):
            return None
        if ref.attribute is None:
            try:
                _, line = inspect.getsourcelines(ref.cls)
            except (OSError, TypeError):
                line = 1
        else:
            line = _find_attribute_line(ref.cls, ref.attribute) or 1
        return f"http://localhost:{self.port}/open?file={path}&line={line}"


@dataclass
class AutoAPIResolver:
    """Resolves source references to Sphinx AutoAPI documentation pages.

    *base_url* is the root of the generated docs site, e.g.
    ``https://myproject.readthedocs.io/en/latest`` or a local
    ``file:///path/to/docs/_build/html``.
    """

    base_url: str

    def resolve(self, ref: SourceRef) -> Optional[str]:
        try:
            module = ref.cls.__module__
            qualname = ref.cls.__qualname__
        except AttributeError:
            return None
        module_path = module.replace(".", "/")
        anchor = f"{module}.{qualname}"
        if ref.attribute is not None:
            anchor = f"{anchor}.{ref.attribute}"
        base = self.base_url.rstrip("/")
        return f"{base}/autoapi/{module_path}/index.html#{anchor}"
