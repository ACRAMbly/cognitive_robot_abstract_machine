"""
Tests for source-link hyperlink support in EQL verbalization.

Coverage:
- SourceRef: frozen dataclass, cls-only and cls+attribute forms
- _find_attribute_line: AnnAssign lookup, MRO walk, missing attribute
- FileURLResolver: file:// URLs for class and attribute
- IdeaURIResolver: idea:// URI scheme for JetBrains IDEs; PyCharmResolver alias
- LocalBridgeResolver: http://localhost:PORT/open URLs; bridge server launcher detection
- AutoAPIResolver: Sphinx AutoAPI URL structure
- Formatter.wrap_link: PlainFormatter (no-op), HTMLFormatter (<a>), ANSIFormatter (OSC 8)
- ANSIFormatter OSC 8 detection: enabled / disabled paths
- RoleFragment.source_ref: default None, explicit value
- FragmentRenderer._render_role: link injected when resolver + ref both present
- ParagraphRenderer / HierarchicalRenderer: <a> tags appear in HTML output
- VerbalizationPipeline: html() with link_resolver produces clickable class names
- Verbalizer: source_ref propagation for Variable, Attribute chain, bool Attribute
- HTML page template: localhost links intercepted by JS fetch (no navigation)
"""

from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Optional
from unittest.mock import patch

import pytest

import krrood.entity_query_language.factories as eql
from krrood.entity_query_language.factories import an, entity, variable
from krrood.entity_query_language.verbalization.fragments.base import (
    PhraseFragment,
    RoleFragment,
    VerbFragment,
    WordFragment,
    BlockFragment,
)
from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef
from krrood.entity_query_language.verbalization.pipeline import VerbalizationPipeline
from krrood.entity_query_language.verbalization.rendering.formatter import (
    ANSIFormatter,
    HTMLFormatter,
    PlainFormatter,
)
from krrood.entity_query_language.verbalization.rendering.renderer import (
    HierarchicalRenderer,
    ParagraphRenderer,
)
from krrood.entity_query_language.verbalization.rendering.source_link_resolver import (
    AutoAPIResolver,
    FileURLResolver,
    IdeaURIResolver,
    LocalBridgeResolver,
    PyCharmResolver,
    _find_attribute_line,
)
from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


# ── Test fixtures ──────────────────────────────────────────────────────────────


@dataclass
class _Sensor:
    level: int
    active: bool
    name: str


@dataclass
class _SensorChild(_Sensor):
    """Subclass to verify MRO walking in _find_attribute_line."""
    extra: str


class _ConstantResolver:
    """Stub resolver that always returns the same URL, for isolation."""

    def __init__(self, url: str = "http://example.com/source") -> None:
        self._url = url

    def resolve(self, ref: SourceRef) -> Optional[str]:
        return self._url


class _NoneResolver:
    """Stub resolver that always returns None (no link available)."""

    def resolve(self, ref: SourceRef) -> Optional[str]:
        return None


# ── SourceRef ─────────────────────────────────────────────────────────────────


def test_source_ref_cls_only():
    ref = SourceRef(cls=_Sensor)
    assert ref.cls is _Sensor
    assert ref.attribute is None


def test_source_ref_with_attribute():
    ref = SourceRef(cls=_Sensor, attribute="level")
    assert ref.cls is _Sensor
    assert ref.attribute == "level"


def test_source_ref_is_frozen():
    ref = SourceRef(cls=_Sensor)
    with pytest.raises((AttributeError, TypeError)):
        ref.cls = int  # type: ignore[misc]


def test_source_ref_equality():
    assert SourceRef(cls=_Sensor) == SourceRef(cls=_Sensor)
    assert SourceRef(cls=_Sensor, attribute="level") == SourceRef(cls=_Sensor, attribute="level")
    assert SourceRef(cls=_Sensor) != SourceRef(cls=_Sensor, attribute="level")


# ── _find_attribute_line ──────────────────────────────────────────────────────


def test_find_attribute_line_returns_int_for_known_field():
    line = _find_attribute_line(_Sensor, "level")
    assert isinstance(line, int)
    assert line > 0


def test_find_attribute_line_points_to_correct_line():
    """The returned line must contain the attribute annotation."""
    line = _find_attribute_line(_Sensor, "level")
    assert line is not None
    source_file = inspect.getfile(_Sensor)
    with open(source_file) as f:
        lines = f.readlines()
    target_line = lines[line - 1]  # line numbers are 1-based
    assert "level" in target_line


def test_find_attribute_line_returns_none_for_missing_field():
    assert _find_attribute_line(_Sensor, "nonexistent_field") is None


def test_find_attribute_line_walks_mro_for_inherited_field():
    line = _find_attribute_line(_SensorChild, "level")
    assert line is not None
    assert line > 0


def test_find_attribute_line_finds_child_own_field():
    line = _find_attribute_line(_SensorChild, "extra")
    assert line is not None
    assert line > 0


# ── FileURLResolver ────────────────────────────────────────────────────────────


def test_file_url_resolver_class_starts_with_file():
    r = FileURLResolver()
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert url.startswith("file://")


def test_file_url_resolver_class_url_contains_filename():
    r = FileURLResolver()
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert "test_source_links" in url


def test_file_url_resolver_class_url_has_line_anchor():
    r = FileURLResolver()
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert "#" in url


def test_file_url_resolver_attribute_url_has_line_anchor():
    r = FileURLResolver()
    url = r.resolve(SourceRef(cls=_Sensor, attribute="level"))
    assert url is not None
    assert "#" in url


def test_file_url_resolver_attribute_line_differs_from_class_line():
    r = FileURLResolver()
    class_url = r.resolve(SourceRef(cls=_Sensor))
    attr_url = r.resolve(SourceRef(cls=_Sensor, attribute="level"))
    # Both must be present and point to different lines
    assert class_url is not None and attr_url is not None
    assert class_url != attr_url


def test_file_url_resolver_returns_none_for_builtin():
    r = FileURLResolver()
    assert r.resolve(SourceRef(cls=int)) is None


# ── IdeaURIResolver ───────────────────────────────────────────────────────────


def test_idea_uri_resolver_uses_idea_scheme():
    r = IdeaURIResolver()
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert url.startswith("idea://open?file=")


def test_idea_uri_resolver_class_url_contains_path():
    r = IdeaURIResolver()
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert "test_source_links" in url


def test_idea_uri_resolver_class_url_has_line_param():
    r = IdeaURIResolver()
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert "&line=" in url
    line_part = url.split("&line=")[1]
    assert int(line_part) > 0


def test_idea_uri_resolver_attribute_url_has_line_param():
    r = IdeaURIResolver()
    url = r.resolve(SourceRef(cls=_Sensor, attribute="level"))
    assert url is not None
    assert "&line=" in url
    line_part = url.split("&line=")[1]
    assert int(line_part) > 0


def test_idea_uri_resolver_attribute_line_differs_from_class_line():
    r = IdeaURIResolver()
    class_url = r.resolve(SourceRef(cls=_Sensor))
    attr_url = r.resolve(SourceRef(cls=_Sensor, attribute="level"))
    assert class_url is not None and attr_url is not None
    assert class_url != attr_url


def test_idea_uri_resolver_returns_none_for_builtin():
    r = IdeaURIResolver()
    assert r.resolve(SourceRef(cls=int)) is None


def test_pycharm_resolver_is_alias_for_idea_uri_resolver():
    assert PyCharmResolver is IdeaURIResolver


# ── AutoAPIResolver ────────────────────────────────────────────────────────────


def test_autoapi_resolver_class_url_structure():
    r = AutoAPIResolver(base_url="https://docs.example.com")
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert url.startswith("https://docs.example.com/autoapi/")
    assert "#" in url


def test_autoapi_resolver_class_anchor_contains_qualname():
    r = AutoAPIResolver(base_url="https://docs.example.com")
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    anchor = url.split("#")[1]
    assert "_Sensor" in anchor


def test_autoapi_resolver_attribute_anchor_contains_attr():
    r = AutoAPIResolver(base_url="https://docs.example.com")
    url = r.resolve(SourceRef(cls=_Sensor, attribute="level"))
    assert url is not None
    anchor = url.split("#")[1]
    assert "level" in anchor


def test_autoapi_resolver_strips_trailing_slash_from_base():
    r1 = AutoAPIResolver(base_url="https://docs.example.com/")
    r2 = AutoAPIResolver(base_url="https://docs.example.com")
    ref = SourceRef(cls=_Sensor)
    assert r1.resolve(ref) == r2.resolve(ref)


def test_autoapi_resolver_module_path_uses_slashes():
    r = AutoAPIResolver(base_url="https://docs.example.com")
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    # Module path segment must use "/" not "."
    path_part = url.split("/autoapi/")[1].split("/index.html")[0]
    assert "." not in path_part


# ── Formatter.wrap_link ────────────────────────────────────────────────────────


def test_plain_formatter_wrap_link_is_noop():
    f = PlainFormatter()
    assert f.wrap_link("Robot", "http://example.com") == "Robot"


def test_html_formatter_wrap_link_produces_anchor():
    f = HTMLFormatter()
    result = f.wrap_link("Robot", "http://example.com")
    assert result == '<a href="http://example.com">Robot</a>'


def test_html_formatter_wrap_link_preserves_inner_markup():
    f = HTMLFormatter()
    colored = '<span style="color:cornflowerblue">Robot</span>'
    result = f.wrap_link(colored, "http://example.com")
    assert result.startswith('<a href="http://example.com">')
    assert "cornflowerblue" in result


def test_ansi_formatter_wrap_link_osc8_enabled():
    f = ANSIFormatter()
    object.__setattr__(f, "_hyperlinks_enabled", True)  # force-enable for test isolation
    result = f.wrap_link("Robot", "http://example.com")
    assert "\033]8;;http://example.com\033\\" in result
    assert "Robot" in result
    assert result.endswith("\033]8;;\033\\")


def test_ansi_formatter_wrap_link_osc8_disabled_returns_text():
    f = ANSIFormatter()
    object.__setattr__(f, "_hyperlinks_enabled", False)
    result = f.wrap_link("Robot", "http://example.com")
    assert result == "Robot"


# ── ANSIFormatter OSC 8 detection ─────────────────────────────────────────────


def test_ansi_formatter_detects_gnome_terminal_via_vte_version():
    with patch.dict("os.environ", {"VTE_VERSION": "6800"}, clear=False):
        f = ANSIFormatter()
        assert f._hyperlinks_enabled is True


def test_ansi_formatter_detects_vscode_terminal():
    with patch.dict("os.environ", {"TERM_PROGRAM": "vscode"}, clear=False):
        f = ANSIFormatter()
        assert f._hyperlinks_enabled is True


def test_ansi_formatter_detects_kitty():
    with patch.dict("os.environ", {"TERM": "xterm-kitty"}, clear=False):
        f = ANSIFormatter()
        assert f._hyperlinks_enabled is True


def test_ansi_formatter_unknown_terminal_disables_hyperlinks():
    env = {"VTE_VERSION": "", "TERM_PROGRAM": "unknown-term", "TERM": "xterm"}
    with patch.dict("os.environ", env, clear=False):
        f = ANSIFormatter()
        assert f._hyperlinks_enabled is False


# ── RoleFragment.source_ref ────────────────────────────────────────────────────


def test_role_fragment_source_ref_defaults_to_none():
    frag = RoleFragment(text="Robot", role=SemanticRole.VARIABLE)
    assert frag.source_ref is None


def test_role_fragment_accepts_source_ref():
    ref = SourceRef(cls=_Sensor)
    frag = RoleFragment(text="Sensor", role=SemanticRole.VARIABLE, source_ref=ref)
    assert frag.source_ref is ref


# ── FragmentRenderer._render_role ─────────────────────────────────────────────


def test_paragraph_renderer_injects_link_when_resolver_and_ref_present():
    resolver = _ConstantResolver("http://example.com")
    r = ParagraphRenderer(HTMLFormatter(), resolver)
    ref = SourceRef(cls=_Sensor)
    frag = RoleFragment(text="Sensor", role=SemanticRole.VARIABLE, source_ref=ref)
    result = r.render(frag)
    assert '<a href="http://example.com">' in result
    assert "Sensor" in result


def test_paragraph_renderer_no_link_when_no_resolver():
    r = ParagraphRenderer(HTMLFormatter())
    ref = SourceRef(cls=_Sensor)
    frag = RoleFragment(text="Sensor", role=SemanticRole.VARIABLE, source_ref=ref)
    result = r.render(frag)
    assert "<a " not in result
    assert "Sensor" in result


def test_paragraph_renderer_no_link_when_resolver_returns_none():
    r = ParagraphRenderer(HTMLFormatter(), _NoneResolver())
    ref = SourceRef(cls=_Sensor)
    frag = RoleFragment(text="Sensor", role=SemanticRole.VARIABLE, source_ref=ref)
    result = r.render(frag)
    assert "<a " not in result


def test_paragraph_renderer_no_link_when_no_source_ref():
    resolver = _ConstantResolver("http://example.com")
    r = ParagraphRenderer(HTMLFormatter(), resolver)
    frag = RoleFragment(text="Sensor", role=SemanticRole.VARIABLE)
    result = r.render(frag)
    assert "<a " not in result


def test_hierarchical_renderer_injects_link():
    resolver = _ConstantResolver("http://example.com")
    r = HierarchicalRenderer(HTMLFormatter(), resolver)
    ref = SourceRef(cls=_Sensor)
    block = BlockFragment(
        header=RoleFragment(text="Sensor", role=SemanticRole.VARIABLE, source_ref=ref),
        items=[WordFragment("some condition")],
    )
    result = r.render(block)
    assert '<a href="http://example.com">' in result


# ── Pipeline: html() with link_resolver ───────────────────────────────────────


def test_pipeline_html_with_resolver_links_variable_name():
    r = variable(_Sensor, [])
    resolver = _ConstantResolver("http://example.com/sensor")
    text = VerbalizationPipeline.html(link_resolver=resolver).verbalize(an(entity(r)))
    assert '<a href="http://example.com/sensor">' in text
    assert "_Sensor" in text


def test_pipeline_html_without_resolver_no_anchor_tags():
    r = variable(_Sensor, [])
    text = VerbalizationPipeline.html().verbalize(an(entity(r)))
    assert "<a " not in text


def test_pipeline_ansi_with_resolver_and_osc8_emits_escape():
    r = variable(_Sensor, [])
    resolver = _ConstantResolver("http://example.com/sensor")
    with patch.dict("os.environ", {"VTE_VERSION": "6800"}, clear=False):
        text = VerbalizationPipeline.ansi(link_resolver=resolver).verbalize(an(entity(r)))
    assert "\033]8;;" in text


def test_pipeline_ansi_with_resolver_no_osc8_logs_warning_and_no_osc8():
    import krrood.entity_query_language.verbalization.pipeline as pipeline_mod
    from unittest.mock import patch as _patch

    r = variable(_Sensor, [])
    resolver = _ConstantResolver("http://example.com/sensor")
    env = {"VTE_VERSION": "", "TERM_PROGRAM": "unknown", "TERM": "xterm"}
    with patch.dict("os.environ", env, clear=False):
        with _patch.object(pipeline_mod._log, "warning") as mock_warn:
            text = VerbalizationPipeline.ansi(link_resolver=resolver).verbalize(an(entity(r)))
    assert "\033]8;;" not in text
    mock_warn.assert_called_once()
    assert "OSC 8" in mock_warn.call_args[0][0]


# ── Verbalizer source_ref propagation ─────────────────────────────────────────


def _collect_source_refs(fragment: VerbFragment) -> list[SourceRef]:
    """Recursively collect all non-None SourceRef values from a fragment tree."""
    match fragment:
        case RoleFragment(source_ref=ref) if ref is not None:
            return [ref]
        case PhraseFragment(parts=parts):
            return [r for p in parts for r in _collect_source_refs(p)]
        case BlockFragment(header=header, items=items):
            result = _collect_source_refs(header) if header else []
            return result + [r for item in items for r in _collect_source_refs(item)]
        case _:
            return []


def test_variable_fragment_carries_source_ref_for_its_type():
    x = variable(_Sensor, [])
    frag = EQLVerbalizer().build(x)
    refs = _collect_source_refs(frag)
    assert any(r.cls is _Sensor and r.attribute is None for r in refs)


def test_attribute_fragment_carries_source_ref_with_attribute_name():
    x = variable(_Sensor, [])
    frag = EQLVerbalizer().build(x.level > 5)
    refs = _collect_source_refs(frag)
    assert any(r.cls is _Sensor and r.attribute == "level" for r in refs)


def test_bool_attribute_chain_carries_source_ref():
    x = variable(_Sensor, [])
    frag = EQLVerbalizer().build(x.active)
    refs = _collect_source_refs(frag)
    assert any(r.cls is _Sensor and r.attribute == "active" for r in refs)


def test_comparator_fragment_has_both_class_and_attr_refs():
    # The where-clause in a full query is reduced to a plain string by verbalize(),
    # so check the attribute ref on the comparator fragment directly.
    x = variable(_Sensor, [])
    frag = EQLVerbalizer().build(x.level > 0)
    refs = _collect_source_refs(frag)
    class_refs = [r for r in refs if r.cls is _Sensor and r.attribute is None]
    attr_refs = [r for r in refs if r.cls is _Sensor and r.attribute == "level"]
    assert class_refs, "Expected a SourceRef for the _Sensor class"
    assert attr_refs, "Expected a SourceRef for _Sensor.level"


# ── FileURLResolver end-to-end with verbalizer ────────────────────────────────


def test_file_url_resolver_end_to_end_html():
    """Full pipeline: variable query → HTML with file:// links for class names."""
    x = variable(_Sensor, [])
    resolver = FileURLResolver()
    text = VerbalizationPipeline.html(link_resolver=resolver).verbalize(an(entity(x)))
    assert 'href="file://' in text
    assert "_Sensor" in text


def test_idea_uri_resolver_end_to_end_ansi():
    """ANSI pipeline with IdeaURIResolver emits OSC 8 escape sequences."""
    x = variable(_Sensor, [])
    resolver = IdeaURIResolver()
    with patch.dict("os.environ", {"VTE_VERSION": "6800"}, clear=False):
        text = VerbalizationPipeline.ansi(link_resolver=resolver).verbalize(an(entity(x)))
    assert "\033]8;;idea://open?file=" in text
    assert "_Sensor" in text


def test_autoapi_resolver_end_to_end_html():
    x = variable(_Sensor, [])
    resolver = AutoAPIResolver(base_url="https://docs.example.com")
    text = VerbalizationPipeline.html(link_resolver=resolver).verbalize(an(entity(x)))
    assert 'href="https://docs.example.com/autoapi/' in text
    assert "_Sensor" in text


# ── VerbalizationPipeline.display / display_fragment ──────────────────────────


def test_display_opens_browser_outside_jupyter():
    """Outside Jupyter, display() writes a temp HTML file and calls webbrowser.open."""
    from unittest.mock import MagicMock, patch as _patch
    import krrood.entity_query_language.verbalization.pipeline as pipeline_mod

    x = variable(_Sensor, [])
    pipeline = VerbalizationPipeline.html(link_resolver=FileURLResolver())
    with _patch.object(pipeline_mod, "_is_ipython", return_value=False):
        with _patch("webbrowser.open") as mock_open:
            pipeline.display(an(entity(x)))

    mock_open.assert_called_once()
    opened_url = mock_open.call_args[0][0]
    assert opened_url.startswith("file://")
    assert opened_url.endswith(".html")


def test_display_writes_full_html_page_to_temp_file():
    """The temp file written by display() is a complete HTML document."""
    from unittest.mock import patch as _patch
    import krrood.entity_query_language.verbalization.pipeline as pipeline_mod
    import os

    x = variable(_Sensor, [])
    pipeline = VerbalizationPipeline.html()
    captured_path: list[str] = []

    def fake_open(url: str) -> None:
        captured_path.append(url.removeprefix("file://"))

    with _patch.object(pipeline_mod, "_is_ipython", return_value=False):
        with _patch("webbrowser.open", side_effect=fake_open):
            pipeline.display(an(entity(x)))

    assert captured_path, "webbrowser.open was not called"
    path = captured_path[0]
    assert os.path.exists(path)
    content = open(path, encoding="utf-8").read()
    assert "<!DOCTYPE html>" in content
    assert "_Sensor" in content
    os.unlink(path)


def test_display_in_jupyter_calls_ipython_display():
    """Inside a Jupyter session, display() calls IPython.display.HTML."""
    from unittest.mock import MagicMock, patch as _patch
    import krrood.entity_query_language.verbalization.pipeline as pipeline_mod

    x = variable(_Sensor, [])
    pipeline = VerbalizationPipeline.html(link_resolver=IdeaURIResolver())
    mock_html_cls = MagicMock()
    mock_ipython_display = MagicMock()
    pipeline.display(an(entity(x)))
    with _patch.object(pipeline_mod, "_is_ipython", return_value=True):
        with _patch.dict(
            "sys.modules",
            {"IPython.display": MagicMock(
                display=mock_ipython_display, HTML=mock_html_cls
            )},
        ):
            pipeline.display(an(entity(x)))

    mock_html_cls.assert_called_once()
    html_arg = mock_html_cls.call_args[0][0]
    assert "_Sensor" in html_arg
    mock_ipython_display.assert_called_once()


def test_display_fragment_works_like_display():
    """display_fragment() accepts a pre-built fragment instead of an expression."""
    from unittest.mock import patch as _patch
    import krrood.entity_query_language.verbalization.pipeline as pipeline_mod
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer

    x = variable(_Sensor, [])
    fragment = EQLVerbalizer().build(an(entity(x)))
    pipeline = VerbalizationPipeline.html()

    with _patch.object(pipeline_mod, "_is_ipython", return_value=False):
        with _patch("webbrowser.open") as mock_open:
            pipeline.display_fragment(fragment)

    mock_open.assert_called_once()


def test_display_html_page_has_dark_background_style():
    """The browser-fallback page includes the dark-background stylesheet."""
    from unittest.mock import patch as _patch
    import krrood.entity_query_language.verbalization.pipeline as pipeline_mod
    import os

    x = variable(_Sensor, [])
    pipeline = VerbalizationPipeline.html()
    captured_path: list[str] = []

    def fake_open(url: str) -> None:
        captured_path.append(url.removeprefix("file://"))

    with _patch.object(pipeline_mod, "_is_ipython", return_value=False):
        with _patch("webbrowser.open", side_effect=fake_open):
            pipeline.display(an(entity(x)))

    content = open(captured_path[0], encoding="utf-8").read()
    assert "background" in content
    os.unlink(captured_path[0])


# ── LocalBridgeResolver ────────────────────────────────────────────────────────


def test_local_bridge_resolver_default_port():
    r = LocalBridgeResolver()
    assert r.port == 8765


def test_local_bridge_resolver_custom_port():
    r = LocalBridgeResolver(port=9000)
    assert r.port == 9000


def test_local_bridge_resolver_class_url_format():
    r = LocalBridgeResolver(port=8765)
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert url.startswith("http://localhost:8765/open?file=")
    assert "&line=" in url


def test_local_bridge_resolver_class_url_contains_filename():
    r = LocalBridgeResolver()
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert "test_source_links" in url


def test_local_bridge_resolver_attribute_url_has_line_param():
    r = LocalBridgeResolver()
    url = r.resolve(SourceRef(cls=_Sensor, attribute="level"))
    assert url is not None
    assert "&line=" in url
    line_part = url.split("&line=")[1]
    assert int(line_part) > 0


def test_local_bridge_resolver_attribute_line_differs_from_class_line():
    r = LocalBridgeResolver()
    class_url = r.resolve(SourceRef(cls=_Sensor))
    attr_url = r.resolve(SourceRef(cls=_Sensor, attribute="level"))
    assert class_url is not None and attr_url is not None
    assert class_url != attr_url


def test_local_bridge_resolver_returns_none_for_builtin():
    r = LocalBridgeResolver()
    assert r.resolve(SourceRef(cls=int)) is None


def test_local_bridge_resolver_custom_port_in_url():
    r = LocalBridgeResolver(port=12345)
    url = r.resolve(SourceRef(cls=_Sensor))
    assert url is not None
    assert "localhost:12345" in url


def test_local_bridge_resolver_end_to_end_html():
    """Full pipeline: variable → HTML with localhost bridge links."""
    x = variable(_Sensor, [])
    resolver = LocalBridgeResolver(port=8765)
    pipeline = VerbalizationPipeline.html(link_resolver=resolver)
    text = pipeline.verbalize(an(entity(x)))
    pipeline.display(an(entity(x)))
    assert 'href="http://localhost:8765/open?file=' in text
    assert "_Sensor" in text


# ── HTML page template: JS fetch interceptor for localhost links ────────────────


def test_html_page_template_contains_fetch_interceptor():
    """The full-page HTML template for browser display includes the JS fetch interceptor."""
    from krrood.entity_query_language.verbalization.pipeline import _HTML_PAGE_TEMPLATE

    page = _HTML_PAGE_TEMPLATE.format(body="<p>test</p>")
    assert "fetch(" in page
    assert "localhost" in page
    assert "preventDefault" in page


def test_html_page_template_interceptor_targets_localhost_only():
    """The JS interceptor regex only matches http://localhost: URLs."""
    from krrood.entity_query_language.verbalization.pipeline import _HTML_PAGE_TEMPLATE

    page = _HTML_PAGE_TEMPLATE.format(body="<p>test</p>")
    # The regex pattern that guards the fetch call must reference localhost
    assert "localhost" in page


# ── Bridge server: _find_charm launcher detection ──────────────────────────────


def test_find_charm_returns_string_or_none():
    """_find_charm() must return either a non-empty string or None."""
    from krrood.entity_query_language.verbalization.rendering.bridge_server import _find_charm

    result = _find_charm()
    assert result is None or (isinstance(result, str) and len(result) > 0)


def test_find_charm_returns_executable_when_charm_on_path():
    """When a known launcher name is on PATH, _find_charm returns its path."""
    import shutil
    from unittest.mock import patch as _patch
    from krrood.entity_query_language.verbalization.rendering.bridge_server import _find_charm

    real_python = shutil.which("python3") or shutil.which("python")
    if real_python is None:
        pytest.skip("python not on PATH")

    # Pretend "charm" resolves to our python interpreter to test the detection path.
    original_which = shutil.which

    def fake_which(name):
        if name == "charm":
            return real_python
        return original_which(name)

    with _patch("shutil.which", side_effect=fake_which):
        result = _find_charm()

    assert result == real_python


def test_find_charm_returns_none_when_nothing_found(tmp_path):
    """When no launcher is anywhere, _find_charm returns None."""
    from unittest.mock import patch as _patch
    from krrood.entity_query_language.verbalization.rendering.bridge_server import _find_charm

    with _patch("shutil.which", return_value=None):
        with _patch("pathlib.Path.home", return_value=tmp_path):
            result = _find_charm()

    assert result is None


# ── Bridge server: HTTP handler ────────────────────────────────────────────────


def test_bridge_server_handler_returns_200(tmp_path):
    """The bridge HTTP handler responds 200 regardless of whether a launcher is found."""
    import io
    from unittest.mock import MagicMock, patch as _patch
    from http.server import BaseHTTPRequestHandler
    from krrood.entity_query_language.verbalization.rendering.bridge_server import _BridgeHandler

    # Build a minimal mock request/socket that BaseHTTPRequestHandler needs.
    raw_request = b"GET /open?file=/tmp/foo.py&line=1 HTTP/1.0\r\n\r\n"
    mock_socket = MagicMock()
    mock_socket.makefile.return_value = io.BytesIO(raw_request)

    output_buffer = io.BytesIO()
    mock_socket.sendall = output_buffer.write

    with _patch(
        "krrood.entity_query_language.verbalization.rendering.bridge_server._find_charm",
        return_value=None,
    ):
        handler = _BridgeHandler.__new__(_BridgeHandler)
        handler.rfile = io.BytesIO(b"")
        handler.wfile = output_buffer
        handler.client_address = ("127.0.0.1", 0)
        handler.server = MagicMock()
        handler.path = "/open?file=/tmp/foo.py&line=1"
        handler.requestline = "GET /open?file=/tmp/foo.py&line=1 HTTP/1.0"
        handler.command = "GET"
        handler.request_version = "HTTP/1.0"
        response_parts: list[bytes] = []
        handler.send_response = lambda code, *a: response_parts.append(str(code).encode())
        handler.send_header = lambda k, v: None
        handler.end_headers = lambda: None
        handler.wfile = MagicMock()
        handler.do_GET()

    assert response_parts and response_parts[0] == b"200"


def test_bridge_server_handler_calls_launcher_when_found():
    """When a launcher is found, the handler spawns it with --line and file path."""
    import io
    from unittest.mock import MagicMock, patch as _patch
    from krrood.entity_query_language.verbalization.rendering.bridge_server import _BridgeHandler

    with _patch(
        "krrood.entity_query_language.verbalization.rendering.bridge_server._find_charm",
        return_value="/usr/bin/charm",
    ):
        with _patch("subprocess.Popen") as mock_popen:
            handler = _BridgeHandler.__new__(_BridgeHandler)
            handler.path = "/open?file=/home/user/foo.py&line=42"
            handler.send_response = MagicMock()
            handler.send_header = MagicMock()
            handler.end_headers = MagicMock()
            handler.wfile = MagicMock()
            handler.do_GET()

    mock_popen.assert_called_once()
    cmd = mock_popen.call_args[0][0]
    assert "/usr/bin/charm" in cmd
    assert "--line" in cmd
    assert "42" in cmd
    assert "/home/user/foo.py" in cmd
