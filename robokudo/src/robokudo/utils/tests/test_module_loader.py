"""Tests for loading analysis engines with constructor arguments."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

from robokudo.utils.module_loader import ModuleLoader


@dataclass
class ConfigurableAnalysisEngine:
    """Mimic an analysis engine that accepts an optional recording directory."""

    output_directory: Path | None = None
    """Directory forwarded by the loader, if any."""


@pytest.fixture
def loader(monkeypatch: pytest.MonkeyPatch) -> ModuleLoader:
    """Create a loader whose analysis engine module is the configurable mimic."""
    module_loader = ModuleLoader()
    monkeypatch.setattr(
        module_loader,
        '_load_module',
        lambda ros_pkg_name, module_type, module_name: SimpleNamespace(
            AnalysisEngine=ConfigurableAnalysisEngine
        ),
    )
    return module_loader


def test_load_ae_forwards_engine_arguments(
    loader: ModuleLoader,
    tmp_path: Path,
) -> None:
    """Pass keyword arguments to the analysis engine constructor."""
    engine = loader.load_ae(
        ros_pkg_name='robokudo',
        module_name='configurable',
        output_directory=tmp_path,
    )

    assert engine.output_directory == tmp_path


def test_load_ae_without_arguments_uses_engine_defaults(
    loader: ModuleLoader,
) -> None:
    """Create the analysis engine with its defaults when no arguments are given."""
    engine = loader.load_ae(ros_pkg_name='robokudo', module_name='configurable')

    assert engine.output_directory is None
