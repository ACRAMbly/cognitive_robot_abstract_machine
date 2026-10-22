# %% Imports

from __future__ import annotations

from enum import StrEnum

from robokudo.utils.module_loader import ModuleLoader


# %% Public launcher configuration


class ExtensionModule(StrEnum):
    """Package and engine names accepted by the RoboKudo launcher."""

    PACKAGE = "robokudo_acrambly"
    """Import namespace containing the custom perception components."""

    ENGINE = "scripted_tetris"
    """Public analysis-engine descriptor."""


# %% Engine discovery


def test_launcher_loads_the_engine_from_the_extension_package() -> None:
    """The existing RoboKudo loader discovers the extension's public engine."""
    engine = ModuleLoader().load_ae(ExtensionModule.PACKAGE, ExtensionModule.ENGINE)

    assert engine.name() == ExtensionModule.ENGINE
