"""
Tests for :class:`VerbalizationSurfaceGenerator`.
"""

from __future__ import annotations

import black
import krrood
from krrood.entity_query_language.testing.surface_generation import (
    VerbalizationSurfaceGenerator,
)
from krrood.entity_query_language.testing.surface_verification import (
    SymbolicSurfaceSnapshot,
)
from krrood.entity_query_language.verbalization import _example_domain

from . import verbalization_surfaces as committed_verbalization_surfaces_module

# %% generation against a small, controlled domain


def test_generated_surfaces_pass_their_own_snapshot_verification():
    """
    Feeding the generator's ``covered_surfaces()`` into a fresh snapshot passes both
    verification assertions -- the same coverage and wording checks a hand-written entry
    has to pass, against the real objects the generator produces.
    """
    snapshot = SymbolicSurfaceSnapshot(package=_example_domain, surfaces=())
    generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

    round_trip_snapshot = SymbolicSurfaceSnapshot(
        package=_example_domain, surfaces=generator.covered_surfaces()
    )
    round_trip_snapshot.assert_surfaces_cover_every_callable()
    round_trip_snapshot.assert_declared_surfaces_render_as_stated()


# %% generation against the real krrood snapshot


def test_generated_krrood_module_matches_the_committed_file():
    """
    The committed ``verbalization_surfaces.py`` is exactly what the generator produces
    for krrood's own snapshot -- it is a generated file, not hand-authored.
    """
    snapshot = SymbolicSurfaceSnapshot(package=krrood, surfaces=())
    generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

    generated_source = black.format_str(generator.generate(), mode=black.Mode())

    with open(committed_verbalization_surfaces_module.__file__) as committed_file:
        committed_source = committed_file.read()

    assert generated_source == committed_source
