"""
Tests for :class:`VerbalizationSurfaceGenerator`.

``verbalization_surfaces.py`` itself is regenerated for the real ``krrood`` package by
``conftest.py`` on every test run, so there is no separate test asserting it matches
what the generator produces -- it always does, by construction. What remains worth
testing here is the generation logic itself, against a small controlled domain.
"""

from __future__ import annotations

from krrood.entity_query_language.testing.surface_generation import (
    VerbalizationSurfaceGenerator,
)
from krrood.entity_query_language.testing.surface_verification import (
    SymbolicSurfaceSnapshot,
)
from krrood.entity_query_language.verbalization import _example_domain

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
