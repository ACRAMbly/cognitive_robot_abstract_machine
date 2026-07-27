"""
Tests for :class:`VerbalizationResultGenerator`.

``verbalization_results.py`` itself is regenerated for the real ``krrood`` package by
``conftest.py`` on every test run, so there is no separate test asserting it matches
what the generator produces -- it always does, by construction. What remains worth
testing here is the generation logic itself, against a small controlled domain.
"""

from __future__ import annotations

from krrood.entity_query_language.testing.result_generation import (
    VerbalizationResultGenerator,
)
from krrood.entity_query_language.testing.result_verification import (
    SymbolicResultSnapshot,
)
from krrood.entity_query_language.verbalization import _example_domain

# %% generation against a small, controlled domain


def test_generated_results_pass_their_own_snapshot_verification():
    """
    Feeding the generator's ``covered_results()`` into a fresh snapshot passes both
    verification assertions -- the same coverage and wording checks a hand-written entry
    has to pass, against the real objects the generator produces.
    """
    snapshot = SymbolicResultSnapshot(package=_example_domain, results=())
    generator = VerbalizationResultGenerator(snapshot=snapshot)

    round_trip_snapshot = SymbolicResultSnapshot(
        package=_example_domain, results=generator.covered_results()
    )
    round_trip_snapshot.assert_results_cover_every_callable()
    round_trip_snapshot.assert_declared_results_render_as_stated()
