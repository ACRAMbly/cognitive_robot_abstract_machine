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
from .snapshot_config import KRROOD_OPERAND_OVERRIDES

# %% generation against a small, controlled domain


class TestGeneratedModuleCoversTheSameCallablesAsTheSnapshot:
    """
    The generated ``SURFACES`` tuple covers exactly the fragment-implementing callables
    the snapshot discovers, nothing more and nothing less.
    """

    def test_covered_callables_match_generated_entries(self):
        snapshot = SymbolicSurfaceSnapshot(package=_example_domain, surfaces=())
        generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

        namespace = {}
        exec(compile(generator.generate(), "<generated>", "exec"), namespace)

        generated_classes = {
            surface.callable_class for surface in namespace["SURFACES"]
        }
        assert generated_classes == set(generator.covered_callables())


class TestGeneratedModuleRendersExactlyWhatEachClassRenders:
    """
    Every generated entry's sentence is exactly what its class renders with the
    snapshot's placeholder operands -- the generator states no opinion of its own.
    """

    def test_generated_sentences_match_rendered_surfaces(self):
        snapshot = SymbolicSurfaceSnapshot(package=_example_domain, surfaces=())
        generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

        namespace = {}
        exec(compile(generator.generate(), "<generated>", "exec"), namespace)

        rendered_by_class = {
            surface.callable_class: surface.sentence
            for surface in namespace["SURFACES"]
        }
        for cls in generator.covered_callables():
            assert rendered_by_class[cls] == snapshot.rendered_surface(cls)


class TestGeneratedModulePassesItsOwnSnapshotVerification:
    """
    Feeding the generated ``SURFACES`` back into a fresh snapshot passes both
    verification assertions -- the round trip a hand-written entry has to pass too.
    """

    def test_generated_surfaces_pass_coverage_and_wording_assertions(self):
        snapshot = SymbolicSurfaceSnapshot(package=_example_domain, surfaces=())
        generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

        namespace = {}
        exec(compile(generator.generate(), "<generated>", "exec"), namespace)

        round_trip_snapshot = SymbolicSurfaceSnapshot(
            package=_example_domain, surfaces=namespace["SURFACES"]
        )
        round_trip_snapshot.assert_surfaces_cover_every_callable()
        round_trip_snapshot.assert_declared_surfaces_render_as_stated()


# %% generation against the real krrood snapshot


class TestGeneratedKrroodModuleMatchesTheCommittedFile:
    """
    The committed ``verbalization_surfaces.py`` is exactly what the generator produces
    for krrood's own snapshot -- it is a generated file, not hand-authored.
    """

    def test_generated_source_matches_committed_file(self):
        snapshot = SymbolicSurfaceSnapshot(
            package=krrood, surfaces=(), operand_overrides=KRROOD_OPERAND_OVERRIDES
        )
        generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

        generated_source = black.format_str(generator.generate(), mode=black.Mode())

        with open(committed_verbalization_surfaces_module.__file__) as committed_file:
            committed_source = committed_file.read()

        assert generated_source == committed_source
