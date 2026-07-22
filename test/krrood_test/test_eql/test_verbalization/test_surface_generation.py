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


def _generated_surfaces(generator: VerbalizationSurfaceGenerator):
    namespace = {}
    exec(compile(generator.generate(), "<generated>", "exec"), namespace)
    return namespace["SURFACES"]


def test_generated_module_covers_the_same_callables_as_the_snapshot():
    """
    The generated ``SURFACES`` tuple covers exactly the fragment-implementing callables
    the snapshot discovers, nothing more and nothing less.
    """
    snapshot = SymbolicSurfaceSnapshot(package=_example_domain, surfaces=())
    generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

    generated_classes = {
        surface.callable_class for surface in _generated_surfaces(generator)
    }
    assert generated_classes == set(generator.covered_callables())


def test_generated_module_renders_exactly_what_each_class_renders():
    """
    Every generated entry's sentence is exactly what its class renders with the
    snapshot's placeholder operands -- the generator states no opinion of its own.
    """
    snapshot = SymbolicSurfaceSnapshot(package=_example_domain, surfaces=())
    generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

    rendered_by_class = {
        surface.callable_class: surface.sentence
        for surface in _generated_surfaces(generator)
    }
    for cls in generator.covered_callables():
        assert rendered_by_class[cls] == snapshot.rendered_surface(cls)


def test_generated_module_passes_its_own_snapshot_verification():
    """
    Feeding the generated ``SURFACES`` back into a fresh snapshot passes both
    verification assertions -- the round trip a hand-written entry has to pass too.
    """
    snapshot = SymbolicSurfaceSnapshot(package=_example_domain, surfaces=())
    generator = VerbalizationSurfaceGenerator(snapshot=snapshot)

    round_trip_snapshot = SymbolicSurfaceSnapshot(
        package=_example_domain, surfaces=_generated_surfaces(generator)
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
