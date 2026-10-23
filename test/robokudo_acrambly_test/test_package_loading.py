# %% Imports

from __future__ import annotations

from enum import StrEnum
from importlib import import_module
from unittest.mock import Mock

import pytest
from py_trees.composites import Sequence

from robokudo.annotators.cluster_pose_bb import ClusterPoseBBAnnotator
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.image_cluster_extractor import ImageClusterExtractor
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.query import GenerateQueryResult, QueryAnnotator
from robokudo.descriptors import CrDescriptorFactory
from robokudo.utils.module_loader import ModuleLoader

# %% Public launcher configuration


class ExtensionModule(StrEnum):
    """Package and module names for the extension's public perception components."""

    PACKAGE = "robokudo_acrambly"
    """Import namespace containing the custom perception components."""

    ENGINE = "scripted_tetris"
    """Public analysis-engine descriptor."""

    STACKING_ENGINE = "stacking_robokudo"
    """Query-driven colored-cube analysis-engine descriptor."""

    QUERY_CLIENT = "robokudo_acrambly.scripts.robokudo_cram_integration"
    """Runnable colored-block pose query client."""


# %% Engine discovery


def test_launcher_loads_the_engine_from_the_extension_package() -> None:
    """The existing RoboKudo loader discovers the extension's public engine."""
    engine = ModuleLoader().load_ae(ExtensionModule.PACKAGE, ExtensionModule.ENGINE)

    assert engine.name() == ExtensionModule.ENGINE


def test_launcher_loads_the_stacking_engine_from_the_extension_package() -> None:
    """Discover the colored-cube engine through the extension's launcher namespace."""
    engine = ModuleLoader().load_ae(
        ExtensionModule.PACKAGE, ExtensionModule.STACKING_ENGINE
    )
    descriptor = import_module(type(engine).__module__)

    assert type(engine) is descriptor.AnalysisEngine
    assert (
        descriptor.__package__
        == f"{ExtensionModule.PACKAGE}.descriptors.analysis_engines"
    )


def test_query_client_is_importable_without_starting_queries() -> None:
    """Import the standalone query API from the extension's scripts package."""
    client = import_module(ExtensionModule.QUERY_CLIENT)

    assert (
        client.query_colored_block_poses_from_robokudo.__module__
        == ExtensionModule.QUERY_CLIENT
    )


# %% Query-driven pipeline construction


def test_stacking_engine_preserves_the_colored_cube_pipeline(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Construct the ordered perception stages without opening a camera."""
    camera_descriptor = CollectionReaderAnnotator.Descriptor(
        camera_config=Mock(), camera_interface=Mock()
    )
    monkeypatch.setattr(
        CrDescriptorFactory,
        CrDescriptorFactory.create_descriptor.__name__,
        Mock(return_value=camera_descriptor),
    )
    engine = ModuleLoader().load_ae(
        ExtensionModule.PACKAGE, ExtensionModule.STACKING_ENGINE
    )

    pipeline = engine.implementation()

    assert [type(child) for child in pipeline.children] == [
        Sequence,
        QueryAnnotator,
        CollectionReaderAnnotator,
        ImagePreprocessorAnnotator,
        ImageClusterExtractor,
        ClusterPoseBBAnnotator,
        GenerateQueryResult,
    ]
    collection_reader = next(
        child
        for child in pipeline.children
        if isinstance(child, CollectionReaderAnnotator)
    )
    assert collection_reader.descriptor is camera_descriptor
