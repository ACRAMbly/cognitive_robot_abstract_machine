"""
The analysis engines RoboKudo loads to read a synthetic scenario.
"""

from __future__ import annotations

# %% Imports

from types import ModuleType

import pytest
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.core import BaseAnnotator
from robokudo.utils.module_loader import ModuleLoader

import robokudo_acrambly
from robokudo_acrambly.descriptors.analysis_engines import (
    synthetic_aircraft,
    synthetic_tetris,
)
from robokudo_acrambly.synthetic_observations.camera_interface import (
    SyntheticObservationInterface,
)


# %% Loading


@pytest.fixture(
    params=[synthetic_tetris, synthetic_aircraft], ids=["tetris", "aircraft"]
)
def engine_module(request: pytest.FixtureRequest) -> ModuleType:
    """
    The module of each synthetic analysis engine.
    """
    return request.param


def module_short_name(*, module: ModuleType) -> str:
    """
    :return: The name RoboKudo is given to load ``module`` as an analysis engine.
    """
    return module.__name__.rsplit(".", 1)[-1]


def test_robokudo_loads_the_engine_by_its_module_name(
    engine_module: ModuleType,
) -> None:
    engine = ModuleLoader().load_ae(
        ros_pkg_name=robokudo_acrambly.__name__,
        module_name=module_short_name(module=engine_module),
    )
    assert type(engine) is engine_module.AnalysisEngine
    assert engine.name() == module_short_name(module=engine_module)


# %% Pipeline


def test_pipeline_only_reads_the_engine_scenario(engine_module: ModuleType) -> None:
    engine = engine_module.AnalysisEngine()
    pipeline = engine.implementation()
    annotators = [
        behaviour
        for behaviour in pipeline.iterate()
        if isinstance(behaviour, BaseAnnotator)
    ]
    assert [type(annotator) for annotator in annotators] == [CollectionReaderAnnotator]
    camera_interface = annotators[0].descriptor.parameters.camera_interface
    assert type(camera_interface) is SyntheticObservationInterface
    assert camera_interface.camera_config.scenario is engine.scenario
