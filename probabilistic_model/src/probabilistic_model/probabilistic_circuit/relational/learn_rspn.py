from __future__ import annotations

import pandas as pd
from typing import (
    List,
)

from krrood.ormatic.data_access_objects.dao import DataAccessObject
from krrood.parametrization.feature_extractor import FeatureExtractor
from probabilistic_model.learning.jpt.jpt import JointProbabilityTree
from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
)


def learn_probabilistic_circuit(
    instances: List[DataAccessObject],
) -> ProbabilisticCircuit:
    """
    Learn a ProbabilisticCircuit from a class and a list of instances.
    :param cls: The class to learn from.
    :param instances: The instances to learn from.
    :return: The learned ProbabilisticCircuit.
    """

    extractor = FeatureExtractor(instances)

    if not instances:
        raise ValueError("No instances provided")

    df: pd.DataFrame = extractor.create_dataframe()
    df = extractor.preprocess_dataframe(df)
    df = df.sort_index(axis=1)
    variables = infer_variables_from_dataframe(df)

    jpt = JointProbabilityTree(variables, min_samples_per_leaf=2)
    jpt = jpt.fit(df)
    return jpt
