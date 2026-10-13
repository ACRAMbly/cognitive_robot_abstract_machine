from __future__ import annotations

from typing_extensions import List

from probabilistic_model.distributions.distributions import (
    IntegerDistribution,
    SymbolicDistribution,
    UnivariateDistribution,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.base import (
    UnivariateInputLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.dirac_delta_layer import (
    DiracDeltaLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.discrete_layer import (
    IntegerLayer,
    SymbolicLayer,
)


def point_masses_layer(
    variable_index: int, distributions: List[UnivariateDistribution]
) -> UnivariateInputLayer:
    """
    The layer of several point masses on one variable, such as the one a
    conditioned variable is reattached with.

    For a discrete variable this is a discrete layer, since a Dirac delta layer can
    neither hold the symbolic or integer distributions that
    :func:`~probabilistic_model.distributions.helper.make_dirac` creates for it nor
    answer a set of symbols as an event.

    :param variable_index: The index of the variable of the distributions.
    :param distributions: Distributions of one type that each put all of their mass
        on one value, as :func:`~probabilistic_model.distributions.helper.make_dirac`
        creates them.
    :return: The input layer with one node per distribution.
    """
    first = distributions[0]
    if isinstance(first, SymbolicDistribution):
        return SymbolicLayer.from_distributions(variable_index, distributions)
    if isinstance(first, IntegerDistribution):
        return IntegerLayer.from_distributions(variable_index, distributions)
    return DiracDeltaLayer.from_distributions(variable_index, distributions)
