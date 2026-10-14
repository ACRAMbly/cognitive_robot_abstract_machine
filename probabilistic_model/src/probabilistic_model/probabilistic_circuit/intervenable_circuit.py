from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from sortedcontainers import SortedSet
from typing_extensions import Iterable, List, Self

from probabilistic_model.exceptions import (
    EmptyCompositionError,
    MixtureOverDifferentVariablesError,
    NumberOfWeightsMismatchError,
    ProductOverSharedVariablesError,
)
from probabilistic_model.probabilistic_model import ProbabilisticModel

# %% what a composition requires of its parts


def validate_mixture(
    components: List[IntervenableCircuit], log_weights: List[float]
) -> None:
    """
    Check that circuits and weights make up a smooth mixture.

    :param components: The components to mix.
    :param log_weights: The logarithmic weight of every component.
    :raises EmptyCompositionError: If there is no component.
    :raises NumberOfWeightsMismatchError: If there is not one weight per component.
    :raises MixtureOverDifferentVariablesError: If the components are not over the same
        variables.
    """
    if not components:
        raise EmptyCompositionError(IntervenableCircuit.mixture_of)
    if len(log_weights) != len(components):
        raise NumberOfWeightsMismatchError(len(log_weights), len(components))
    variables_per_component = [list(component.variables) for component in components]
    if any(
        variables != variables_per_component[0]
        for variables in variables_per_component[1:]
    ):
        raise MixtureOverDifferentVariablesError(variables_per_component)


def validate_product(factors: List[IntervenableCircuit]) -> None:
    """
    Check that circuits make up a decomposable product.

    :param factors: The factors to multiply.
    :raises EmptyCompositionError: If there is no factor.
    :raises ProductOverSharedVariablesError: If several factors are over one variable.
    """
    if not factors:
        raise EmptyCompositionError(IntervenableCircuit.product_of)
    seen = SortedSet()
    shared = SortedSet()
    for factor in factors:
        shared |= seen & SortedSet(factor.variables)
        seen |= SortedSet(factor.variables)
    if shared:
        raise ProductOverSharedVariablesError(list(shared))


# %% the circuit type


@dataclass(eq=False)
class IntervenableCircuit(ProbabilisticModel, ABC):
    """
    A circuit type that causal circuits read and build interventional distributions in.

    An interventional distribution is a mixture of products of the circuits a cause and
    an effect are read out of, so a circuit type that offers both compositions lets a
    causal query be answered without the causal circuit knowing how the nodes are
    stored.
    """

    @classmethod
    @abstractmethod
    def mixture_of(cls, components: List[Self], log_weights: Iterable[float]) -> Self:
        """
        Mix copies of circuits over the same variables, normalized.

        The components stay as they are.

        :param components: The components to mix, at least one.
        :param log_weights: The logarithmic weight of every component.
        :return: The mixture.
        :raises EmptyCompositionError: If there is no component.
        :raises NumberOfWeightsMismatchError: If there is not one weight per component.
        :raises MixtureOverDifferentVariablesError: If the components are not over the
            same variables.
        """
        raise NotImplementedError

    @classmethod
    @abstractmethod
    def product_of(cls, factors: List[Self]) -> Self:
        """
        Multiply copies of circuits over disjoint variables.

        The factors stay as they are.

        :param factors: The factors to multiply, at least one.
        :return: The product.
        :raises EmptyCompositionError: If there is no factor.
        :raises ProductOverSharedVariablesError: If several factors are over one
            variable.
        """
        raise NotImplementedError
