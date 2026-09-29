from __future__ import annotations

import functools
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
from random_events.interval import Interval
from random_events.product_algebra import SimpleEvent
from random_events.set import Set
from random_events.sigma_algebra import AbstractCompositeSet
from random_events.variable import Symbolic, Variable
from sortedcontainers import SortedSet
from typing_extensions import Any, Dict, List, Optional, Self, Tuple, Type

from probabilistic_model.distributions.distributions import (
    DiscreteDistribution,
    IntegerDistribution,
    SymbolicDistribution,
)
from probabilistic_model.exceptions import ShapeMismatchError
from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    NodeMask,
    NodeStateValues,
    NodeValues,
    SampleArray,
    SampleColumn,
    SampleNodeValues,
    StateIndices,
    StateMask,
    States,
)
from probabilistic_model.probabilistic_circuit.tensorized.exceptions import (
    UndefinedCumulativeDistributionError,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import Layer
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.base import (
    InputLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.query_cache import (
    QueryCache,
    memoized,
)
from probabilistic_model.probabilistic_circuit.tensorized.structural_query import (
    LayerWithLogProbabilities,
)
from probabilistic_model.probabilistic_circuit.tensorized.utils import (
    embedded_logsumexp,
)
from probabilistic_model.utils import MissingDict


@dataclass(eq=False, repr=False)
class DiscreteLayer(InputLayer, ABC):
    """
    Abstract base class for the input layers of discrete univariate distributions.

    The probability of every state of the variable is stored for every node, so that a
    likelihood is a single gather from a (#nodes, #states) block.
    """

    states: States
    """
    The states of the variable, sorted ascending.
    """

    log_probabilities: NodeStateValues
    """
    The logarithmic probability of every state for every node.
    """

    @property
    def number_of_nodes(self) -> int:
        return self.log_probabilities.shape[0]

    @property
    def number_of_states(self) -> int:
        """
        :return: The number of states of the variable.
        """
        return len(self.states)

    @property
    def number_of_own_parameters(self) -> int:
        return int(self.log_probabilities.size)

    @property
    def probabilities(self) -> NodeStateValues:
        """
        :return: The probabilities of every state for every node in linear space.
        """
        return np.exp(self.log_probabilities)

    def validate_own(self):
        if self.log_probabilities.shape[1] != self.number_of_states:
            raise ShapeMismatchError(
                (self.number_of_nodes, self.number_of_states),
                self.log_probabilities.shape,
            )

    @abstractmethod
    def selected_states(self, assignment: AbstractCompositeSet) -> StateMask:
        """
        :param assignment: The assignment of the variable of this layer.
        :return: The states the assignment contains.
        """
        raise NotImplementedError

    def state_indices_of(self, values: SampleColumn) -> StateIndices:
        """
        Look up the index of every value in :attr:`states`.

        :param values: The values of the variable as the layers read them: the value
            itself for an integer variable and the position of the domain element for a
            symbolic variable, see :class:`SymbolicEncoding`.
        :return: The index of every value, or ``-1`` for values that are not a state.
        """
        values = np.asarray(values, dtype=float).reshape(-1)
        states = self.states.astype(float)
        positions = np.searchsorted(states, values)
        positions = np.clip(positions, 0, max(self.number_of_states - 1, 0))
        found = states[positions] == values
        return np.where(found, positions, -1)

    @memoized
    def log_likelihood_of_nodes(
        self, events: SampleArray, cache: Optional[QueryCache] = None
    ) -> SampleNodeValues:
        indices = self.state_indices_of(self.column_of(events))
        result = np.full((len(indices), self.number_of_nodes), -np.inf)
        known = indices >= 0
        if known.any():
            result[known] = self.log_probabilities[:, indices[known]].T
        return result

    @memoized
    def cumulative_distribution_of_nodes(
        self, events: SampleArray, cache: Optional[QueryCache] = None
    ) -> SampleNodeValues:
        raise UndefinedCumulativeDistributionError(self.__class__)

    @memoized
    def probability_of_simple_event_of_nodes(
        self,
        event: SimpleEvent,
        variables: SortedSet,
        cache: Optional[QueryCache] = None,
    ) -> NodeValues:
        selected = self.selected_states(event[variables[self.variable]])
        return self.probabilities[:, selected].sum(axis=1)

    def type_of_truncated_layer(
        self, assignment: AbstractCompositeSet, singleton_allowed: bool
    ) -> Type[Layer]:
        return self.__class__

    def log_truncated_of_assignment(
        self, assignment: AbstractCompositeSet, singleton_allowed: bool
    ) -> LayerWithLogProbabilities:
        """
        Truncating a discrete distribution keeps the probabilities of the states the
        assignment contains and renormalizes, which is one masked row-sum for the whole
        layer.

        :param assignment: The assignment of the variable of this layer.
        :param singleton_allowed: Whether the truncation may leave a single state; a
            discrete layer handles that case like any other.
        :return: The truncated layer and the probability of the assignment under every
            node, in log space.
        """
        return self.renormalized_to(self.selected_states(assignment))

    def log_conditional_of_value(self, value: Any) -> LayerWithLogProbabilities:
        """
        Conditioning on a value is truncating to the state of that value.

        :param value: The value as the layer reads it, see :meth:`state_indices_of`.
        :return: The conditioned layer and the log-probability of the value under every
            node.
        """
        selected = np.zeros(self.number_of_states, dtype=bool)
        [index] = self.state_indices_of(np.array([value], dtype=float))
        if index >= 0:
            selected[index] = True
        return self.renormalized_to(selected)

    def renormalized_to(self, selected: StateMask) -> LayerWithLogProbabilities:
        """
        :param selected: The states to keep.
        :return: The layer with the probability of every other state set to zero and
            renormalized, and the log-probability of the kept states under every node.
            A node without probability for the kept states keeps its parameters and is
            removed by the prune pass.
        """
        probabilities = np.where(selected, self.probabilities, 0.0)
        total = probabilities.sum(axis=1)
        alive = total > 0

        with np.errstate(divide="ignore", invalid="ignore"):
            log_probabilities = np.log(
                probabilities / np.where(alive, total, 1.0)[:, None]
            )
            node_log_probabilities = np.where(
                alive, np.log(np.where(alive, total, 1.0)), -np.inf
            )

        log_probabilities = np.where(
            alive[:, None], log_probabilities, self.log_probabilities
        )
        return LayerWithLogProbabilities(
            self.with_parameters(self.states.copy(), log_probabilities),
            node_log_probabilities,
        )

    def normalize_own(self):
        self.log_probabilities = self.log_probabilities - embedded_logsumexp(
            self.log_probabilities, axis=1
        ).reshape(-1, 1)

    def probabilities_of_node(self, node: int) -> MissingDict:
        """
        :param node: The index of a node.
        :return: The probability of every state with a non-zero probability.
        """
        return MissingDict(
            float,
            {
                int(state): float(probability)
                for state, probability in zip(self.states, self.probabilities[node])
                if probability > 0
            },
        )

    def with_parameters(
        self, states: States, log_probabilities: NodeStateValues
    ) -> Self:
        """
        :param states: The states of the new layer.
        :param log_probabilities: The logarithmic probability of every state for every
            node of the new layer.
        :return: A layer over the same variable as this one with these parameters.
        """
        return self.__class__(self.variable, states, log_probabilities)

    @classmethod
    def from_distributions(
        cls, variable_index: int, distributions: List[DiscreteDistribution]
    ) -> Self:
        return cls(variable_index, *cls.parameters_of(distributions))

    @classmethod
    def parameters_of(
        cls, distributions: List[DiscreteDistribution]
    ) -> Tuple[States, NodeStateValues]:
        """
        :param distributions: Distributions over the variable of this layer.
        :return: The states that any of the distributions has, and the logarithmic
            probability of every state under every distribution.
        """
        probabilities_by_state = [
            cls.probabilities_by_state_of(distribution)
            for distribution in distributions
        ]
        states = sorted(
            {
                state
                for probabilities in probabilities_by_state
                for state in probabilities
            }
        )
        state_to_column = {state: index for index, state in enumerate(states)}

        probabilities = np.zeros((len(distributions), len(states)))
        for row, probabilities_of_row in enumerate(probabilities_by_state):
            for state, probability in probabilities_of_row.items():
                probabilities[row, state_to_column[state]] = probability

        with np.errstate(divide="ignore"):
            log_probabilities = np.log(probabilities)
        return np.array(states, dtype=np.int64), log_probabilities

    @classmethod
    def probabilities_by_state_of(
        cls, distribution: DiscreteDistribution
    ) -> Dict[int, float]:
        """
        :param distribution: A distribution over the variable of this layer.
        :return: The probability of every state of the distribution, keyed by the state
            as this layer stores it.
        """
        return dict(distribution.probabilities)

    def select_nodes(self, mask: NodeMask) -> Self:
        return self.with_parameters(self.states.copy(), self.log_probabilities[mask])

    @classmethod
    def concatenate(cls, layers: List[Self]) -> Self:
        """
        Truncating a discrete layer never changes its states, so the probability blocks
        of the layers line up.

        :param layers: Layers with the same variable and states.
        :return: One layer with the nodes of all layers, in order.
        """
        return layers[0].with_parameters(
            layers[0].states.copy(),
            np.concatenate([layer.log_probabilities for layer in layers]),
        )

    def sample_of_node(
        self, node: int, amount: int, variables: SortedSet
    ) -> SampleColumn:
        probabilities = self.probabilities[node]
        total = probabilities.sum()
        if total <= 0:
            return np.full(amount, np.nan)
        return np.random.choice(self.states, size=amount, p=probabilities / total)

    def __deepcopy__(self, memo: Optional[Dict[int, Any]] = None) -> Self:
        """
        :param memo: The copies made so far, keyed by the id of the original.
        :return: A copy of this layer that shares no arrays with it.
        """
        if memo is None:
            memo = {}
        if id(self) in memo:
            return memo[id(self)]
        result = self.with_parameters(self.states.copy(), self.log_probabilities.copy())
        memo[id(self)] = result
        return result


@dataclass
class SymbolicEncoding:
    """
    The translation between the two representations of a value of a symbolic variable.

    The events and samples of this package hold the hash of a domain element. A symbolic
    layer holds the position of the element in the domain instead, a small integer that
    it looks up without hashing. The layered circuit encodes its input once per query
    and decodes its samples.
    """

    variable: Symbolic
    """
    The symbolic variable.
    """

    @functools.cached_property
    def elements(self) -> Tuple[Any, ...]:
        """
        :return: The domain elements of the variable, in the order that defines their
            positions.
        """
        return tuple(
            simple_set.element for simple_set in self.variable.domain.simple_sets
        )

    @functools.cached_property
    def hashes(self) -> SampleColumn:
        """
        :return: The hash of every domain element, at its position.
        """
        return np.array([hash(element) for element in self.elements], dtype=float)

    def indices_of_hashes(self, values: SampleColumn) -> StateIndices:
        """
        :param values: Values of the variable as the events of this package hold them.
        :return: The position of every value in the domain, or ``-1`` for a value that
            is not the hash of a domain element.
        """
        values = np.asarray(values, dtype=float).reshape(-1)
        order = np.argsort(self.hashes)
        sorted_hashes = self.hashes[order]
        positions = np.clip(np.searchsorted(sorted_hashes, values), 0, len(order) - 1)
        found = sorted_hashes[positions] == values
        return np.where(found, order[positions], -1)

    def hashes_of_indices(self, indices: StateIndices) -> SampleColumn:
        """
        :param indices: Positions of domain elements.
        :return: The value of every position as the events of this package hold it.
        """
        return self.hashes[np.asarray(indices, dtype=np.int64)]

    def index_of_element(self, element: Any) -> int:
        """
        :param element: A domain element, or its hash.
        :return: The position of the element in the domain, or ``-1``.
        """
        return int(self.indices_of_hashes(np.array([hash(element)], dtype=float))[0])


@dataclass(eq=False, repr=False)
class SymbolicLayer(DiscreteLayer):
    """
    A layer of categorical distributions over one symbolic variable.

    The states are the positions of the domain elements, see :class:`SymbolicEncoding`.
    """

    domain_hashes: SampleColumn
    """
    The hash of every domain element of the variable, at the position of the element.
    """

    def with_parameters(
        self, states: States, log_probabilities: NodeStateValues
    ) -> Self:
        return self.__class__(
            self.variable, states, log_probabilities, self.domain_hashes
        )

    @classmethod
    def from_distributions(
        cls, variable_index: int, distributions: List[SymbolicDistribution]
    ) -> Self:
        return cls(
            variable_index,
            *cls.parameters_of(distributions),
            SymbolicEncoding(distributions[0].variable).hashes,
        )

    @classmethod
    def probabilities_by_state_of(
        cls, distribution: SymbolicDistribution
    ) -> Dict[int, float]:
        encoding = SymbolicEncoding(distribution.variable)
        return {
            encoding.index_of_element(hash_value): probability
            for hash_value, probability in distribution.probabilities.items()
        }

    def node_distribution(self, index: int, variable: Variable) -> SymbolicDistribution:
        return SymbolicDistribution(
            variable=variable,
            probabilities=MissingDict(
                float,
                {
                    int(self.domain_hashes[state]): probability
                    for state, probability in self.probabilities_of_node(index).items()
                },
            ),
        )

    def selected_states(self, assignment: Set) -> StateMask:
        hashes = np.array(
            [hash(simple_set) for simple_set in assignment.simple_sets], dtype=float
        )
        return np.isin(self.domain_hashes[self.states], hashes)


@dataclass(eq=False, repr=False)
class IntegerLayer(DiscreteLayer):
    """
    A layer of distributions over one integer variable.
    """

    def node_distribution(self, index: int, variable: Variable) -> IntegerDistribution:
        return IntegerDistribution(
            variable=variable, probabilities=self.probabilities_of_node(index)
        )

    def selected_states(self, assignment: Interval) -> StateMask:
        return np.array(
            [state in assignment for state in self.states.tolist()], dtype=bool
        )

    @memoized
    def cumulative_distribution_of_nodes(
        self, events: SampleArray, cache: Optional[QueryCache] = None
    ) -> SampleNodeValues:
        column = np.asarray(self.column_of(events), dtype=float).reshape(-1, 1)
        reached = column >= self.states.reshape(1, -1)
        return reached.astype(float) @ self.probabilities.T

    def moment_of_nodes_own(
        self, order: int, center: float, variable: Variable
    ) -> NodeValues:
        deviations = (self.states.astype(float) - center) ** order
        return self.probabilities @ deviations
