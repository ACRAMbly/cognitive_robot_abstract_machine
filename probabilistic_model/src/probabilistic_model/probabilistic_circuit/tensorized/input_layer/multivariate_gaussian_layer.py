from __future__ import annotations

import itertools
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
from random_events.interval import Bound, SimpleInterval, singleton
from random_events.product_algebra import Event, SimpleEvent
from random_events.variable import Variable
from scipy.special import ndtr
from scipy.stats import multivariate_normal, truncnorm
from sortedcontainers import SortedSet
from typing_extensions import Any, Dict, List, Optional, Self, Sequence, Tuple, Type

from probabilistic_model.distributions.multivariate_gaussian import (
    Covariance,
    MultivariateGaussianDistribution,
)
from probabilistic_model.distributions.truncated_multivariate_gaussian import (
    TruncatedMultivariateGaussianDistribution,
)
from probabilistic_model.exceptions import ShapeMismatchError
from probabilistic_model.probabilistic_model import PartialPointType
from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    NodeIndices,
    NodeMask,
    NodeScopeIntervalBounds,
    NodeScopeIntervals,
    NodeScopeMatrices,
    NodeScopeValues,
    NodeValues,
    NodeVariableValues,
    SampleArray,
    SampleNodeMask,
    SampleNodeValues,
    SampleScopeValues,
    VariableIndices,
    VariableMask,
    VariableValues,
)
from probabilistic_model.probabilistic_circuit.tensorized.exceptions import (
    NoClosedFormError,
)
from probabilistic_model.probabilistic_circuit.tensorized.forward_sample_assignment import (
    ForwardSampleAssignment,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import Layer
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.product_layer import (
    ProductLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.dirac_delta_layer import (
    DiracDeltaLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.gaussian_layer import (
    GaussianLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.moment_query import (
    MomentQuery,
)
from probabilistic_model.probabilistic_circuit.tensorized.query_cache import (
    QueryCache,
    memoized,
)
from probabilistic_model.probabilistic_circuit.tensorized.structural_query import (
    LayerWithLogProbabilities,
    StructuralQuery,
)

# %% boxes over the scope of a layer


@dataclass
class HyperrectangleArray:
    """
    Axis-aligned boxes over the variables in the scope of a layer, one simple interval
    per variable and box.

    The leading axes enumerate the boxes: a single box has the shape
    (#variables of the layer, 2), one box per node (#nodes, #variables of the layer, 2).
    """

    interval: NodeScopeIntervals
    """
    The lower and upper bound of every interval.
    """

    bounds: NodeScopeIntervalBounds
    """
    Whether the lower and upper bound of every interval are open or closed.
    """

    @classmethod
    def of_simple_intervals(cls, intervals: Sequence[SimpleInterval]) -> Self:
        """
        :param intervals: One simple interval per variable in the scope of a layer.
        :return: The box they span.
        """
        return cls(
            np.array(
                [[interval.lower, interval.upper] for interval in intervals],
                dtype=float,
            ),
            np.array(
                [[int(interval.left), int(interval.right)] for interval in intervals],
                dtype=np.int64,
            ),
        )

    @property
    def lower(self) -> np.ndarray:
        return self.interval[..., 0]

    @property
    def upper(self) -> np.ndarray:
        return self.interval[..., 1]

    @property
    def is_whole_space(self) -> bool:
        """
        :return: Whether every box leaves every variable unbounded.
        """
        return bool(np.all(self.lower == -np.inf) and np.all(self.upper == np.inf))

    def intersection_with(self, other: HyperrectangleArray) -> HyperrectangleArray:
        """
        :param other: Hyperrectangles whose shape broadcasts against these.
        :return: The intersection of every box with the matching box of ``other``.
            Where two bounds coincide the result is open if either of them is.
            :attr:`Bound.OPEN` is the larger value, so that is a maximum.
        """
        own_lower, other_lower = np.broadcast_arrays(self.lower, other.lower)
        own_upper, other_upper = np.broadcast_arrays(self.upper, other.upper)
        own_left, other_left = np.broadcast_arrays(
            self.bounds[..., 0], other.bounds[..., 0]
        )
        own_right, other_right = np.broadcast_arrays(
            self.bounds[..., 1], other.bounds[..., 1]
        )
        left = np.where(
            own_lower > other_lower,
            own_left,
            np.where(
                own_lower < other_lower, other_left, np.maximum(own_left, other_left)
            ),
        )
        right = np.where(
            own_upper < other_upper,
            own_right,
            np.where(
                own_upper > other_upper,
                other_right,
                np.maximum(own_right, other_right),
            ),
        )
        return HyperrectangleArray(
            np.stack(
                [
                    np.maximum(own_lower, other_lower),
                    np.minimum(own_upper, other_upper),
                ],
                axis=-1,
            ),
            np.stack([left, right], axis=-1),
        )

    def contains(self, values: SampleScopeValues) -> SampleNodeMask:
        """
        :param values: Points over the variables of the boxes, shape (#samples,
            #variables of the layer).
        :return: Whether every box, one per node, contains every point, shape
            (#samples, #nodes).
        """
        points = values[:, None, :]
        left_closed = self.bounds[..., 0] == int(Bound.CLOSED)
        right_closed = self.bounds[..., 1] == int(Bound.CLOSED)
        left = np.where(left_closed, self.lower <= points, self.lower < points)
        right = np.where(right_closed, points <= self.upper, points < self.upper)
        return (left & right).all(axis=-1)

    def simple_event_of(self, index: int, variables: Sequence[Variable]) -> SimpleEvent:
        """
        :param index: The index of a box along the first axis.
        :param variables: The variables of the box.
        :return: That box as a simple event.
        """
        return SimpleEvent.from_data(
            {
                variable: SimpleInterval.from_data(
                    float(self.interval[index, position, 0]),
                    float(self.interval[index, position, 1]),
                    Bound(int(self.bounds[index, position, 0])),
                    Bound(int(self.bounds[index, position, 1])),
                ).as_composite_set()
                for position, variable in enumerate(variables)
            }
        )

    def select(self, indices: Any) -> HyperrectangleArray:
        """
        :param indices: A mask or index array over the first axis.
        :return: The selected boxes.
        """
        return HyperrectangleArray(self.interval[indices], self.bounds[indices])

    def broadcast_to(self, number_of_boxes: int) -> HyperrectangleArray:
        """
        :param number_of_boxes: How many copies to make of this single box.
        :return: The copies, one per entry of the first axis.
        """
        return HyperrectangleArray(
            np.tile(self.interval, (number_of_boxes, 1, 1)),
            np.tile(self.bounds, (number_of_boxes, 1, 1)),
        )


# %% the parameters every Gaussian layer shares


@dataclass(eq=False, repr=False)
class AbstractMultivariateGaussianLayer(Layer, ABC):
    """
    Abstract base class for the input layers of Gaussians over several continuous
    variables at once.

    Every node is a Gaussian over the same variables, and its mean and covariance are
    laid out in the order of those variables in the circuit.
    """

    scope: VariableIndices
    """
    The sorted indices of the variables of every node.
    """

    mean: NodeScopeValues
    """
    The mean of every node.
    """

    covariance: NodeScopeMatrices
    """
    The covariance matrix of every node.
    """

    @property
    def child_layers(self) -> List[Layer]:
        """
        :return: An empty list. An input layer is a leaf of the layer graph.
        """
        return []

    @property
    def variables(self) -> VariableIndices:
        return self.scope

    @property
    def number_of_nodes(self) -> int:
        return len(self.mean)

    @property
    def number_of_scope_variables(self) -> int:
        """
        :return: How many variables every node is over.
        """
        return len(self.scope)

    @property
    def number_of_own_parameters(self) -> int:
        dimension = self.number_of_scope_variables
        return self.number_of_nodes * (dimension + dimension * (dimension + 1) // 2)

    def validate_own(self):
        dimension = self.number_of_scope_variables
        if self.mean.shape != (self.number_of_nodes, dimension):
            raise ShapeMismatchError(self.mean.shape, (self.number_of_nodes, dimension))
        expected = (self.number_of_nodes, dimension, dimension)
        if self.covariance.shape != expected:
            raise ShapeMismatchError(self.covariance.shape, expected)

    def values_of_scope(self, events: SampleArray) -> SampleScopeValues:
        """
        :param events: The events with shape (#events, #variables of the circuit).
        :return: The columns of the variables of this layer.
        """
        return events[:, self.scope]

    def scope_variables(self, variables: SortedSet) -> List[Variable]:
        """
        :param variables: The variables of the circuit.
        :return: The variables of this layer.
        """
        return [variables[index] for index in self.scope]

    # %% the untruncated Gaussians

    def untruncated_log_density_of_nodes(
        self, values: SampleScopeValues
    ) -> SampleNodeValues:
        """
        :param values: Points over the variables of this layer.
        :return: The log-density of the Gaussian of every node at every point, shape
            (#points, #nodes).
        """
        cholesky = np.linalg.cholesky(self.covariance)
        # whiten by every node at once: one matrix product of the points with the
        # stacked inverse Cholesky factors, shape (#points, #nodes * #variables)
        whitening = np.linalg.inv(cholesky)
        dimension = self.number_of_scope_variables
        stacked = whitening.reshape(-1, dimension)
        whitened_means = np.einsum("nij,nj->ni", whitening, self.mean).reshape(-1)
        whitened = (values @ stacked.T - whitened_means).reshape(
            len(values), self.number_of_nodes, dimension
        )
        log_determinant = 2 * np.log(np.diagonal(cholesky, axis1=1, axis2=2)).sum(
            axis=1
        )
        return -0.5 * (
            np.einsum("snd,snd->sn", whitened, whitened)
            + dimension * math.log(2 * math.pi)
            + log_determinant
        )

    def untruncated_probability_of_boxes(
        self, boxes: HyperrectangleArray
    ) -> NodeValues:
        """
        The probability of an axis-aligned box under a correlated Gaussian has no closed
        form, so it is integrated numerically by :mod:`scipy.stats.multivariate_normal`
        over the variables the box bounds. A box that bounds only one variable is
        answered in closed form for all nodes at once.

        :param boxes: One box per node.
        :return: The probability of the box of every node under the Gaussian of that
            node.
        """
        result = np.zeros(self.number_of_nodes)
        possible = (boxes.lower < boxes.upper).all(axis=1)
        bounded = np.isfinite(boxes.lower) | np.isfinite(boxes.upper)

        for pattern in np.unique(bounded[possible], axis=0):
            nodes = possible & (bounded == pattern).all(axis=1)
            positions = np.flatnonzero(pattern)
            if len(positions) == 0:
                result[nodes] = 1.0
            elif len(positions) == 1:
                [position] = positions
                mean = self.mean[nodes, position]
                deviation = np.sqrt(self.covariance[nodes, position, position])
                result[nodes] = ndtr(
                    (boxes.upper[nodes, position] - mean) / deviation
                ) - ndtr((boxes.lower[nodes, position] - mean) / deviation)
            else:
                for node in np.flatnonzero(nodes):
                    result[node] = multivariate_normal(
                        self.mean[node, positions],
                        self.covariance[node][np.ix_(positions, positions)],
                    ).cdf(
                        boxes.upper[node, positions],
                        lower_limit=boxes.lower[node, positions],
                    )
        return np.clip(result, 0.0, 1.0)

    def boxes_of(
        self, event: SimpleEvent, variables: SortedSet
    ) -> List[HyperrectangleArray]:
        """
        :param event: A simple event.
        :param variables: The variables of the circuit.
        :return: The boxes the event makes of the variables of this layer, one per
            combination of their simple intervals.
        """
        return [
            HyperrectangleArray.of_simple_intervals(intervals)
            for intervals in itertools.product(
                *(
                    event[variable].simple_sets
                    for variable in self.scope_variables(variables)
                )
            )
        ]

    def gaussians_over(self, positions: NodeIndices) -> MultivariateGaussianLayer:
        """
        :param positions: Positions in the scope of this layer, ascending.
        :return: The untruncated Gaussians of the nodes over only those variables.
        """
        return MultivariateGaussianLayer(
            self.scope[positions],
            self.mean[:, positions],
            self.covariance[:, positions][:, :, positions],
        )

    def gaussian_conditionals(
        self, fixed: NodeIndices, free: NodeIndices, values: np.ndarray
    ) -> MultivariateGaussianLayer:
        """
        :param fixed: The positions in the scope of the variables held at a value.
        :param free: The positions in the scope of the other variables.
        :param values: What the fixed variables are held at.
        :return: The untruncated Gaussians of the nodes over the free variables,
            conditioned on the fixed ones.
        """
        free_with_fixed = self.covariance[:, free][:, :, fixed]
        gain = np.swapaxes(
            np.linalg.solve(
                self.covariance[:, fixed][:, :, fixed],
                np.swapaxes(free_with_fixed, 1, 2),
            ),
            1,
            2,
        )
        return MultivariateGaussianLayer(
            self.scope[free],
            self.mean[:, free]
            + np.einsum("nij,nj->ni", gain, values - self.mean[:, fixed]),
            self.covariance[:, free][:, :, free]
            - gain @ np.swapaxes(free_with_fixed, 1, 2),
        )

    def untruncated_distribution(
        self, index: int, variables: SortedSet
    ) -> MultivariateGaussianDistribution:
        """
        :param index: The index of a node.
        :param variables: The variables of the circuit.
        :return: The untruncated Gaussian of that node.
        """
        return MultivariateGaussianDistribution(
            variables=tuple(self.scope_variables(variables)),
            mean=self.mean[index].copy(),
            covariance=Covariance.from_matrix(self.covariance[index]),
        )

    # %% per node view

    @abstractmethod
    def node_distribution(self, index: int, variables: SortedSet) -> Any:
        """
        :param index: The index of a node.
        :param variables: The variables of the circuit.
        :return: The distribution of that node.
        """
        raise NotImplementedError

    def node_distributions(self, variables: SortedSet) -> List[Any]:
        """
        :param variables: The variables of the circuit.
        :return: The distribution of every node.
        """
        return [
            self.node_distribution(index, variables)
            for index in range(self.number_of_nodes)
        ]

    @abstractmethod
    def select_nodes(self, mask: NodeMask) -> Self:
        """
        :param mask: A boolean mask over the nodes of this layer.
        :return: A layer that only holds the selected nodes.
        """
        raise NotImplementedError

    @classmethod
    @abstractmethod
    def concatenate(cls, layers: List[Self]) -> Self:
        """
        Join layers of this type over the same variables into one layer, the nodes of
        ``layers[k]`` as one contiguous block.

        :param layers: The layers to join.
        :return: The joined layer.
        """
        raise NotImplementedError

    # %% queries

    @memoized
    def support_of_nodes(
        self, variables: SortedSet, cache: Optional[QueryCache] = None
    ) -> List[Event]:
        return [
            distribution.support for distribution in self.node_distributions(variables)
        ]

    def sample_forward(
        self,
        assignment: ForwardSampleAssignment,
        samples: SampleArray,
        variables: SortedSet,
    ):
        rows, nodes = [], []
        for node, rows_of_node in enumerate(assignment.rows_of(self)):
            if rows_of_node.is_empty:
                continue
            rows.append(rows_of_node.rows)
            nodes.append(np.full(len(rows_of_node.rows), node))
        if not rows:
            return
        rows = np.concatenate(rows)
        samples[rows[:, None], self.scope[None, :]] = self.samples_of_nodes(
            np.concatenate(nodes)
        )

    @abstractmethod
    def samples_of_nodes(self, nodes: NodeIndices) -> SampleScopeValues:
        """
        :param nodes: The node to draw each sample from.
        :return: One sample per entry of ``nodes``.
        """
        raise NotImplementedError

    # %% structural

    @abstractmethod
    def type_of_layer_truncated_to_box(self, box: HyperrectangleArray) -> Type[Layer]:
        """
        :param box: A single box.
        :return: The type of the layer :meth:`log_truncated_of_box` returns for it.
        """
        raise NotImplementedError

    @abstractmethod
    def log_truncated_of_box(
        self, box: HyperrectangleArray
    ) -> LayerWithLogProbabilities:
        """
        Truncate every node to the same box.

        :param box: A single box.
        :return: The truncated layer, with as many nodes as this one, and the log-
            probability of the box under every node.
        """
        raise NotImplementedError

    def log_truncated_of_boxes(
        self, boxes: List[HyperrectangleArray]
    ) -> LayerWithLogProbabilities:
        """
        Truncate every node to the union of disjoint boxes.

        :param boxes: The boxes.
        :return: The truncated layer and the log-probabilities of its nodes. A node
            truncated to several boxes becomes a mixture of its truncations.
        """
        if not boxes:
            return LayerWithLogProbabilities(
                self.__deepcopy__(), np.full(self.number_of_nodes, -np.inf)
            )
        pieces = [self.log_truncated_of_box(box) for box in boxes]
        if len(pieces) == 1:
            return pieces[0]
        return SumLayer.mixture_of_pieces(pieces)

    def type_of_truncated_layer(
        self, event: SimpleEvent, variables: SortedSet
    ) -> Type[Layer]:
        """
        :param event: A simple event.
        :param variables: The variables of the circuit.
        :return: The type of the layer truncating to the event creates.
        """
        boxes = self.boxes_of(event, variables)
        if len(boxes) > 1:
            return SumLayer
        if not boxes:
            return self.__class__
        return self.type_of_layer_truncated_to_box(boxes[0])

    @memoized
    def log_truncated_of_simple_event(
        self,
        event: SimpleEvent,
        query: StructuralQuery,
        cache: Optional[QueryCache] = None,
    ) -> LayerWithLogProbabilities:
        # a Gaussian gives every single point probability zero, so a singleton makes a
        # node impossible whether singletons are allowed or not
        return query.log_probabilities.record(
            self.log_truncated_of_boxes(self.boxes_of(event, query.variables))
        )

    def can_truncate_in_one_batch(
        self, events: List[SimpleEvent], query: StructuralQuery
    ) -> bool:
        # the truncations are joined with concatenate, which needs them all to be
        # layers of one Gaussian type
        types = {
            self.type_of_truncated_layer(event, query.variables) for event in events
        }
        return len(types) == 1 and issubclass(
            types.pop(), AbstractMultivariateGaussianLayer
        )

    @memoized
    def log_truncated_of_simple_events(
        self,
        events: List[SimpleEvent],
        query: StructuralQuery,
        cache: Optional[QueryCache] = None,
    ) -> LayerWithLogProbabilities:
        truncated = [
            self.log_truncated_of_boxes(self.boxes_of(event, query.variables))
            for event in events
        ]
        layers = [piece.layer for piece in truncated]
        return query.log_probabilities.record(
            LayerWithLogProbabilities(
                type(layers[0]).concatenate(layers),
                np.concatenate([piece.log_probabilities for piece in truncated]),
            )
        )

    @memoized
    def log_conditional_of_point(
        self,
        point: PartialPointType,
        query: StructuralQuery,
        cache: Optional[QueryCache] = None,
    ) -> LayerWithLogProbabilities:
        # the fixed variables become Dirac deltas next to the Gaussians of the others,
        # conditioned on them
        variables = self.scope_variables(query.variables)
        fixed = np.array(
            [
                position
                for position, variable in enumerate(variables)
                if variable in point
            ],
            dtype=np.int64,
        )
        if len(fixed) == 0:
            return query.log_probabilities.record(
                LayerWithLogProbabilities(
                    self.__deepcopy__(), np.zeros(self.number_of_nodes)
                )
            )

        values = np.array([float(point[variables[position]]) for position in fixed])
        free = np.setdiff1d(np.arange(self.number_of_scope_variables), fixed)
        conditioned = self.log_conditional_of_values(fixed, free, values)

        point_masses = [
            DiracDeltaLayer(
                int(self.scope[position]),
                np.full(self.number_of_nodes, value),
                np.ones(self.number_of_nodes),
            )
            for position, value in zip(fixed, values)
        ]
        factors = point_masses if len(free) == 0 else [conditioned.layer] + point_masses
        return query.log_probabilities.record(
            LayerWithLogProbabilities(
                ProductLayer.node_wise_product_of(factors),
                conditioned.log_probabilities,
            )
        )

    @abstractmethod
    def log_conditional_of_values(
        self, fixed: NodeIndices, free: NodeIndices, values: np.ndarray
    ) -> LayerWithLogProbabilities:
        """
        :param fixed: The positions in the scope of the variables held at a value.
        :param free: The positions in the scope of the other variables, possibly none.
        :param values: What the fixed variables are held at.
        :return: The layer over the free variables conditioned on the values, which is
            meaningless without free variables, and the log-likelihood of the values
            under every node.
        """
        raise NotImplementedError

    def rebuild(
        self,
        needed: Dict[int, NodeMask],
        rebuilt: Dict[int, Optional[Layer]],
    ) -> Optional[Layer]:
        alive = needed[id(self)]
        if not alive.any():
            return None
        return self.select_nodes(alive)

    def marginal(
        self, kept: VariableMask, cache: Optional[QueryCache] = None
    ) -> Optional[Layer]:
        positions = np.flatnonzero(kept[self.scope])
        if len(positions) == 0:
            return None
        if len(positions) == self.number_of_scope_variables:
            return self.__deepcopy__()
        return self.marginal_over(positions)

    @abstractmethod
    def marginal_over(self, positions: NodeIndices) -> Layer:
        """
        :param positions: Some, but not all, positions in the scope, ascending.
        :return: The marginal of every node over those variables.
        """
        raise NotImplementedError

    @memoized
    def remap_variables(
        self, remap: VariableIndices, cache: Optional[QueryCache] = None
    ):
        remapped = remap[self.scope]
        order = np.argsort(remapped)
        self.scope = remapped[order]
        self.reorder_scope(order)

    def reorder_scope(self, order: NodeIndices):
        """
        Lay the parameters out in a new order of the variables, in place.

        :param order: The old position of every variable, in the new order.
        """
        self.mean = self.mean[:, order]
        self.covariance = self.covariance[:, order][:, :, order]

    def apply_translation_own(self, translation: VariableValues):
        self.mean = self.mean + translation[self.scope]

    def apply_scaling_own(self, scaling: VariableValues):
        factors = scaling[self.scope]
        self.mean = self.mean * factors
        self.covariance = self.covariance * factors[:, None] * factors[None, :]


# %% Gaussians


@dataclass(eq=False, repr=False)
class MultivariateGaussianLayer(AbstractMultivariateGaussianLayer):
    """
    A layer of Gaussians over several continuous variables at once.
    """

    def node_distribution(
        self, index: int, variables: SortedSet
    ) -> MultivariateGaussianDistribution:
        return self.untruncated_distribution(index, variables)

    @classmethod
    def from_distributions(
        cls,
        variables: SortedSet,
        distributions: List[MultivariateGaussianDistribution],
    ) -> Self:
        """
        :param variables: The variables of the circuit.
        :param distributions: Gaussians over the same variables, in any order.
        :return: The layer with one node per distribution.
        """
        scope = np.sort(
            [variables.index(variable) for variable in distributions[0].variables]
        )
        means, covariances = [], []
        for distribution in distributions:
            order = [distribution.index_of(variables[index]) for index in scope]
            means.append(distribution.mean[order])
            covariances.append(distribution.covariance.matrix[np.ix_(order, order)])
        return cls(scope.astype(np.int64), np.array(means), np.array(covariances))

    def select_nodes(self, mask: NodeMask) -> Self:
        return self.__class__(self.scope.copy(), self.mean[mask], self.covariance[mask])

    @classmethod
    def concatenate(cls, layers: List[Self]) -> Self:
        return cls(
            layers[0].scope.copy(),
            np.concatenate([layer.mean for layer in layers]),
            np.concatenate([layer.covariance for layer in layers]),
        )

    # %% queries

    @memoized
    def log_likelihood_of_nodes(
        self, events: SampleArray, cache: Optional[QueryCache] = None
    ) -> SampleNodeValues:
        return self.untruncated_log_density_of_nodes(self.values_of_scope(events))

    @memoized
    def cumulative_distribution_of_nodes(
        self, events: SampleArray, cache: Optional[QueryCache] = None
    ) -> SampleNodeValues:
        values = self.values_of_scope(events)
        return np.stack(
            [
                np.atleast_1d(
                    multivariate_normal(self.mean[node], self.covariance[node]).cdf(
                        values
                    )
                )
                for node in range(self.number_of_nodes)
            ],
            axis=1,
        )

    @memoized
    def probability_of_simple_event_of_nodes(
        self,
        event: SimpleEvent,
        variables: SortedSet,
        cache: Optional[QueryCache] = None,
    ) -> NodeValues:
        return sum(
            (
                self.untruncated_probability_of_boxes(
                    box.broadcast_to(self.number_of_nodes)
                )
                for box in self.boxes_of(event, variables)
            ),
            np.zeros(self.number_of_nodes),
        )

    @memoized
    def log_mode_of_nodes(
        self, variables: SortedSet, cache: Optional[QueryCache] = None
    ) -> Tuple[List[Event], NodeValues]:
        # a Gaussian is most dense at its mean
        scope_variables = self.scope_variables(variables)
        modes = [
            SimpleEvent.from_data(
                {
                    variable: singleton(float(value))
                    for variable, value in zip(scope_variables, mean)
                }
            ).as_composite_set()
            for mean in self.mean
        ]
        _, log_determinant = np.linalg.slogdet(self.covariance)
        return modes, -0.5 * (
            self.number_of_scope_variables * math.log(2 * math.pi) + log_determinant
        )

    @memoized
    def moment_of_nodes(
        self,
        query: MomentQuery,
        variables: SortedSet,
        cache: Optional[QueryCache] = None,
    ) -> NodeVariableValues:
        # the moment of one variable is the moment of its univariate marginal
        result = np.zeros((self.number_of_nodes, query.number_of_variables))
        for position, index in enumerate(self.scope):
            if not query.requested[index]:
                continue
            marginal = GaussianLayer(
                int(index),
                self.mean[:, position],
                np.sqrt(self.covariance[:, position, position]),
            )
            result[:, index] = marginal.moment_of_nodes_own(
                int(query.order[index]), float(query.center[index]), variables[index]
            )
        return result

    def samples_of_nodes(self, nodes: NodeIndices) -> SampleScopeValues:
        standard = np.random.standard_normal(
            (len(nodes), self.number_of_scope_variables)
        )
        cholesky = np.linalg.cholesky(self.covariance)
        return self.mean[nodes] + np.einsum("rij,rj->ri", cholesky[nodes], standard)

    # %% structural

    def type_of_layer_truncated_to_box(self, box: HyperrectangleArray) -> Type[Layer]:
        if box.is_whole_space:
            return MultivariateGaussianLayer
        return TruncatedMultivariateGaussianLayer

    def log_truncated_of_box(
        self, box: HyperrectangleArray
    ) -> LayerWithLogProbabilities:
        # the whole space leaves every node a Gaussian, any other box confines it
        if self.type_of_layer_truncated_to_box(box) is MultivariateGaussianLayer:
            return LayerWithLogProbabilities(
                self.__deepcopy__(), np.zeros(self.number_of_nodes)
            )
        boxes = box.broadcast_to(self.number_of_nodes)
        probability = self.untruncated_probability_of_boxes(boxes)
        alive = probability > 0
        log_probabilities = np.where(
            alive, np.log(np.where(alive, probability, 1.0)), -np.inf
        )
        return LayerWithLogProbabilities(
            TruncatedMultivariateGaussianLayer(
                self.scope.copy(),
                self.mean.copy(),
                self.covariance.copy(),
                boxes.interval,
                boxes.bounds,
                log_probabilities.copy(),
            ),
            log_probabilities,
        )

    def log_conditional_of_values(
        self, fixed: NodeIndices, free: NodeIndices, values: np.ndarray
    ) -> LayerWithLogProbabilities:
        log_likelihood = self.gaussians_over(fixed).untruncated_log_density_of_nodes(
            values[None, :]
        )[0]
        if len(free) == 0:
            return LayerWithLogProbabilities(self.__deepcopy__(), log_likelihood)
        return LayerWithLogProbabilities(
            self.gaussian_conditionals(fixed, free, values), log_likelihood
        )

    def marginal_over(self, positions: NodeIndices) -> Layer:
        return self.gaussians_over(positions)

    def __deepcopy__(self, memo=None) -> MultivariateGaussianLayer:
        if memo is None:
            memo = {}
        if id(self) in memo:
            return memo[id(self)]
        result = self.__class__(
            self.scope.copy(), self.mean.copy(), self.covariance.copy()
        )
        memo[id(self)] = result
        return result


# %% Gaussians confined to a box


@dataclass(eq=False, repr=False)
class TruncatedMultivariateGaussianLayer(AbstractMultivariateGaussianLayer):
    """
    A layer of Gaussians over several continuous variables, each confined to a box.

    This is the layer that truncating a :class:`MultivariateGaussianLayer` to a box
    produces.
    """

    interval: NodeScopeIntervals
    """
    The lower and upper bound of the box of every node, per variable.
    """

    bounds: NodeScopeIntervalBounds
    """
    Whether the bounds of the box of every node are open or closed.
    """

    log_normalizing_constant: NodeValues
    """
    The log-probability of the box of every node under its untruncated Gaussian.
    """

    burn_in_period_length: int = 100
    """
    How many times each chain of the sampler draws every variable before its last state
    becomes a sample.
    """

    @property
    def boxes(self) -> HyperrectangleArray:
        """
        :return: The box of every node.
        """
        return HyperrectangleArray(self.interval, self.bounds)

    @property
    def number_of_own_parameters(self) -> int:
        return (
            super().number_of_own_parameters
            + 2 * self.number_of_nodes * self.number_of_scope_variables
        )

    def validate_own(self):
        super().validate_own()
        expected = (self.number_of_nodes, self.number_of_scope_variables, 2)
        if self.interval.shape != expected:
            raise ShapeMismatchError(self.interval.shape, expected)
        if self.bounds.shape != expected:
            raise ShapeMismatchError(self.bounds.shape, expected)
        if self.log_normalizing_constant.shape != (self.number_of_nodes,):
            raise ShapeMismatchError(
                self.log_normalizing_constant.shape, (self.number_of_nodes,)
            )

    def node_distribution(
        self, index: int, variables: SortedSet
    ) -> TruncatedMultivariateGaussianDistribution:
        return TruncatedMultivariateGaussianDistribution(
            untruncated=self.untruncated_distribution(index, variables),
            box=self.boxes.simple_event_of(index, self.scope_variables(variables)),
            burn_in_period_length=self.burn_in_period_length,
        )

    @classmethod
    def from_distributions(
        cls,
        variables: SortedSet,
        distributions: List[TruncatedMultivariateGaussianDistribution],
    ) -> Self:
        """
        :param variables: The variables of the circuit.
        :param distributions: Truncated Gaussians over the same variables, in any order.
        :return: The layer with one node per distribution.
        """
        untruncated = MultivariateGaussianLayer.from_distributions(
            variables, [distribution.untruncated for distribution in distributions]
        )
        scope_variables = untruncated.scope_variables(variables)
        boxes = [
            HyperrectangleArray.of_simple_intervals(
                [distribution.interval_of(variable) for variable in scope_variables]
            )
            for distribution in distributions
        ]
        return cls(
            untruncated.scope,
            untruncated.mean,
            untruncated.covariance,
            np.array([box.interval for box in boxes]),
            np.array([box.bounds for box in boxes]),
            np.log(
                [distribution.normalizing_constant for distribution in distributions]
            ),
            distributions[0].burn_in_period_length,
        )

    def with_nodes(self, indices: Any) -> Self:
        """
        :param indices: A mask or index array over the nodes.
        :return: A layer of only those nodes.
        """
        return self.__class__(
            self.scope.copy(),
            self.mean[indices],
            self.covariance[indices],
            self.interval[indices],
            self.bounds[indices],
            self.log_normalizing_constant[indices],
            self.burn_in_period_length,
        )

    def select_nodes(self, mask: NodeMask) -> Self:
        return self.with_nodes(mask)

    @classmethod
    def concatenate(cls, layers: List[Self]) -> Self:
        return cls(
            layers[0].scope.copy(),
            np.concatenate([layer.mean for layer in layers]),
            np.concatenate([layer.covariance for layer in layers]),
            np.concatenate([layer.interval for layer in layers]),
            np.concatenate([layer.bounds for layer in layers]),
            np.concatenate([layer.log_normalizing_constant for layer in layers]),
            layers[0].burn_in_period_length,
        )

    # %% queries

    @memoized
    def log_likelihood_of_nodes(
        self, events: SampleArray, cache: Optional[QueryCache] = None
    ) -> SampleNodeValues:
        values = self.values_of_scope(events)
        return np.where(
            self.boxes.contains(values),
            self.untruncated_log_density_of_nodes(values)
            - self.log_normalizing_constant,
            -np.inf,
        )

    @memoized
    def cumulative_distribution_of_nodes(
        self, events: SampleArray, cache: Optional[QueryCache] = None
    ) -> SampleNodeValues:
        # the probability of the part of the box below the point
        values = self.values_of_scope(events)
        lower, upper = self.boxes.lower, self.boxes.upper
        result = np.zeros((len(values), self.number_of_nodes))
        for node in range(self.number_of_nodes):
            below_the_box = np.any(values < lower[node], axis=1)
            result[:, node] = np.where(
                below_the_box,
                0.0,
                np.atleast_1d(
                    multivariate_normal(self.mean[node], self.covariance[node]).cdf(
                        np.minimum(values, upper[node]), lower_limit=lower[node]
                    )
                ),
            )
        return np.clip(result / np.exp(self.log_normalizing_constant), 0.0, 1.0)

    @memoized
    def probability_of_simple_event_of_nodes(
        self,
        event: SimpleEvent,
        variables: SortedSet,
        cache: Optional[QueryCache] = None,
    ) -> NodeValues:
        untruncated = sum(
            (
                self.untruncated_probability_of_boxes(self.boxes.intersection_with(box))
                for box in self.boxes_of(event, variables)
            ),
            np.zeros(self.number_of_nodes),
        )
        return untruncated / np.exp(self.log_normalizing_constant)

    @memoized
    def log_mode_of_nodes(
        self, variables: SortedSet, cache: Optional[QueryCache] = None
    ) -> Tuple[List[Event], NodeValues]:
        modes = [
            distribution.log_mode()
            for distribution in self.node_distributions(variables)
        ]
        return [mode for mode, _ in modes], np.array(
            [value for _, value in modes], dtype=float
        )

    @memoized
    def moment_of_nodes(
        self,
        query: MomentQuery,
        variables: SortedSet,
        cache: Optional[QueryCache] = None,
    ) -> NodeVariableValues:
        # a Gaussian confined to a box has no closed-form moment
        if query.requested[self.scope].any():
            raise NoClosedFormError(type(self), type(self).moment_of_nodes)
        return np.zeros((self.number_of_nodes, query.number_of_variables))

    def samples_of_nodes(self, nodes: NodeIndices) -> SampleScopeValues:
        # Gibbs sampling, all chains at once: every sweep draws each variable from its
        # Gaussian given the others, confined to its interval (scipy's truncnorm). Every
        # chain starts at the mean of its node moved into the box. The samples follow the
        # distribution only approximately, closer the more sweeps each chain makes.
        lower = self.boxes.lower[nodes]
        upper = self.boxes.upper[nodes]
        mean = self.mean[nodes]
        precision = np.linalg.inv(self.covariance)[nodes]
        precision_of_itself = np.diagonal(precision, axis1=1, axis2=2)
        conditional_deviation = 1 / np.sqrt(precision_of_itself)

        samples = np.clip(mean, lower, upper)
        for _ in range(self.burn_in_period_length):
            for position in range(self.number_of_scope_variables):
                difference = samples - mean
                conditional_mean = (
                    mean[:, position]
                    - (
                        np.einsum("rj,rj->r", difference, precision[:, position])
                        - precision_of_itself[:, position] * difference[:, position]
                    )
                    / precision_of_itself[:, position]
                )
                deviation = conditional_deviation[:, position]
                samples[:, position] = truncnorm.rvs(
                    a=(lower[:, position] - conditional_mean) / deviation,
                    b=(upper[:, position] - conditional_mean) / deviation,
                    loc=conditional_mean,
                    scale=deviation,
                    size=len(nodes),
                )
        return samples

    # %% structural

    def type_of_layer_truncated_to_box(self, box: HyperrectangleArray) -> Type[Layer]:
        return TruncatedMultivariateGaussianLayer

    def log_truncated_of_box(
        self, box: HyperrectangleArray
    ) -> LayerWithLogProbabilities:
        # every node keeps its Gaussian, confined to the intersection of the two boxes
        intersection = self.boxes.intersection_with(box)
        probability = self.untruncated_probability_of_boxes(intersection)
        alive = probability > 0
        log_normalizing_constant = np.where(
            alive, np.log(np.where(alive, probability, 1.0)), -np.inf
        )

        # impossible nodes keep their parameters and are dropped by the prune pass
        interval = np.where(alive[:, None, None], intersection.interval, self.interval)
        bounds = np.where(alive[:, None, None], intersection.bounds, self.bounds)
        truncated = self.__class__(
            self.scope.copy(),
            self.mean.copy(),
            self.covariance.copy(),
            interval,
            bounds,
            np.where(alive, log_normalizing_constant, self.log_normalizing_constant),
            self.burn_in_period_length,
        )
        return LayerWithLogProbabilities(
            truncated, log_normalizing_constant - self.log_normalizing_constant
        )

    def log_conditional_of_values(
        self, fixed: NodeIndices, free: NodeIndices, values: np.ndarray
    ) -> LayerWithLogProbabilities:
        # the Gaussian conditional of every node, confined to the slice its box makes
        # at the values
        inside = self.boxes.select((slice(None), fixed)).contains(values[None, :])[0]
        log_density = self.gaussians_over(fixed).untruncated_log_density_of_nodes(
            values[None, :]
        )[0]
        if len(free) == 0:
            return LayerWithLogProbabilities(
                self.__deepcopy__(),
                np.where(inside, log_density - self.log_normalizing_constant, -np.inf),
            )

        conditionals = self.gaussian_conditionals(fixed, free, values)
        slices = self.boxes.select((slice(None), free))
        probability = conditionals.untruncated_probability_of_boxes(slices)
        alive = inside & (probability > 0)
        log_probability_of_slice = np.log(np.where(alive, probability, 1.0))
        conditioned = self.__class__(
            conditionals.scope,
            conditionals.mean,
            conditionals.covariance,
            slices.interval,
            slices.bounds,
            log_probability_of_slice,
            self.burn_in_period_length,
        )
        return LayerWithLogProbabilities(
            conditioned,
            np.where(
                alive,
                log_density + log_probability_of_slice - self.log_normalizing_constant,
                -np.inf,
            ),
        )

    def marginal_over(self, positions: NodeIndices) -> Layer:
        # the marginal of a Gaussian confined to a box is not a Gaussian confined to a
        # box
        raise NoClosedFormError(type(self), type(self).marginal)

    def reorder_scope(self, order: NodeIndices):
        super().reorder_scope(order)
        self.interval = self.interval[:, order]
        self.bounds = self.bounds[:, order]

    def apply_translation_own(self, translation: VariableValues):
        super().apply_translation_own(translation)
        self.interval = self.interval + translation[self.scope][None, :, None]

    def apply_scaling_own(self, scaling: VariableValues):
        super().apply_scaling_own(scaling)
        self.interval = self.interval * scaling[self.scope][None, :, None]
        # the box scales with the Gaussian, so its probability stays the same

    def __deepcopy__(self, memo=None) -> TruncatedMultivariateGaussianLayer:
        if memo is None:
            memo = {}
        if id(self) in memo:
            return memo[id(self)]
        # selecting by an index array copies every parameter array
        result = self.with_nodes(np.arange(self.number_of_nodes))
        memo[id(self)] = result
        return result
