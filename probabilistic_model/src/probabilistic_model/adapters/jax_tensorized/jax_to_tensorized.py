from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass, field

import numpy as np
from random_events.interval import Bound
from random_events.variable import Integer, Symbolic
from scipy.sparse import coo_array
from sortedcontainers import SortedSet
from typing_extensions import Dict

from probabilistic_model.adapters.exceptions import CannotConvertError
from probabilistic_model.adapters.jax_tensorized.converter import (
    InputType,
    JaxToTensorizedConverter,
)
from probabilistic_model.adapters.jax_tensorized.utils import (
    columns_of_domain_elements,
    to_numpy,
)
from probabilistic_model.probabilistic_circuit.jax import (
    discrete_layer as jax_discrete_layer,
    gaussian_layer as jax_gaussian_layer,
    inner_layer as jax_inner_layer,
    input_layer as jax_input_layer,
    probabilistic_circuit as jax_probabilistic_circuit,
    uniform_layer as jax_uniform_layer,
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
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.discrete_layer import (
    DiscreteLayer,
    IntegerLayer,
    SymbolicLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.gaussian_layer import (
    GaussianLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.probability_table import (
    DenseProbabilityTable,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.uniform_layer import (
    UniformLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.tensorized.row_grouped_sparse_array import (
    RowGroupedSparseArray,
    SparseEntries,
)
from probabilistic_model.probabilistic_circuit.tensorized.symbolic_encoding import (
    SymbolicEncoding,
)


@dataclass
class LayeredCircuitBuilder:
    """
    The state of converting the layers of one circuit of the ``jax`` package into one
    layered circuit, which converts every layer once so that a layer shared by several
    parents stays shared.
    """

    variables: SortedSet
    """
    The variables of the circuit, in the order the layers index them.
    """

    layers_by_jax_layer: Dict[int, Layer] = field(default_factory=dict)
    """
    The layer created for every layer converted so far, keyed by the id of the layer of
    the ``jax`` package.
    """

    def layer_of(self, jax_layer: jax_inner_layer.Layer) -> Layer:
        """
        :param jax_layer: A layer of the circuit.
        :return: The layer of the ``tensorized`` package with the same nodes.
        """
        if id(jax_layer) not in self.layers_by_jax_layer:
            self.layers_by_jax_layer[id(jax_layer)] = JaxToTensorizedConverter.convert(
                jax_layer, self
            )
        return self.layers_by_jax_layer[id(jax_layer)]


# %% inner layers


class JaxSumLayerToSumLayerConverter(JaxToTensorizedConverter[InputType, SumLayer]):
    """
    Base class for converters of the sum layers of the ``jax`` package, which keep one
    weight matrix per child layer, into one sum layer with a single sparse weight matrix
    whose columns are the nodes of all child layers in order.

    Training leaves the weights unnormalized, so the converted weights are normalized.
    """

    @staticmethod
    @abstractmethod
    def entries_of(log_weights) -> SparseEntries:
        """
        :param log_weights: The logarithmic weights of the edges into one child layer.
        :return: The stored entries of the weights, the columns indexing the nodes of
            that child layer.
        """
        raise NotImplementedError

    @classmethod
    def convert(
        cls, data: jax_inner_layer.SumLayer, builder: LayeredCircuitBuilder
    ) -> SumLayer:
        child_layers = [
            builder.layer_of(child_layer) for child_layer in data.child_layers
        ]
        entries = []
        offset = 0
        for log_weights, child_layer in zip(data.log_weights, child_layers):
            child_entries = cls.entries_of(log_weights)
            child_entries.columns = child_entries.columns + offset
            entries.append(child_entries)
            offset += child_layer.number_of_nodes

        layer = SumLayer(
            child_layers,
            RowGroupedSparseArray.from_entries(
                SparseEntries.concatenate(entries), (data.number_of_nodes, offset)
            ),
        )
        layer.normalize_own()
        return layer


class SparseSumLayerToSumLayerConverter(
    JaxSumLayerToSumLayerConverter[jax_inner_layer.SparseSumLayer]
):

    @staticmethod
    def entries_of(log_weights) -> SparseEntries:
        indices = np.asarray(log_weights.indices, dtype=np.int64)
        return SparseEntries(to_numpy(log_weights.data), indices[:, 0], indices[:, 1])


class DenseSumLayerToSumLayerConverter(
    JaxSumLayerToSumLayerConverter[jax_inner_layer.DenseSumLayer]
):

    @staticmethod
    def entries_of(log_weights) -> SparseEntries:
        rows, columns = np.indices(log_weights.shape, dtype=np.int64)
        return SparseEntries(
            to_numpy(log_weights).ravel(), rows.ravel(), columns.ravel()
        )


class ProductLayerToProductLayerConverter(
    JaxToTensorizedConverter[jax_inner_layer.ProductLayer, ProductLayer]
):
    """
    Both packages store the edges of a product layer as the same sparse matrix of child
    node indices.
    """

    @classmethod
    def convert(
        cls, data: jax_inner_layer.ProductLayer, builder: LayeredCircuitBuilder
    ) -> ProductLayer:
        indices = np.asarray(data.edges.indices, dtype=np.int64)
        edges = coo_array(
            (
                np.asarray(data.edges.data, dtype=np.int64),
                (indices[:, 0], indices[:, 1]),
            ),
            shape=data.edges.shape,
        )
        return ProductLayer(
            [builder.layer_of(child_layer) for child_layer in data.child_layers], edges
        )


# %% input layers


class GaussianLayerToGaussianLayerConverter(
    JaxToTensorizedConverter[jax_gaussian_layer.GaussianLayer, GaussianLayer]
):
    """
    The scale of a Gaussian layer of the ``jax`` package includes its minimum scale.
    """

    @classmethod
    def convert(
        cls, data: jax_gaussian_layer.GaussianLayer, builder: LayeredCircuitBuilder
    ) -> GaussianLayer:
        return GaussianLayer(
            data.variable, to_numpy(data.location), to_numpy(data.scale)
        )


class UniformLayerToUniformLayerConverter(
    JaxToTensorizedConverter[jax_uniform_layer.UniformLayer, UniformLayer]
):
    """
    A uniform layer of the ``jax`` package treats every interval as open.
    """

    @classmethod
    def convert(
        cls, data: jax_uniform_layer.UniformLayer, builder: LayeredCircuitBuilder
    ) -> UniformLayer:
        interval = to_numpy(data.interval)
        return UniformLayer(
            data.variable,
            interval,
            np.full(interval.shape, int(Bound.OPEN), dtype=np.int64),
        )


class DiracDeltaLayerToDiracDeltaLayerConverter(
    JaxToTensorizedConverter[jax_input_layer.DiracDeltaLayer, DiracDeltaLayer]
):

    @classmethod
    def convert(
        cls, data: jax_input_layer.DiracDeltaLayer, builder: LayeredCircuitBuilder
    ) -> DiracDeltaLayer:
        return DiracDeltaLayer(
            data.variable, to_numpy(data.location), to_numpy(data.density_cap)
        )


class DiscreteLayerToDiscreteLayerConverter(
    JaxToTensorizedConverter[jax_discrete_layer.DiscreteLayer, DiscreteLayer]
):
    """
    A discrete layer of the ``jax`` package looks the probability of a value up in the
    column with the value as its index: the value itself for an integer variable and the
    hash of the domain element for a symbolic variable.

    The variable decides whether the layer becomes a symbolic or an integer layer.
    """

    @classmethod
    def convert(
        cls, data: jax_discrete_layer.DiscreteLayer, builder: LayeredCircuitBuilder
    ) -> DiscreteLayer:
        variable = builder.variables[data.variable]
        log_probabilities = to_numpy(data.normalized_log_probabilities)
        if isinstance(variable, Symbolic):
            columns = columns_of_domain_elements(variable)
            return SymbolicLayer(
                data.variable,
                np.arange(len(columns), dtype=np.int64),
                DenseProbabilityTable(log_probabilities[:, columns]),
                SymbolicEncoding(variable).hashes,
            )
        if isinstance(variable, Integer):
            return IntegerLayer(
                data.variable,
                np.arange(log_probabilities.shape[1], dtype=np.int64),
                DenseProbabilityTable(log_probabilities),
            )
        raise CannotConvertError(data_type=type(variable))


# %% circuit


class JaxCircuitToLayeredCircuitConverter(
    JaxToTensorizedConverter[
        jax_probabilistic_circuit.DifferentiableLayeredCircuit,
        LayeredProbabilisticCircuit,
    ]
):
    """
    Convert a circuit of the ``jax`` package, usually after training it, into a layered
    circuit that answers every query.

    A classification circuit has one root node per class and stays in the ``jax``
    package, so no subclass of a circuit is converted.
    """

    @classmethod
    def convert(
        cls, data: jax_probabilistic_circuit.DifferentiableLayeredCircuit
    ) -> LayeredProbabilisticCircuit:
        if not cls.can_convert(data):
            raise CannotConvertError(data_type=type(data))
        variables = SortedSet(data.variables)
        builder = LayeredCircuitBuilder(variables)
        return LayeredProbabilisticCircuit(variables, builder.layer_of(data.root))
