from __future__ import annotations

from dataclasses import dataclass, field

import jax.numpy as jnp
import numpy as np
from jax.experimental.sparse import BCOO
from random_events.variable import Symbolic
from sortedcontainers import SortedSet
from typing_extensions import Dict

from probabilistic_model.adapters.jax_tensorized.converter import (
    TensorizedToJaxConverter,
)
from probabilistic_model.adapters.jax_tensorized.exceptions import (
    StatesAreNotColumnIndicesError,
)
from probabilistic_model.adapters.jax_tensorized.jax_to_tensorized import (
    columns_of_domain_elements,
)
from probabilistic_model.probabilistic_circuit.jax import (
    discrete_layer as jax_discrete_layer,
    gaussian_layer as jax_gaussian_layer,
    inner_layer as jax_inner_layer,
    input_layer as jax_input_layer,
    probabilistic_circuit as jax_probabilistic_circuit,
    uniform_layer as jax_uniform_layer,
)
from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    StateIndices,
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
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.uniform_layer import (
    UniformLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)


@dataclass
class JaxCircuitBuilder:
    """
    The state of converting the layers of one layered circuit into one circuit of the
    ``jax`` package, which converts every layer once so that a layer shared by several
    parents stays shared.
    """

    variables: SortedSet
    """
    The variables of the circuit, in the order the layers index them.
    """

    jax_layers_by_layer: Dict[int, jax_inner_layer.Layer] = field(default_factory=dict)
    """
    The layer of the ``jax`` package created for every layer converted so far, keyed by
    the id of the layer.
    """

    def jax_layer_of(self, layer: Layer) -> jax_inner_layer.Layer:
        """
        :param layer: A layer of the circuit.
        :return: The layer of the ``jax`` package with the same nodes.
        """
        if id(layer) not in self.jax_layers_by_layer:
            self.jax_layers_by_layer[id(layer)] = TensorizedToJaxConverter.convert(
                layer, self
            )
        return self.jax_layers_by_layer[id(layer)]


def sparse_matrix(
    values: np.ndarray, rows: np.ndarray, columns: np.ndarray, shape
) -> BCOO:
    """
    :param values: The value of every stored entry.
    :param rows: The row of every stored entry.
    :param columns: The column of every stored entry.
    :param shape: The shape of the dense matrix.
    :return: The sparse matrix of the ``jax`` package that stores the entries.
    """
    indices = np.stack([rows, columns], axis=1).astype(np.int32).reshape(-1, 2)
    return BCOO(
        (jnp.asarray(values), jnp.asarray(indices)), shape=tuple(shape)
    ).sort_indices()


# %% inner layers


class SumLayerToSparseSumLayerConverter(
    TensorizedToJaxConverter[SumLayer, jax_inner_layer.SparseSumLayer]
):
    """
    Split the single weight matrix of a sum layer into one sparse weight matrix per
    child layer.
    """

    @classmethod
    def convert(
        cls, data: SumLayer, builder: JaxCircuitBuilder
    ) -> jax_inner_layer.SparseSumLayer:
        offsets = data.column_offsets
        rows = data.log_weights.rows
        columns = data.log_weights.columns
        log_weights = data.normalized_edge_log_weights
        weights_per_child_layer = []
        for index, child_layer in enumerate(data.child_layers):
            into_child_layer = (columns >= offsets[index]) & (
                columns < offsets[index + 1]
            )
            weights_per_child_layer.append(
                sparse_matrix(
                    log_weights[into_child_layer],
                    rows[into_child_layer],
                    columns[into_child_layer] - offsets[index],
                    (data.number_of_nodes, child_layer.number_of_nodes),
                )
            )
        return jax_inner_layer.SparseSumLayer(
            [builder.jax_layer_of(child_layer) for child_layer in data.child_layers],
            weights_per_child_layer,
        )


class ProductLayerToProductLayerConverter(
    TensorizedToJaxConverter[ProductLayer, jax_inner_layer.ProductLayer]
):
    """
    Both packages store the edges of a product layer as the same sparse matrix of child
    node indices.
    """

    @classmethod
    def convert(
        cls, data: ProductLayer, builder: JaxCircuitBuilder
    ) -> jax_inner_layer.ProductLayer:
        edges = sparse_matrix(
            data.edges.data.astype(np.int32),
            data.edges.row,
            data.edges.col,
            data.edges.shape,
        )
        return jax_inner_layer.ProductLayer(
            [builder.jax_layer_of(child_layer) for child_layer in data.child_layers],
            edges,
        )


# %% input layers


class GaussianLayerToGaussianLayerConverter(
    TensorizedToJaxConverter[GaussianLayer, jax_gaussian_layer.GaussianLayer]
):
    """
    The converted layer has no minimum scale, so that it has the same scale.
    """

    @classmethod
    def convert(
        cls, data: GaussianLayer, builder: JaxCircuitBuilder
    ) -> jax_gaussian_layer.GaussianLayer:
        return jax_gaussian_layer.GaussianLayer(
            data.variable,
            location=jnp.asarray(data.location),
            log_scale=jnp.log(jnp.asarray(data.scale)),
            min_scale=jnp.zeros(data.number_of_nodes),
        )


class UniformLayerToUniformLayerConverter(
    TensorizedToJaxConverter[UniformLayer, jax_uniform_layer.UniformLayer]
):
    """
    A uniform layer of the ``jax`` package treats every interval as open, so a closed
    bound is lost.
    """

    @classmethod
    def convert(
        cls, data: UniformLayer, builder: JaxCircuitBuilder
    ) -> jax_uniform_layer.UniformLayer:
        return jax_uniform_layer.UniformLayer(data.variable, jnp.asarray(data.interval))


class DiracDeltaLayerToDiracDeltaLayerConverter(
    TensorizedToJaxConverter[DiracDeltaLayer, jax_input_layer.DiracDeltaLayer]
):
    """
    A Dirac delta layer of the ``jax`` package compares a value with its location
    exactly, so the tolerance is lost.
    """

    @classmethod
    def convert(
        cls, data: DiracDeltaLayer, builder: JaxCircuitBuilder
    ) -> jax_input_layer.DiracDeltaLayer:
        return jax_input_layer.DiracDeltaLayer(
            data.variable, jnp.asarray(data.location), jnp.asarray(data.density_cap)
        )


class SymbolicLayerToDiscreteLayerConverter(
    TensorizedToJaxConverter[SymbolicLayer, jax_discrete_layer.DiscreteLayer]
):
    """
    Lay the states of a symbolic layer out as the columns of a probability table with
    one column per domain element, the column of an element being its hash.
    """

    @classmethod
    def convert(
        cls, data: SymbolicLayer, builder: JaxCircuitBuilder
    ) -> jax_discrete_layer.DiscreteLayer:
        variable: Symbolic = builder.variables[data.variable]
        columns = columns_of_domain_elements(variable)
        return discrete_layer_of(data, columns[data.states], len(columns))


class IntegerLayerToDiscreteLayerConverter(
    TensorizedToJaxConverter[IntegerLayer, jax_discrete_layer.DiscreteLayer]
):
    """
    Lay the states of an integer layer out as the columns of a probability table that
    reaches up to the largest state.
    """

    @classmethod
    def convert(
        cls, data: IntegerLayer, builder: JaxCircuitBuilder
    ) -> jax_discrete_layer.DiscreteLayer:
        if (data.states < 0).any():
            raise StatesAreNotColumnIndicesError(
                variable=builder.variables[data.variable], states=data.states
            )
        return discrete_layer_of(data, data.states, int(data.states.max()) + 1)


def discrete_layer_of(
    data: DiscreteLayer, columns: StateIndices, number_of_columns: int
) -> jax_discrete_layer.DiscreteLayer:
    """
    :param data: A discrete layer.
    :param columns: The column of the probability table that holds every state of the
        layer.
    :param number_of_columns: The number of columns of the probability table.
    :return: The discrete layer of the ``jax`` package, impossible in every column
        that holds no state.
    """
    probabilities = np.zeros((data.number_of_nodes, number_of_columns))
    probabilities[:, columns] = data.table.dense_probabilities()
    with np.errstate(divide="ignore"):
        log_probabilities = np.log(probabilities)
    return jax_discrete_layer.DiscreteLayer(
        data.variable, jnp.asarray(log_probabilities)
    )


# %% circuit


class LayeredCircuitToJaxCircuitConverter(
    TensorizedToJaxConverter[
        LayeredProbabilisticCircuit, jax_probabilistic_circuit.ProbabilisticCircuit
    ]
):
    """
    Convert a layered circuit into a circuit of the ``jax`` package, so that a circuit
    learned in another way can be refined by gradient descent.
    """

    @classmethod
    def convert(
        cls, data: LayeredProbabilisticCircuit
    ) -> jax_probabilistic_circuit.ProbabilisticCircuit:
        builder = JaxCircuitBuilder(data.variables)
        return jax_probabilistic_circuit.ProbabilisticCircuit(
            SortedSet(data.variables), builder.jax_layer_of(data.root)
        )
