import unittest
from enum import IntEnum

import jax.numpy as jnp
import numpy as np
from jax.experimental.sparse import BCOO
from random_events.interval import Bound
from random_events.set import Set
from random_events.variable import Continuous, Integer, Symbolic
from scipy.sparse import coo_array
from sortedcontainers import SortedSet

from probabilistic_model.adapters.exceptions import CannotConvertError
from probabilistic_model.adapters.jax_tensorized.exceptions import (
    StatesAreNotColumnIndicesError,
)
from probabilistic_model.adapters.jax_tensorized.jax_to_tensorized import (
    JaxCircuitToLayeredCircuitConverter,
)
from probabilistic_model.adapters.jax_tensorized.tensorized_to_jax import (
    LayeredCircuitToJaxCircuitConverter,
)
from probabilistic_model.learning.region_graph.region_graph import RegionGraph
from probabilistic_model.probabilistic_circuit.jax import (
    discrete_layer as jax_discrete_layer,
    gaussian_layer as jax_gaussian_layer,
    inner_layer as jax_inner_layer,
    input_layer as jax_input_layer,
    uniform_layer as jax_uniform_layer,
)
from probabilistic_model.probabilistic_circuit.jax.probabilistic_circuit import (
    ClassificationCircuit,
    DifferentiableLayeredCircuit,
)
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
    IntegerLayer,
    SymbolicLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.gaussian_layer import (
    GaussianLayer,
    TruncatedGaussianLayer,
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


class Color(IntEnum):
    RED = 0
    GREEN = 1
    BLUE = 2


class Size(IntEnum):
    """
    Members whose order of declaration differs from the order of their hashes.
    """

    LARGE = 2
    SMALL = 0
    MEDIUM = 1


w = Continuous("w")
x = Continuous("x")
y = Continuous("y")
z = Continuous("z")
color = Symbolic(name="color", domain=Set.from_iterable(Color))
size = Symbolic(name="size", domain=Set.from_iterable(Size))
count = Integer("count")

# JAX evaluates in single precision
single_precision_tolerance = 1e-4


# %% circuits


def region_graph_circuit(variables: SortedSet) -> DifferentiableLayeredCircuit:
    """
    :param variables: At least four variables, so that every region of the region
        graph can be split.
    :return: A randomly initialized JAX circuit over the variables, as a region graph
        builds it for training.
    """
    region_graph = RegionGraph(variables, partitions=2, depth=1, repetitions=2)
    return region_graph.create_random_region_graph().as_probabilistic_circuit(
        input_units=3, sum_units=3
    )


def sparse_matrix(entries, shape) -> BCOO:
    """
    :param entries: The ``(row, column, value)`` of every stored entry.
    :param shape: The shape of the matrix.
    :return: The sparse matrix.
    """
    rows, columns, values = zip(*entries)
    return BCOO((jnp.array(values), jnp.array(list(zip(rows, columns)))), shape=shape)


def product_edges(entries, shape) -> BCOO:
    """
    :param entries: The ``(child layer, node, child node)`` of every edge.
    :param shape: The number of child layers and the number of nodes.
    :return: The edges of a JAX product layer.
    """
    return sparse_matrix(entries, shape)


def mixture_of_products(
    x_layer: jax_inner_layer.InputLayer, y_layer: jax_inner_layer.InputLayer
) -> DifferentiableLayeredCircuit:
    """
    :return: A circuit over x and y whose root mixes two products of the nodes of the
        layers with unnormalized weights, as training leaves them.
    """
    product = jax_inner_layer.ProductLayer(
        [x_layer, y_layer],
        product_edges([(0, 0, 0), (0, 1, 1), (1, 0, 1), (1, 1, 0)], (2, 2)),
    )
    root = jax_inner_layer.SparseSumLayer(
        [product], [sparse_matrix([(0, 0, 0.3), (0, 1, -1.2)], (1, 2))]
    )
    return DifferentiableLayeredCircuit(SortedSet([x, y]), root)


def gaussian_layer(variable: int) -> jax_gaussian_layer.GaussianLayer:
    """
    :return: A JAX Gaussian layer with two nodes over the variable.
    """
    return jax_gaussian_layer.GaussianLayer(
        variable,
        location=jnp.array([-1.0, 2.0]),
        log_scale=jnp.log(jnp.array([0.5, 1.5])),
        min_scale=jnp.array([0.1, 0.2]),
    )


def layered_circuit_over_color_and_x() -> LayeredProbabilisticCircuit:
    """
    :return: A numpy circuit over a symbolic and a continuous variable that uses every
        input layer JAX supports.
    """
    colors = SymbolicLayer(
        0,
        np.arange(len(Color)),
        DenseProbabilityTable(np.log(np.array([[0.2, 0.5, 0.3], [0.6, 0.1, 0.3]]))),
        SymbolicEncoding(color).hashes,
    )
    gaussians = GaussianLayer(1, np.array([0.0, 3.0]), np.array([1.0, 0.5]))
    uniforms = UniformLayer(
        1,
        np.array([[-1.0, 1.0], [2.0, 5.0]]),
        np.full((2, 2), int(Bound.OPEN), dtype=np.int64),
    )
    dirac_deltas = DiracDeltaLayer(1, np.array([0.5]), np.array([2.0]))
    x_mixture = SumLayer(
        [gaussians, uniforms, dirac_deltas],
        RowGroupedSparseArray.from_entries(
            SparseEntries(
                np.log(np.array([0.4, 0.4, 0.2, 0.7, 0.3])),
                np.array([0, 0, 0, 1, 1]),
                np.array([0, 2, 4, 1, 3]),
            ),
            (2, 5),
        ),
    )
    products = ProductLayer(
        [colors, x_mixture],
        coo_array(
            (np.array([0, 1, 0, 1]), (np.array([0, 0, 1, 1]), np.array([0, 1, 1, 0]))),
            shape=(2, 2),
        ),
    )
    root = SumLayer(
        [products],
        RowGroupedSparseArray.from_entries(
            SparseEntries(
                np.log(np.array([0.25, 0.75])), np.array([0, 0]), np.array([0, 1])
            ),
            (1, 2),
        ),
    )
    return LayeredProbabilisticCircuit(SortedSet([color, x]), root)


# %% JAX to numpy


class JaxToNumpyConversionTestCase(unittest.TestCase):
    """
    A trained JAX circuit converts into a numpy circuit that expresses the same
    distribution, so that every query of the numpy circuit answers for it.
    """

    def assert_same_log_likelihoods(
        self,
        jax_circuit: DifferentiableLayeredCircuit,
        numpy_circuit: LayeredProbabilisticCircuit,
        events: np.ndarray,
    ):
        expected = np.asarray(jax_circuit.log_likelihood(jnp.asarray(events)))
        np.testing.assert_allclose(
            numpy_circuit.log_likelihood(events),
            expected,
            rtol=single_precision_tolerance,
            atol=single_precision_tolerance,
        )

    def test_region_graph_circuit_has_the_same_log_likelihoods(self):
        jax_circuit = region_graph_circuit(SortedSet([w, x, y, z]))
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        self.assert_same_log_likelihoods(
            jax_circuit, numpy_circuit, np.random.normal(size=(50, 4))
        )

    def test_region_graph_circuit_with_symbolic_variable_has_the_same_log_likelihoods(
        self,
    ):
        variables = SortedSet([w, x, y, color])
        jax_circuit = region_graph_circuit(variables)
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        events = np.random.normal(size=(50, 4))
        events[:, variables.index(color)] = np.random.randint(0, len(Color), 50)
        self.assert_same_log_likelihoods(jax_circuit, numpy_circuit, events)

    def test_result_keeps_the_variables(self):
        jax_circuit = region_graph_circuit(SortedSet([w, x, y, z]))
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        self.assertEqual(numpy_circuit.variables, jax_circuit.variables)

    def test_sum_weights_are_normalized(self):
        jax_circuit = mixture_of_products(gaussian_layer(0), gaussian_layer(1))
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        np.testing.assert_allclose(
            numpy_circuit.root.log_normalization_constants, 0.0, atol=1e-12
        )

    def test_gaussian_scale_includes_the_minimum_scale(self):
        jax_layer = gaussian_layer(0)
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(
            mixture_of_products(jax_layer, gaussian_layer(1))
        )
        numpy_layer = numpy_circuit.root.child_layers[0].child_layers[0]
        self.assertIsInstance(numpy_layer, GaussianLayer)
        np.testing.assert_allclose(numpy_layer.scale, np.asarray(jax_layer.scale))

    def test_uniform_and_dirac_delta_layers_have_the_same_log_likelihoods(self):
        uniforms = jax_uniform_layer.UniformLayer(
            0, jnp.array([[-1.0, 1.0], [0.0, 4.0]])
        )
        dirac_deltas = jax_input_layer.DiracDeltaLayer(
            1, jnp.array([0.5, 1.5]), jnp.array([2.0, 3.0])
        )
        jax_circuit = mixture_of_products(uniforms, dirac_deltas)
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        events = np.array([[0.5, 0.5], [0.5, 1.5], [3.0, 1.5], [-0.5, 0.5]])
        self.assert_same_log_likelihoods(jax_circuit, numpy_circuit, events)

    def test_uniform_bounds_are_open(self):
        uniforms = jax_uniform_layer.UniformLayer(
            0, jnp.array([[-1.0, 1.0], [0.0, 4.0]])
        )
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(
            mixture_of_products(uniforms, gaussian_layer(1))
        )
        numpy_layer = numpy_circuit.root.child_layers[0].child_layers[0]
        self.assertTrue((numpy_layer.bounds == int(Bound.OPEN)).all())

    def test_discrete_layer_over_an_integer_variable_becomes_an_integer_layer(self):
        integers = jax_discrete_layer.DiscreteLayer(
            0, jnp.log(jnp.array([[1.0, 2.0, 1.0], [0.0, 1.0, 3.0]]))
        )
        jax_circuit = mixture_of_products(integers, gaussian_layer(1))
        jax_circuit.variables = SortedSet([count, y])
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        numpy_layer = numpy_circuit.root.child_layers[0].child_layers[0]
        self.assertIsInstance(numpy_layer, IntegerLayer)
        events = np.array([[0.0, 0.3], [1.0, -1.0], [2.0, 2.5]])
        self.assert_same_log_likelihoods(jax_circuit, numpy_circuit, events)

    def test_symbolic_column_is_the_hash_of_the_domain_element(self):
        sizes = jax_discrete_layer.DiscreteLayer(
            0, jnp.log(jnp.array([[1.0, 2.0, 5.0], [4.0, 1.0, 3.0]]))
        )
        jax_circuit = mixture_of_products(sizes, gaussian_layer(1))
        jax_circuit.variables = SortedSet([size, y])
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        numpy_layer = numpy_circuit.root.child_layers[0].child_layers[0]
        self.assertIsInstance(numpy_layer, SymbolicLayer)
        events = np.array(
            [
                [hash(Size.SMALL), 0.3],
                [hash(Size.MEDIUM), -1.0],
                [hash(Size.LARGE), 2.5],
            ]
        )
        self.assert_same_log_likelihoods(jax_circuit, numpy_circuit, events)

    def test_shared_child_layer_stays_shared(self):
        shared = gaussian_layer(0)
        jax_circuit = mixture_of_products(shared, gaussian_layer(1))
        first_product = jax_circuit.root.child_layers[0]
        second_product = jax_inner_layer.ProductLayer(
            [shared, gaussian_layer(1)],
            product_edges([(0, 0, 1), (1, 0, 0)], (2, 1)),
        )
        jax_circuit.root = jax_inner_layer.SparseSumLayer(
            [first_product, second_product],
            [
                sparse_matrix([(0, 0, 0.0), (0, 1, 0.0)], (1, 2)),
                sparse_matrix([(0, 0, 0.0)], (1, 1)),
            ],
        )
        numpy_circuit = JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        first, second = numpy_circuit.root.child_layers
        self.assertIs(first.child_layers[0], second.child_layers[0])

    def test_classification_circuit_is_refused(self):
        classification_circuit = RegionGraph(
            SortedSet([w, x, y, z]), partitions=2, depth=1, repetitions=2, classes=2
        )
        classification_circuit = (
            classification_circuit.create_random_region_graph().as_probabilistic_circuit()
        )
        self.assertIsInstance(classification_circuit, ClassificationCircuit)
        with self.assertRaises(CannotConvertError):
            JaxCircuitToLayeredCircuitConverter.convert(classification_circuit)


# %% numpy to JAX


class NumpyToJaxConversionTestCase(unittest.TestCase):
    """
    A numpy circuit converts into a JAX circuit with the same distribution, so that a
    circuit learned in another way can be refined by gradient descent.
    """

    def test_every_supported_input_layer_has_the_same_log_likelihoods(self):
        numpy_circuit = layered_circuit_over_color_and_x()
        jax_circuit = LayeredCircuitToJaxCircuitConverter.convert(numpy_circuit)
        events = np.array(
            [[0.0, 0.5], [1.0, 0.5], [2.0, -0.3], [1.0, 3.2], [0.0, 4.5], [2.0, 9.0]]
        )
        np.testing.assert_allclose(
            np.asarray(jax_circuit.log_likelihood(jnp.asarray(events))),
            numpy_circuit.log_likelihood(events),
            rtol=single_precision_tolerance,
            atol=single_precision_tolerance,
        )

    def test_round_trip_keeps_the_log_likelihoods(self):
        numpy_circuit = layered_circuit_over_color_and_x()
        round_trip = JaxCircuitToLayeredCircuitConverter.convert(
            LayeredCircuitToJaxCircuitConverter.convert(numpy_circuit)
        )
        events = np.array([[0.0, 0.5], [2.0, -0.3], [1.0, 3.2]])
        np.testing.assert_allclose(
            round_trip.log_likelihood(events),
            numpy_circuit.log_likelihood(events),
            rtol=single_precision_tolerance,
        )

    def test_jax_round_trip_keeps_the_log_likelihoods(self):
        jax_circuit = region_graph_circuit(SortedSet([w, x, y, z]))
        round_trip = LayeredCircuitToJaxCircuitConverter.convert(
            JaxCircuitToLayeredCircuitConverter.convert(jax_circuit)
        )
        events = jnp.asarray(np.random.normal(size=(20, 4)))
        np.testing.assert_allclose(
            np.asarray(round_trip.log_likelihood(events)),
            np.asarray(jax_circuit.log_likelihood(events)),
            rtol=single_precision_tolerance,
            atol=single_precision_tolerance,
        )

    def test_symbolic_state_is_stored_in_the_column_of_its_hash(self):
        sizes = SymbolicLayer(
            0,
            np.arange(len(Size)),
            DenseProbabilityTable(np.log(np.array([[0.5, 0.2, 0.3]]))),
            SymbolicEncoding(size).hashes,
        )
        numpy_circuit = LayeredProbabilisticCircuit(
            SortedSet([size]), SumLayer.mixture_of([sizes], [0.0])
        )
        jax_circuit = LayeredCircuitToJaxCircuitConverter.convert(numpy_circuit)
        events = np.array([[hash(member)] for member in Size], dtype=float)
        np.testing.assert_allclose(
            np.asarray(jax_circuit.log_likelihood(jnp.asarray(events))),
            numpy_circuit.log_likelihood(events),
            rtol=single_precision_tolerance,
        )

    def test_truncated_gaussian_layer_is_refused(self):
        truncated = TruncatedGaussianLayer(
            0,
            np.array([[-1.0, 1.0]]),
            np.full((1, 2), int(Bound.CLOSED), dtype=np.int64),
            np.array([0.0]),
            np.array([1.0]),
        )
        numpy_circuit = LayeredProbabilisticCircuit(
            SortedSet([x]), SumLayer.mixture_of([truncated], [0.0])
        )
        with self.assertRaises(CannotConvertError):
            LayeredCircuitToJaxCircuitConverter.convert(numpy_circuit)

    def test_symbolic_states_that_are_not_column_indices_are_refused(self):
        letters = Symbolic(name="letter", domain=Set.from_iterable(["a", "b"]))
        symbols = SymbolicLayer(
            0,
            np.arange(2),
            DenseProbabilityTable(np.log(np.array([[0.5, 0.5]]))),
            SymbolicEncoding(letters).hashes,
        )
        numpy_circuit = LayeredProbabilisticCircuit(
            SortedSet([letters]), SumLayer.mixture_of([symbols], [0.0])
        )
        with self.assertRaises(StatesAreNotColumnIndicesError):
            LayeredCircuitToJaxCircuitConverter.convert(numpy_circuit)

    def test_negative_integer_states_are_refused(self):
        integers = IntegerLayer(
            0, np.array([-1, 0]), DenseProbabilityTable(np.log(np.array([[0.5, 0.5]])))
        )
        numpy_circuit = LayeredProbabilisticCircuit(
            SortedSet([count]), SumLayer.mixture_of([integers], [0.0])
        )
        with self.assertRaises(StatesAreNotColumnIndicesError):
            LayeredCircuitToJaxCircuitConverter.convert(numpy_circuit)


if __name__ == "__main__":
    unittest.main()
