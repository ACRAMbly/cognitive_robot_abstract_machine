from __future__ import annotations

import math
from abc import abstractmethod, ABC
import equinox as eqx
import jax
from jax import numpy as jnp
from jax.experimental.sparse import BCOO, bcoo_concatenate
from jax.scipy.special import logsumexp
from jax.tree_util import tree_flatten
from jaxtyping import Int, Array
from krrood.adapters.json_serializer import SubclassJSONSerializer
from probabilistic_model.exceptions import ShapeMismatchError
from typing_extensions import List, Iterator, Tuple, Union, Dict, Any, Self, Optional

from probabilistic_model.probabilistic_circuit.jax.utils import copy_bcoo


class Layer(eqx.Module, SubclassJSONSerializer, ABC):
    """
    Abstract class for Layers of a layered circuit.

    Layers have the same scope (set of variables) for every node in them.
    """

    _variables: Optional[Array] = eqx.field(static=False, default=None)
    """
    The variable indices of the layer.
    """

    @property
    def variables(self) -> jax.Array:
        raise NotImplementedError

    def set_variables(self, value: jax.Array):
        raise NotImplementedError

    @abstractmethod
    def log_likelihood_of_nodes_single(self, x: Array) -> Array:
        """
        Calculate the log-likelihood of the distribution.

        :param x: The whole event, one value per variable of the circuit, which the
            layer indexes by the indices of its variables.
        :return: The log-likelihood of every node in the layer for x.
        """

    def log_likelihood_of_nodes(self, x: Array) -> Array:
        """
        Vectorized version of :meth:`log_likelihood_of_nodes_single`
        """
        return jax.vmap(self.log_likelihood_of_nodes_single)(x)

    def validate(self):
        """
        Validate the parameters and their layouts.
        """
        raise NotImplementedError

    @property
    def number_of_nodes(self) -> int:
        """
        :return: The number of nodes in the layer.
        """
        raise NotImplementedError

    def all_layers(self) -> List[Layer]:
        """
        :return: A list of all layers in the circuit.
        """
        return [self]

    def all_layers_with_depth(self, depth: int = 0) -> List[Tuple[int, Layer]]:
        """
        :return: A list of tuples of all layers in the circuit with their depth.
        """
        return [(depth, self)]

    def __deepcopy__(self, memo=None) -> "Layer":
        """
        Create a deep copy of the layer.

        :param memo: A dictionary that is used to keep track of objects that have
            already been copied.
        """
        raise NotImplementedError

    def partition(self) -> Tuple[Any, Any]:
        """
        Partition the layer into the parameters and the static structure.

        :return: A tuple containing the parameters and the static structure as pytrees.
        """
        return eqx.partition(self, eqx.is_inexact_array)

    @property
    def number_of_trainable_parameters(self):
        """
        :return: The trainable parameters of the layer and all child layers.
        """
        parameters, _ = self.partition()
        flattened_parameters, _ = tree_flatten(parameters)
        number_of_parameters = sum([len(p) for p in flattened_parameters])
        return number_of_parameters

    @property
    def number_of_components(self) -> int:
        """
        :return: The number of components (leaves + edges) of the entire circuit
        """
        return self.number_of_nodes


class InnerLayer(Layer, ABC):
    """
    Abstract Base Class for inner layers.
    """

    child_layers: List[Layer]
    """
    The child layers of this layer.
    """

    def __init__(self, child_layers: List[Layer]):
        super().__init__()
        self.child_layers = child_layers
        self.variables  # initialize the variables of the layer

    def set_variables(self, value: jnp.array):
        raise AttributeError("Variables of inner layers are read-only.")

    def reset_variables(self):
        object.__setattr__(self, "_variables", None)

    def all_layers(self) -> List[Layer]:
        """
        :return: A list of all layers in the circuit.
        """
        result = [self]
        for child_layer in self.child_layers:
            result.extend(child_layer.all_layers())
        return result

    def all_layers_with_depth(self, depth: int = 0) -> List[Tuple[int, Layer]]:
        """
        :return: A list of tuples of all layers in the circuit with their depth.
        """
        result = [(depth, self)]
        for child_layer in self.child_layers:
            result.extend(child_layer.all_layers_with_depth(depth + 1))
        return result

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["child_layers"] = [
            child_layer.to_json() for child_layer in self.child_layers
        ]
        return result


class InputLayer(Layer, ABC):
    """
    Abstract base class for univariate input units.

    Input layers contain only one type of distribution such that the vectorization of
    the log likelihood calculation works without bottleneck statements like if/else or
    loops.
    """

    def __init__(self, variable: int):
        super().__init__()
        self._variables = jnp.array([variable])

    @property
    def variables(self) -> jax.Array:
        return self._variables

    def set_variables(self, value: jax.Array):
        object.__setattr__(self, "_variables", value)

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["variable"] = self._variables[0].item()
        return result

    @property
    def variable(self):
        return self._variables[0].item()

    def log_likelihood_of_nodes_single(self, x: Array) -> Array:
        return self.log_likelihood_of_nodes_of_value(x[self._variables])

    @abstractmethod
    def log_likelihood_of_nodes_of_value(self, value: Array) -> Array:
        """
        Calculate the log-likelihood of every node for a value of the variable.

        :param value: The value of the variable of this layer, with shape (1,).
        :return: The log-likelihood of every node in the layer.
        """


class SumLayer(InnerLayer, ABC):
    log_weights: List[Union[jax.array, BCOO]]
    child_layers: Union[List[[ProductLayer]], List[InputLayer]]

    def __init__(
        self, child_layers: List[Layer], log_weights: List[Union[jax.array, BCOO]]
    ):
        super().__init__(child_layers)
        self.log_weights = log_weights

    def validate(self):
        for log_weights in self.log_weights:
            if not log_weights.shape[0] == self.number_of_nodes:
                raise ShapeMismatchError(self.number_of_nodes, log_weights.shape[0])

        for log_weights, child_layer in self.log_weighted_child_layers:
            if not log_weights.shape[1] == child_layer.number_of_nodes:
                raise ShapeMismatchError(
                    child_layer.number_of_nodes,
                    log_weights.shape[1],
                )

    @property
    def log_weighted_child_layers(self) -> Iterator[Tuple[BCOO, Layer]]:
        """
        :returns: Yields log log_weights and the child layers zipped together.
        """
        yield from zip(self.log_weights, self.child_layers)

    @property
    def variables(self) -> jax.Array:
        if self._variables is None:
            object.__setattr__(self, "_variables", self.child_layers[0].variables)
        return self._variables

    @property
    def number_of_nodes(self) -> int:
        return self.log_weights[0].shape[0]


class SparseSumLayer(SumLayer):
    log_weights: List[BCOO]

    @property
    def number_of_components(self) -> int:
        return sum([cl.number_of_components for cl in self.child_layers]) + sum(
            [lw.nse for lw in self.log_weights]
        )

    @property
    def concatenated_log_weights(self) -> BCOO:
        """
        :return: The concatenated log_weights of the child layers for each node.
        """
        return bcoo_concatenate(self.log_weights, dimension=1).sort_indices()

    @property
    def log_normalization_constants(self) -> jax.Array:
        result = self.concatenated_log_weights
        maximum = result.data.max()
        result.data = jnp.exp(result.data - maximum)
        result = result.sum(1).todense()
        return maximum + jnp.log(result)

    @property
    def normalized_weights(self):
        """
        :return: The normalized log_weights of the child layers for each node.
        """
        result = self.concatenated_log_weights
        z = self.log_normalization_constants
        result.data = jnp.exp(result.data - z[result.indices[:, 0]])
        return result

    def log_likelihood_of_nodes_single(self, x: jax.Array) -> jax.Array:
        result = jnp.zeros(self.number_of_nodes, dtype=jnp.float32)

        for log_weights, child_layer in self.log_weighted_child_layers:
            # get the log likelihoods of the child nodes
            child_layer_log_likelihood = child_layer.log_likelihood_of_nodes_single(x)

            # weight the log likelihood of the child nodes by the weight for each node of this layer
            cloned_log_weights = copy_bcoo(log_weights)  # clone the log_weights

            # multiply the log_weights with the child layer likelihood
            cloned_log_weights.data += child_layer_log_likelihood[
                cloned_log_weights.indices[:, 1]
            ]
            cloned_log_weights.data = jnp.exp(
                cloned_log_weights.data
            )  # exponent log_weights
            result = result.at[cloned_log_weights.indices[:, 0]].add(
                cloned_log_weights.data, indices_are_sorted=False, unique_indices=False
            )

        return jnp.log(result) - self.log_normalization_constants

    def __deepcopy__(self, memo=None):
        if memo is None:
            memo = {}
        id_self = id(self)
        if id_self in memo:
            return memo[id_self]
        child_layers = [
            child_layer.__deepcopy__(memo) for child_layer in self.child_layers
        ]
        log_weights = [copy_bcoo(log_weight) for log_weight in self.log_weights]
        result = self.__class__(child_layers, log_weights)
        memo[id_self] = result
        return result

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["log_weights"] = [
            (lw.data.tolist(), lw.indices.tolist(), lw.shape) for lw in self.log_weights
        ]
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        child_layer = [
            Layer.from_json(child_layer) for child_layer in data["child_layers"]
        ]
        log_weights = [
            BCOO(
                (jnp.array(lw[0]), jnp.array(lw[1])),
                shape=lw[2],
                indices_sorted=True,
                unique_indices=True,
            )
            for lw in data["log_weights"]
        ]
        return cls(child_layer, log_weights)


class DenseSumLayer(SumLayer):
    log_weights: List[jnp.array]
    child_layers: Union[List[[ProductLayer]], List[InputLayer]]

    @property
    def number_of_components(self) -> float:
        return sum([cl.number_of_components for cl in self.child_layers]) + sum(
            [math.prod(lw.shape) for lw in self.log_weights]
        )

    @property
    def concatenated_log_weights(self) -> Array:
        """
        :return: The concatenated log_weights of the child layers for each node.
        """
        return jnp.concatenate(self.log_weights, axis=1)

    @property
    def log_normalization_constants(self) -> jax.Array:
        return logsumexp(self.concatenated_log_weights, 1)

    @property
    def normalized_weights(self):
        """
        :return: The normalized log_weights of the child layers for each node.
        """
        return jnp.exp(
            self.concatenated_log_weights
            - self.log_normalization_constants.reshape(-1, 1)
        )

    def log_likelihood_of_nodes_single(self, x: jax.Array) -> jax.Array:
        result = jnp.zeros(self.number_of_nodes, dtype=jnp.float32)

        for log_weights, child_layer in self.log_weighted_child_layers:
            # get the log likelihoods of the child nodes
            child_layer_log_likelihood = child_layer.log_likelihood_of_nodes_single(x)

            # weight the log likelihood of the child nodes by the weight for each node of this layer
            log_likelihood = log_weights + child_layer_log_likelihood
            log_likelihood = jnp.exp(logsumexp(log_likelihood, 1))
            result += log_likelihood

        return jnp.log(result) - self.log_normalization_constants

    def __deepcopy__(self, memo=None):
        if memo is None:
            memo = {}
        id_self = id(self)
        if id_self in memo:
            return memo[id_self]
        child_layers = [
            child_layer.__deepcopy__(memo) for child_layer in self.child_layers
        ]
        log_weights = [jnp.copy(log_weight) for log_weight in self.log_weights]
        result = self.__class__(child_layers, log_weights)
        memo[id_self] = result
        return result

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["log_weights"] = [lw.tolist() for lw in self.log_weights]
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        child_layer = [
            Layer.from_json(child_layer) for child_layer in data["child_layers"]
        ]
        log_weights = [jnp.asarray(lw) for lw in data["log_weights"]]
        return cls(child_layer, log_weights)


class ProductLayer(InnerLayer):
    """
    A layer that represents the product of multiple other units.
    """

    child_layers: List[Union[SparseSumLayer, InputLayer]]
    """
    The child of a product layer is a list that contains groups sum units with the same
    scope or groups of input units with the same scope.
    """

    edges: Int[BCOO, "len(child_layers), number_of_nodes"] = eqx.field(static=True)
    """
    The edges consist of a sparse matrix containing integers.

    The first dimension describes the edges for each child layer. The second dimension
    describes the edges for each node in the child layer. The integers are interpreted
    in such a way that n-th value represents a edge (n, edges[n]).

    Nodes in the child layer can be mapped to by multiple nodes in this layer.

    The shape is (#child_layers, #nodes).
    """

    def __init__(self, child_layers: List[Layer], edges: BCOO):
        """
        Initialize the product layer.

        :param child_layers: The child layers of the product layer.
        :param edges: The edges of the product layer.
        """
        super().__init__(child_layers)
        self.edges = edges
        self.variables

    def validate(self):
        if not self.edges.shape == (len(self.child_layers), self.number_of_nodes):
            raise ShapeMismatchError(
                (len(self.child_layers), self.number_of_nodes), self.edges.shape
            )

    @property
    def number_of_nodes(self) -> int:
        return self.edges.shape[1]

    @property
    def number_of_components(self) -> int:
        return (
            sum([cl.number_of_components for cl in self.child_layers]) + self.edges.nse
        )

    @Layer.variables.getter
    def variables(self) -> jax.Array:
        if self._variables is None:
            variables = jnp.concatenate(
                [child_layer.variables for child_layer in self.child_layers]
            )
            variables = jnp.unique(variables)
            object.__setattr__(self, "_variables", variables)
        return self._variables

    def log_likelihood_of_nodes_single(self, x: jax.Array) -> jax.Array:
        result = jnp.zeros(self.number_of_nodes, dtype=jnp.float32)

        for edges, layer in zip(self.edges, self.child_layers):
            # every layer reads the variables of its scope from the whole event
            ll = layer.log_likelihood_of_nodes_single(x)  # shape: #child_nodes

            # gather the ll at the indices of the nodes that are required for the edges
            ll = ll[edges.data]  # shape: #len(edges.values())

            # add the gathered values to the result where the edges define the indices
            result = result.at[edges.indices[:, 0]].add(ll)

        return result

    def __deepcopy__(self, memo=None):
        if memo is None:
            memo = {}
        id_self = id(self)
        if id_self in memo:
            return memo[id_self]
        child_layers = [
            child_layer.__deepcopy__(memo) for child_layer in self.child_layers
        ]
        edges = copy_bcoo(self.edges)
        result = self.__class__(child_layers, edges)
        memo[id_self] = result
        return result

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["edges"] = (
            self.edges.data.tolist(),
            self.edges.indices.tolist(),
            self.edges.shape,
        )
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        child_layer = [
            Layer.from_json(child_layer) for child_layer in data["child_layers"]
        ]
        edges = BCOO(
            (jnp.array(data["edges"][0]), jnp.array(data["edges"][1])),
            shape=data["edges"][2],
            indices_sorted=True,
            unique_indices=True,
        )
        return cls(child_layer, edges)
