from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from random_events.variable import Variable
from sortedcontainers import SortedSet
from typing_extensions import Dict, Tuple

from probabilistic_model.distributions.helper import make_dirac
from probabilistic_model.probabilistic_circuit.groundable_circuit import (
    ConditionedCopies,
    ProductsOfCopies,
)
from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    NodeIndices,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import Layer
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.inner_layer_edge import (
    InnerLayerEdges,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.product_layer import (
    ProductLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.point_masses import (
    point_masses_layer,
)
from probabilistic_model.probabilistic_circuit.tensorized.query_cache import QueryCache
from probabilistic_model.probabilistic_circuit.tensorized.stacked_copies import (
    AlignedCopiesStacker,
    StackedLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.structural_query import (
    StructuralQuery,
)


@dataclass
class ProductFactor:
    """
    One child layer of a product layer, with the node every product multiplies.
    """

    layer: Layer
    """
    The child layer.
    """

    products: NodeIndices
    """
    The products that multiply a node of the child layer.
    """

    nodes: NodeIndices
    """
    The node of the child layer every one of ``products`` multiplies.
    """


@dataclass
class LayeredProducts:
    """
    Products of copies as one product layer, with node ``i`` the ``i``-th product.

    The conditioned copies of one circuit only differ in their weights, so they are
    stacked into one layer graph instead of being built one copy at a time.
    """

    products: ProductsOfCopies
    """
    The products.
    """

    @property
    def variables(self) -> SortedSet[Variable]:
        """
        :return: The variables of the products.
        """
        result = SortedSet(self.products.point_masses)
        for component in self.products.components:
            result.update(component.variables)
        for copies in self.products.conditioned_copies:
            result.update(copies.renaming[variable] for variable in copies.variables)
        return result

    def stacked_copies(self, copies: ConditionedCopies) -> StackedLayer:
        """
        :param copies: Conditioned copies of a circuit.
        :return: The copies stacked, with node ``i`` of the root belonging to the
            ``i``-th product, over the variables of the circuit.
        """
        circuit = copies.circuit
        kept = np.array(
            [variable in copies.variables for variable in circuit.variables]
        )
        layers = []
        for point in self.products.points:
            conditioned = circuit.root.log_conditional_of_point(
                circuit.encoded_point(point),
                StructuralQuery(circuit.variables),
                cache=QueryCache(),
            )
            # a point the circuit deems impossible leaves the copy unconditioned
            impossible = conditioned.log_probabilities[0] == -np.inf
            source = circuit.root if impossible else conditioned.layer
            layers.append(source.marginal(kept))
        stacked = AlignedCopiesStacker().stack(layers)
        stacked.layer.normalize()
        return stacked

    def product_layer(self, variables: SortedSet[Variable]) -> ProductLayer:
        """
        :param variables: The variables of the circuit the layer is attached to.
        :return: The product layer.
        """
        number_of_products = len(self.products.points)
        products = np.arange(number_of_products)
        stacked_of_copies: Dict[Tuple[int, Tuple[Variable, ...]], StackedLayer] = {}

        factors = []
        for copies in self.products.conditioned_copies:
            circuit = copies.circuit
            key = (id(circuit), tuple(copies.variables))
            if key not in stacked_of_copies:
                stacked_of_copies[key] = self.stacked_copies(copies)
            stacked = stacked_of_copies[key]
            remap = np.array(
                [
                    (
                        variables.index(copies.renaming[variable])
                        if variable in copies.variables
                        else -1
                    )
                    for variable in circuit.variables
                ],
                dtype=np.int64,
            )
            layer = stacked.layer.__deepcopy__({})
            layer.remap_variables(remap)
            roots = stacked.nodes_of_copy(
                products, np.zeros(number_of_products, dtype=np.int64)
            )
            factors.append(ProductFactor(layer, products, roots))
        for variable, values in self.products.point_masses.items():
            factors.append(
                ProductFactor(
                    point_masses_layer(
                        variables.index(variable),
                        [make_dirac(variable, value) for value in values],
                    ),
                    products,
                    products,
                )
            )
        for index, component in enumerate(self.products.components):
            component = component.__deepcopy__()
            component.restore_variables(variables)
            factors.append(
                ProductFactor(
                    component.root,
                    np.array([index]),
                    np.zeros(1, dtype=np.int64),
                )
            )

        edges = InnerLayerEdges(
            np.concatenate([factor.products for factor in factors]),
            np.concatenate(
                [
                    np.full(len(factor.products), factor_index, dtype=np.int64)
                    for factor_index, factor in enumerate(factors)
                ]
            ),
            np.concatenate([factor.nodes for factor in factors]),
        )
        return ProductLayer.from_edges(
            [factor.layer for factor in factors], edges, number_of_products
        )
