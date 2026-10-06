"""
Conversions between the circuits of the ``rx`` and the ``jax`` package, which always go
through the layered circuits of the ``tensorized`` package.
"""

from probabilistic_model.adapters.jax_tensorized.jax_to_tensorized import (
    JaxCircuitToLayeredCircuitConverter,
)
from probabilistic_model.adapters.jax_tensorized.tensorized_to_jax import (
    LayeredCircuitToJaxCircuitConverter,
)
from probabilistic_model.adapters.rustworkx_tensorized.rustworkx_to_tensorized import (
    RustworkxCircuitToLayeredCircuitConverter,
)
from probabilistic_model.adapters.rustworkx_tensorized.tensorized_to_rustworkx import (
    LayeredCircuitToRustworkxCircuitConverter,
)
from probabilistic_model.probabilistic_circuit.jax.probabilistic_circuit import (
    DifferentiableLayeredCircuit,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit as RustworkxProbabilisticCircuit,
)


def jax_circuit_of(
    circuit: RustworkxProbabilisticCircuit,
) -> DifferentiableLayeredCircuit:
    """
    :param circuit: A circuit of the ``rx`` package.
    :return: The circuit of the ``jax`` package with the same distribution.
    """
    return LayeredCircuitToJaxCircuitConverter.convert(
        RustworkxCircuitToLayeredCircuitConverter.convert(circuit)
    )


def rustworkx_circuit_of(
    circuit: DifferentiableLayeredCircuit,
) -> RustworkxProbabilisticCircuit:
    """
    :param circuit: A circuit of the ``jax`` package.
    :return: The circuit of the ``rx`` package with the same distribution.
    """
    return LayeredCircuitToRustworkxCircuitConverter.convert(
        JaxCircuitToLayeredCircuitConverter.convert(circuit)
    )
