import pytest

from experiments.probabilistic_model_experiments.gaussian_mixture_speed import (
    ClusteredSamplesFactory,
    GaussianMixtureBenchmark,
    GaussianMixtureQuery,
)
from probabilistic_model.adapters.rustworkx_tensorized.rustworkx_to_tensorized import (
    RustworkxCircuitToLayeredCircuitConverter,
)


@pytest.fixture(scope="module")
def benchmark() -> GaussianMixtureBenchmark:
    number_of_components = 2
    rustworkx_circuit = ClusteredSamplesFactory(number_of_samples=600).learn_circuit(
        number_of_components
    )
    return GaussianMixtureBenchmark(
        number_of_components,
        rustworkx_circuit,
        RustworkxCircuitToLayeredCircuitConverter.convert(rustworkx_circuit),
    )


def test_every_query_is_measured_once_in_order(benchmark):
    results = benchmark.measure()
    assert [result.query for result in results] == list(GaussianMixtureQuery)
    assert {result.number_of_components for result in results} == {
        benchmark.number_of_components
    }


def test_the_central_box_bounds_the_first_two_variables(benchmark):
    [box] = benchmark.central_box().simple_sets
    assert list(box.keys()) == benchmark.variables[:2]
