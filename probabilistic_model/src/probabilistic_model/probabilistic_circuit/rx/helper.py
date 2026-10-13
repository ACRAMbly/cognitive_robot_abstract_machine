from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import *
from probabilistic_model.distributions.distributions import (
    SymbolicDistribution,
    IntegerDistribution,
    DiracDeltaDistribution,
)
from probabilistic_model.distributions.gaussian import GaussianDistribution
from probabilistic_model.distributions.uniform import UniformDistribution
from probabilistic_model.exceptions import EventWithoutUniformMeasureError
from probabilistic_model.utils import MissingDict


def uniform_measure_of_event(event: Event) -> ProbabilisticCircuit:
    """
    Create a uniform measure for the given event.

    :param event: The event
    :return: The circuit describing the uniform measure
    :raises EventWithoutUniformMeasureError: If the event has no volume in the box
        bounding it.
    """
    # calculate the bounding box of the event
    bounding_box = event.bounding_box()

    # create a uniform measure for the bounding box
    uniform_model = uniform_measure_of_simple_event(bounding_box)
    # condition the uniform measure on the event
    uniform_model, _ = uniform_model.truncated(event)

    if uniform_model is None:
        raise EventWithoutUniformMeasureError(event)

    return uniform_model


def _uniform_measure_of_intervals(
    variable: Continuous,
    intervals: Interval,
    probabilistic_circuit: ProbabilisticCircuit,
) -> SumUnit:
    """
    The uniform measure of a union of intervals, each weighted by its width.

    Points have no width, so they have no mass next to an interval that has. Only a
    union of nothing but points is measured by its points, each equally likely.

    :param variable: The variable the intervals are over.
    :param intervals: The union of intervals.
    :param probabilistic_circuit: The circuit to build the measure in.
    :return: A sum over one leaf per interval that has mass.
    """
    distribution = SumUnit(probabilistic_circuit=probabilistic_circuit)
    intervals_with_width = [
        interval for interval in intervals.simple_sets if not interval.is_singleton()
    ]
    if intervals_with_width:
        leaves_and_log_weights = [
            (
                UniformDistribution(variable=variable, interval=interval),
                np.log(interval.upper - interval.lower),
            )
            for interval in intervals_with_width
        ]
    else:
        leaves_and_log_weights = [
            (
                DiracDeltaDistribution(
                    variable=variable, location=point.lower, density_cap=1.0
                ),
                0.0,
            )
            for point in intervals.simple_sets
        ]
    for leaf_distribution, log_weight in leaves_and_log_weights:
        distribution.add_subcircuit(
            UnivariateContinuousLeaf(
                leaf_distribution, probabilistic_circuit=probabilistic_circuit
            ),
            log_weight,
        )
    distribution.normalize()
    return distribution


def uniform_measure_of_simple_event(simple_event: SimpleEvent) -> ProbabilisticCircuit:
    """
    Create a uniform measure for the given simple event.

    :param simple_event: The simple event
    :return: The circuit describing the uniform measure over the simple event
    """
    # initialize the root of the circuit
    result = ProbabilisticCircuit()
    uniform_model = ProductUnit(probabilistic_circuit=result)
    for variable, assignment in simple_event.items():

        # handle different variables
        if isinstance(variable, Continuous):

            distribution = _uniform_measure_of_intervals(variable, assignment, result)

        # create uniform distribution for symbolic variables
        elif isinstance(variable, Symbolic):
            distribution = SymbolicDistribution(
                variable=variable,
                probabilities=MissingDict(
                    float,
                    {
                        hash(value): 1 / len(assignment.simple_sets)
                        for value in assignment
                    },
                ),
            )
            distribution = UnivariateDiscreteLeaf(
                distribution, probabilistic_circuit=result
            )

        # create uniform distribution for integer variables
        elif isinstance(variable, Integer):
            distribution = IntegerDistribution(
                variable=variable,
                probabilities=MissingDict(
                    float,
                    {
                        value.lower: 1 / len(assignment.simple_sets)
                        for value in assignment
                    },
                ),
            )
            distribution = UnivariateDiscreteLeaf(
                distribution, probabilistic_circuit=result
            )

        else:
            raise NotImplementedError

        # mount the distribution on the root
        uniform_model.add_subcircuit(distribution)

    return result


def fully_factorized(
    variables: Iterable[Variable],
    means: Optional[Dict[Continuous, float]] = None,
    variances: Optional[Dict[Continuous, float]] = None,
) -> ProbabilisticCircuit:
    """
    Create a fully factorized distribution over a set of variables.

    For symbolic variables, the distribution is uniform. For continuous variables, the
    distribution is normal.

    :param variables: The variables.
    :param means: The means of the normal distributions. Defaults to 0 for every not
        specified variable.
    :param variances: The variances of the normal distributions. Defaults to 1 for every
        not specified variable.
    :return: The circuit describing the fully factorized normal distribution
    """
    pc = ProbabilisticCircuit()
    if means is None:
        means = {}

    if variances is None:
        variances = {}

    # initialize the root of the circuit
    root = ProductUnit(probabilistic_circuit=pc)
    for variable in variables:

        # create a normal distribution for every continuous or integer variable
        if isinstance(variable, (Continuous, Integer)):
            distribution = GaussianDistribution(
                variable=variable,
                location=means.get(variable, 0.0),
                scale=variances.get(variable, 1.0),
            )
            distribution = leaf(distribution, pc)

        # create uniform distribution for symbolic variables
        elif isinstance(variable, Symbolic):
            domain_elements = list(variable.domain.simple_sets)
            distribution = SymbolicDistribution(
                variable=variable,
                probabilities=MissingDict(
                    float, {hash(v): 1 / len(domain_elements) for v in domain_elements}
                ),
            )
            distribution = leaf(distribution, pc)
        else:
            raise NotImplementedError(f"Variable type not supported: {variable}")
        root.add_subcircuit(distribution)

    return root.probabilistic_circuit


def multiply_distributions(
    distribution: ProbabilisticCircuit, expansion: ProbabilisticCircuit
):
    """
    Expand the `distribution` by the `expansion` using factorization.

    `distribution` is expanded in-place.

    The resulting distribution is the product of the expansion and the distribution.

    :param distribution: The probabilistic circuit to expand
    :param expansion: The probabilistic circuit to use for expansion
    :return: The expanded probabilistic circuit
    """
    old_distribution_root = distribution.root
    expansion_root_index = expansion.root.index
    remap = distribution.mount(expansion.root)
    ff_root = remap[expansion_root_index]

    new_root = ProductUnit(probabilistic_circuit=distribution)
    new_root.add_subcircuit(ff_root)
    new_root.add_subcircuit(old_distribution_root)
    return distribution
