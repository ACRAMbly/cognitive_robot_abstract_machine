---
jupytext:
  cell_metadata_filter: -all
  formats: md:myst
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.11.5
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Representations of Circuits

While understanding the concepts of a probabilistic circuit is subject to math, implementing it is a whole different
story.
This section discusses different approaches to represent circuits.

## the DAG (rustworkx) way

The easiest and naive way of implementing a circuit is using a directed acyclic graph (DAG).
The graph directly follows definition {prf:ref}`def-probabilistic-circuit`.

Let's look at an example.

```{code-cell} ipython3
import plotly
plotly.offline.init_notebook_mode()
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import *
from probabilistic_model.distributions.distributions import *
from probabilistic_model.distributions.uniform import *
from random_events.variable import Continuous
import networkx as nx
from probabilistic_model.probabilistic_circuit.jax.probabilistic_circuit import ProbabilisticCircuit as JaxPC
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import equinox as eqx


x = Continuous("x")
y = Continuous("y")
model = ProbabilisticCircuit()
sum1, sum2, sum3 = SumUnit(probabilistic_circuit=model), SumUnit(probabilistic_circuit=model), SumUnit(probabilistic_circuit=model)
sum4, sum5 = SumUnit(probabilistic_circuit=model), SumUnit(probabilistic_circuit=model)
prod1, prod2 = ProductUnit(probabilistic_circuit=model), ProductUnit(probabilistic_circuit=model)

sum1.add_subcircuit(prod1, np.log(0.5))
sum1.add_subcircuit(prod2, np.log(0.5))
prod1.add_subcircuit(sum2)
prod1.add_subcircuit(sum4)
prod2.add_subcircuit(sum3)
prod2.add_subcircuit(sum5)

d_x1 = leaf(UniformDistribution(variable=x, interval=SimpleInterval.from_data(0, 1)), probabilistic_circuit=model)
d_x2 = leaf(UniformDistribution(variable=x, interval=SimpleInterval.from_data(2, 3)), probabilistic_circuit=model)
d_y1 = leaf(UniformDistribution(variable=y, interval=SimpleInterval.from_data(0, 1)), probabilistic_circuit=model)
d_y2 = leaf(UniformDistribution(variable=y, interval=SimpleInterval.from_data(3, 4)), probabilistic_circuit=model)

sum2.add_subcircuit(d_x1, np.log(0.8))
sum2.add_subcircuit(d_x2, np.log(0.2))
sum3.add_subcircuit(d_x1, np.log(0.7))
sum3.add_subcircuit(d_x2, np.log(0.3))

sum4.add_subcircuit(d_y1, np.log(0.5))
sum4.add_subcircuit(d_y2, np.log(0.5))
sum5.add_subcircuit(d_y1, np.log(0.1))
sum5.add_subcircuit(d_y2, np.log(0.9))

model.plot_structure()
plt.show()
```

```{code-cell} ipython3
fig = go.Figure(model.plot(), model.plotly_layout())
fig.show()
```

The Benefits of the DAG representation are:
- Understandability
- Simplicity of implementation 
- Low installation overhead
- Great extendability
- Great for structure learning
- Great for teaching

The drawbacks are:
- Python implementations are usually slow
- Rustowrkx does not benefit from SMID instruction like jax would
- No benefit from modern hardware acceleration


## The Layered way
Modern literature suggests representing circuits in a way that is compatible with modern hardware 
acceleration. {cite}`liu2024scaling`, {cite}`peharz2020einsum`. 

Doing so requires a topological sorting of the circuit. In that topological sorting, each layer represents a set of 
nodes at the same depth (distance to the root) that can be computed in parallel.
These nodes have to be of the same type, such that their operations (weighted sum, product, density, etc.) can be computed in parallel.
The example from above would look as following:

```{code-cell} ipython3
jax_model = JaxPC.from_rustworkx(model, progress_bar=False)
print(jax_model.root)
```

The way a layered pc is structured is shown in the figure below.

![Grouped operations in a layered circuit](layered_example.png)

We can see that similar operations have been grouped together. 
Now they can be executed using a jax backend instead of a for loop in python. 
The Benefits of the layered representation are:
- Speed
- Compatible with modern frameworks for machine learning frameworks
- Compatible with modern hardware acceleration
- Most likely the future of probabilistic circuits


The drawbacks are:
- Harder to understand
- Harder to maintain
- Requires quite the overhead to install


## JAX Implementation

As of today, the layered approach is implemented in jax and supports all inferences that do not change the structure of
the circuit. 
These are all but marginalization and conditioning/truncation. 

The JAX implementation uses equinox to aid with an OOP approach to the circuit.
It uses sparse matrices to represent edges between the layers and hence does not suffer from extreme memory consumption
like EinsumNetworks.

JAX layered circuits are approximately **20** times faster than the rustworkx implementation in calculating the 
log-likelihood, and hence are a great tool for doing deep learning with circuits.
For the speed-up to kick in, the JAX computational graph that describes the circuit has to be compiled.
This is expensive, so don't do it more than needed.
However, for a fixed circuit, the speed-up is immense.

In the scripts folder, you can reproduce these results.

Be aware that the JAX implementation is still in development and might not be as stable as the networkx implementation.
I would be happy to get support here if someone is interested in it.

JAX and networkx formats can be converted into each other.

## NumPy Implementation

The JAX implementation trades the structural inferences for hardware acceleration. The
numpy implementation in `probabilistic_model.probabilistic_circuit.tensorized` keeps the layered
layout but gives the structural inferences back, so it supports every query the rustworkx
implementation supports.

It uses the same decomposition into layers: a `SumLayer` stores the weights of all of its
nodes as one sparse matrix whose columns are the nodes of its child layers, a
`ProductLayer` stores the edges of all of its nodes as one sparse integer matrix, and an
input layer stores the parameters of all of its nodes in contiguous arrays. A rustworkx
circuit is converted into a layered one, and back, by the converters in
`probabilistic_model.adapters.rustworkx_tensorized`.

```{code-cell} ipython3
from probabilistic_model.adapters.rustworkx_tensorized.rustworkx_to_tensorized import RustworkxCircuitToLayeredCircuitConverter

numpy_model = RustworkxCircuitToLayeredCircuitConverter.convert(model)
print(numpy_model)
print(numpy_model.root)
```

Queries that do not change the structure are evaluated for all nodes of a layer at once,
just like in JAX:

```{code-cell} ipython3
samples = numpy_model.sample(5)
print(numpy_model.log_likelihood(samples))
print(numpy_model.expectation())
```

The structural queries work as well and return a layered circuit again:

```{code-cell} ipython3
from random_events.interval import closed
from random_events.product_algebra import SimpleEvent

event = SimpleEvent.from_data({x: closed(0.25, 2.5)}).as_composite_set()
truncated, probability = numpy_model.truncated(event)
print(probability)
print(truncated.marginal([y]))
print(numpy_model.conditional({x: 0.5})[0])
```

Three design decisions make this possible:

- Truncating an input layer keeps its number of nodes, so the edges of the parents stay
  valid. When a node splits into several pieces, or when the truncated nodes no longer
  share one type, the pieces are grouped into layers by type and a sum layer selects the
  pieces of each original node. Nodes that became impossible are marked with a
  log-probability of `-inf` and are then removed by a pass that prunes every impossible
  and unreachable node and renumbers the sparse structures.
- Every bottom-up query is memoized by the identity of the layer, so a layer that several
  parents point at is evaluated once rather than once per path.
- A structural pass never writes into the layers it reads; it builds new ones. That is
  what lets a truncation to a composite event work off a single copy of the circuit.

### Truncating to an event with many simple sets

Truncating to an event with `k` simple sets is the most demanding query of the package.
The graph implementation truncates a copy of the circuit to each simple set and mixes the
`k` results. Doing that in a layered circuit would defeat the layout: the result gets one
set of layers per simple set, so a circuit with ten layers turns into one with hundreds of
layers holding a handful of nodes each, and every later query pays python overhead per
layer instead of running over arrays.

The numpy implementation truncates to all `k` simple sets in **one** pass instead. Every
layer is replicated once per simple set inside its own parameter block, so the number of
layers of the result does not depend on `k`, only its blocks are up to `k` times taller. A layer whose type changes with the assignment -- a Gaussian layer becomes a
truncated Gaussian layer, a composite assignment splits a node into several pieces --
still batches fine as long as every simple set in the `k` produces the *same* resulting
type; only a batch whose simple sets disagree on the type (for instance a Gaussian layer
where one simple set leaves the whole real line and another bounds it) reports that it
cannot be batched, and the circuit falls back to truncating once per simple set.

Measured on a joint probability tree with 528 nodes over 4 variables, truncated to a
staircase of disjoint boxes:

| simple sets | rustworkx | numpy layered | layers in the result |
| --- | --- | --- | --- |
| 5 | 76 ms | 6.8 ms | 11 |
| 10 | 192 ms | 9.4 ms | 11 |
| 25 | 331 ms | 17.1 ms | 11 |
| 50 | 703 ms | 30.3 ms | 11 |
| 100 | 1413 ms | 57.4 ms | 11 |

Truncating one simple set at a time instead, the same 100-set result is spread over 821
layers and takes 287 ms to build.

`experiments/src/experiments/probabilistic_model_experiments/layered_circuit_speed.py`
reproduces this table and the query timings below it.

### Speed of the other queries

On the same joint probability tree, before truncation:

| query | rustworkx | numpy layered |
| --- | --- | --- |
| `log_likelihood`, 100 events | 8.6 ms | 2.1 ms |
| `log_likelihood`, 1000 events | 14.6 ms | 15.8 ms |
| `log_likelihood`, 10000 events | 70.9 ms | 161 ms |
| `sample`, 1000 samples | 5.1 ms | 6.6 ms |
| `sample`, 10000 samples | 6.0 ms | 7.6 ms |
| `probability_of_simple_event` | 11.5 ms | 0.7 ms |

and on the circuit truncated to 100 simple sets, which has 5575 nodes:

| query | rustworkx | numpy layered |
| --- | --- | --- |
| `log_likelihood`, 100 events | 77 ms | 14 ms |
| `log_likelihood`, 1000 events | 125 ms | 195 ms |
| `log_likelihood`, 10000 events | 456 ms | 1804 ms |
| `sample`, 1000 samples | 26 ms | 104 ms |
| `sample`, 10000 samples | 42 ms | 66 ms |
| `probability_of_simple_event` | 263 ms | 4.5 ms |

Conditioning on a partial point, on the tree before truncation:

| conditioned variables | rustworkx | numpy layered |
| --- | --- | --- |
| 1 | 14.5 ms | 3.6 ms |
| 2 | 14.9 ms | 3.3 ms |
| 3 | 15.7 ms | 3.1 ms |
| 4 | 17.0 ms | 2.3 ms |

Conditioning sees a smaller speedup than truncation, since there is nothing to batch in
a single point the way there is in a many-simple-set truncation.

The layered layout removes the per-node python overhead, which dominates small queries,
and a query over a `SimpleEvent` becomes one pass over a handful of arrays. It does not
make the *asymptotics* better, and it is slower than the rustworkx implementation for
large batches of events or samples on circuits whose leaves have small disjoint
supports, like the leaves of a joint probability tree: rustworkx evaluates each leaf only
at the events inside its support, while a layer evaluates its whole `(#events, #nodes)`
block.

### Gaussians over several variables

The input layers above hold one variable each. A `MultivariateGaussianLayer` holds
Gaussians over several variables at once, with the mean and covariance matrix of every node
stacked into arrays; it is what the multivariate Gaussian leaves of a Gaussian mixture,
as `GaussianMixtureModel` learns it, convert into. Truncating it to a box turns it into a
`TruncatedMultivariateGaussianLayer`, which stores the box of every node, and truncating to
several boxes mixes the truncations like any other input layer. Conditioning keeps a
Gaussian over the free variables and turns the fixed ones into Dirac deltas.

The marginal over some of the variables of a node confined to a box has no closed form,
since what is left is not a Gaussian confined to a box, so it raises `NoClosedFormError`.
The moments of such a node are integrated numerically by Gauss-Legendre quadrature, with
the settings `MomentIntegration` holds. The probability of a box that bounds one variable
is answered in closed form, in log space so that boxes far in the tails keep a probability
above zero; a box that bounds several variables is integrated numerically by
`scipy.stats.multivariate_normal`, over only the variables it bounds.

Measured on Gaussian mixtures over 4 variables, fitted to clustered samples, against the
rustworkx circuit the learning method returns:

| query | 5 components | 20 components | 80 components |
| --- | --- | --- | --- |
| `log_likelihood`, 1000 events | 1.4 ms / 0.5 ms | 5.5 ms / 1.0 ms | 22 ms / 3.2 ms |
| `log_likelihood`, 10000 events | 3.4 ms / 2.6 ms | 15 ms / 12 ms | 59 ms / 30 ms |
| `sample`, 1000 | 1.6 ms / 0.4 ms | 6.0 ms / 0.5 ms | 22 ms / 1.0 ms |
| probability of a box | 23 ms / 1.6 ms | 76 ms / 5.2 ms | 268 ms / 20 ms |
| truncation to a box | 14 ms / 2.6 ms | 65 ms / 6.2 ms | 280 ms / 21 ms |
| truncation to 10 boxes | 140 ms / 18 ms | 544 ms / 55 ms | 2323 ms / 202 ms |
| `log_likelihood` after truncation, 1000 events | 68 ms / 0.5 ms | 286 ms / 1.4 ms | 1126 ms / 4.8 ms |
| `sample` after truncation, 1000 | 561 ms / 261 ms | 1633 ms / 270 ms | 5596 ms / 276 ms |
| conditioning on one variable | 3.3 ms / 1.9 ms | 12 ms / 2.0 ms | 48 ms / 1.9 ms |
| conditioning on all but one variable | 3.3 ms / 1.8 ms | 12 ms / 1.9 ms | 49 ms / 1.9 ms |

Each cell is rustworkx / numpy layered. The truncated layer samples by Gibbs sampling like
the distribution does, but runs the chains of all of its nodes together, so its cost barely
grows with the number of components.
`experiments/src/experiments/probabilistic_model_experiments/gaussian_mixture_speed.py`
reproduces the table.

Use the rustworkx implementation to build and learn circuits, the numpy implementation
when the same fixed circuit is queried many times and the structural inferences are
needed, and the JAX implementation when the circuit has to be trained by gradient descent.
All three convert into each other.
