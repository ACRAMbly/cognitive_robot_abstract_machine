from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from random_events.product_algebra import Event
from random_events.sigma_algebra import AbstractCompositeSet
from typing_extensions import Generic, Self, TypeVar

from krrood.patterns.subclass_safe_generic import SubClassSafeGeneric

DistributionType = TypeVar("DistributionType")
"""
What represents the distribution a query leaves behind, such as a model or the root
layer of a circuit.
"""

# %% modes


@dataclass
class Mode:
    """
    The most likely events of a model and their likelihood.
    """

    event: Event
    """
    The set of most likely points.
    """

    likelihood: float
    """
    The likelihood of every point in :attr:`event`.
    """


@dataclass
class LogMode:
    """
    The most likely events of a model and their log-likelihood.
    """

    event: Event
    """
    The set of most likely points.
    """

    log_likelihood: float
    """
    The log-likelihood of every point in :attr:`event`.
    """

    def exponentiated(self) -> Mode:
        """
        :return: This mode with its likelihood instead of its log-likelihood.
        """
        return Mode(self.event, float(np.exp(self.log_likelihood)))


@dataclass
class UnivariateLogMode:
    """
    The most likely values of a univariate distribution and their log-likelihood.
    """

    assignment: AbstractCompositeSet
    """
    The set of most likely values of the variable.
    """

    log_likelihood: float
    """
    The log-likelihood of every value in :attr:`assignment`.
    """


# %% truncations


@dataclass
class Truncation(Generic[DistributionType], SubClassSafeGeneric):
    """
    A model truncated to an event, P(* | event), and the probability of that event.
    """

    distribution: DistributionType | None
    """
    The truncated distribution, or *None* if the event is impossible.
    """

    probability: float
    """
    The probability of the event.
    """


@dataclass
class LogTruncation(Generic[DistributionType], SubClassSafeGeneric):
    """
    A model truncated to an event, P(* | event), and the log-probability of that event.
    """

    distribution: DistributionType | None
    """
    The truncated distribution, or *None* if the event is impossible.
    """

    log_probability: float
    """
    The log-probability of the event.
    """

    @classmethod
    def impossible(cls) -> Self:
        """
        :return: The truncation to an event that has no probability.
        """
        return cls(None, -np.inf)

    def exponentiated(self) -> Truncation[DistributionType]:
        """
        :return: This truncation with the probability instead of the log-probability.
        """
        return Truncation(self.distribution, float(np.exp(self.log_probability)))


# %% conditionings


@dataclass
class Conditioning(Generic[DistributionType], SubClassSafeGeneric):
    """
    A model conditioned on a partial point, P(* | point), and the likelihood of that
    point.
    """

    distribution: DistributionType | None
    """
    The conditioned distribution, or *None* if the point is impossible.
    """

    likelihood: float
    """
    The likelihood of the point.
    """


@dataclass
class LogConditioning(Generic[DistributionType], SubClassSafeGeneric):
    """
    A model conditioned on a partial point, P(* | point), and the log-likelihood of that
    point.
    """

    distribution: DistributionType | None
    """
    The conditioned distribution, or *None* if the point is impossible.
    """

    log_likelihood: float
    """
    The log-likelihood of the point.
    """

    @classmethod
    def impossible(cls) -> Self:
        """
        :return: The conditioning on a point that has no likelihood.
        """
        return cls(None, -np.inf)

    def exponentiated(self) -> Conditioning[DistributionType]:
        """
        :return: This conditioning with the likelihood instead of the log-likelihood.
        """
        return Conditioning(self.distribution, float(np.exp(self.log_likelihood)))
