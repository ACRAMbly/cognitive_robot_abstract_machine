from dataclasses import dataclass, field

import numpy as np

from probabilistic_model.probabilistic_model import ProbabilisticModel


@dataclass
class MonteCarloEstimator:
    """
    This is a wrapper class for using monte carlo estimations of a model that can be
    sampled from and where the likelihood can be evaluated.
    """

    model: ProbabilisticModel
    """
    The wrapped model.
    """

    sample_size: int = field(default=100)
    """
    The number of samples to use for the estimation.
    """

    def l1_metric(self, other: ProbabilisticModel) -> float:
        """
        Estimate the L1 metric :math:`\\int |p(x) - q(x)| dx` between the model
        :math:`p` and another model :math:`q`.

        The estimate is the importance sampling estimate under the equal mixture
        :math:`m = (p + q) / 2`, which draws :attr:`sample_size` samples from each model
        and averages :math:`2 |p(x) - q(x)| / (p(x) + q(x))`. It is unbiased and always
        lies in :math:`[0, 2]`.

        .. seealso::

            E. Veach and L. Guibas, "Optimally combining sampling techniques for
            Monte Carlo rendering", SIGGRAPH 1995 (the balance heuristic).

            V. Elvira, L. Martino, D. Luengo and M. F. Bugallo, "Generalized multiple
            importance sampling", Statistical Science 34(1), 2019.

        :param other: The other model to compare to.
        :return: The estimated L1 metric between the two models.
        """
        samples = np.concatenate(
            [self.model.sample(self.sample_size), other.sample(self.sample_size)]
        )
        likelihood_of_model = self.model.likelihood(samples)
        likelihood_of_other = other.likelihood(samples)
        return np.mean(
            2
            * np.abs(likelihood_of_model - likelihood_of_other)
            / (likelihood_of_model + likelihood_of_other)
        )
