from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from typing_extensions import Iterator, Iterable

from coraplex.locations.sampling import Sampling
from semantic_digital_twin.spatial_types.spatial_types import Pose


@dataclass
class Location(Iterable[Pose], ABC):
    """
    A region of poses the robot can be sent to, iterated as the pose candidates sampled
    from it.
    """

    sampling: Sampling = field(default_factory=Sampling, kw_only=True)
    """
    How this location's candidates are sampled.
    """

    @abstractmethod
    def candidates(self, sampling: Sampling) -> Iterator[Pose]:
        """
        Sample pose candidates from this location.

        Every location says what it does with the terms it is given, so none of them is
        chosen on a caller's behalf.

        :param sampling: How to sample the candidates.
        :return: The pose candidates, in the order they should be tried.
        """

    def ground(self) -> Pose:
        """
        :return: The first pose candidate of this location.
        """
        return next(iter(self))

    def __iter__(self) -> Iterator[Pose]:
        """
        :return: The candidates, sampled as :attr:`sampling` says.

        .. warning::
            Must stay a generator, so nothing is sampled before the first ``next``.
            EQL's ``variable`` calls :func:`iter` on its domain while the plan is built.
        """
        yield from self.candidates(self.sampling)
