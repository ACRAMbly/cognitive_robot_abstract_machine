"""
How pose candidates are sampled from a rated set.
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Optional


@dataclass
class Sampling:
    """
    How a location samples its pose candidates: how many, and from which seed.
    """

    number_of_samples: int = 2000
    """
    How many candidates to sample.

    Far more than a caller judges properly, since a standing pose inside the furniture
    costs nothing to refuse.
    """

    seed: Optional[int] = None
    """
    Fixes the sampling, so a run can be repeated exactly.

    ``None`` samples afresh every time, which is what sampling from a map buys over
    reading it off in the order the map rates it.
    """
