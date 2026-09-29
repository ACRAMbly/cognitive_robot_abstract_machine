"""
A class whose instances expose attributes beyond the ones they are constructed with.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Rectangle:
    """
    A rectangle constructed from its sides, exposing its area as a derived attribute.
    """

    width: float
    """
    The length of the horizontal sides.
    """

    height: float
    """
    The length of the vertical sides.
    """

    @property
    def area(self) -> float:
        """
        :return: The area the sides enclose.
        """
        return self.width * self.height

    def perimeter(self) -> float:
        """
        :return: The length of the boundary.
        """
        return 2 * (self.width + self.height)
