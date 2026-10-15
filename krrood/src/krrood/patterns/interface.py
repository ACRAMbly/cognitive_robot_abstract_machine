from __future__ import annotations

from abc import ABC

from typing_extensions import Any, Self

from krrood.patterns.exceptions import InterfaceInstantiationError

# %% interfaces


class Interface(ABC):
    """
    A class that only declares what its subclasses provide.

    A class that names :class:`Interface` among its own bases cannot be instantiated;
    its subclasses can.
    """

    def __new__(cls, *args: Any, **kwargs: Any) -> Self:
        if Interface in cls.__bases__:
            raise InterfaceInstantiationError(cls)
        return super().__new__(cls)
