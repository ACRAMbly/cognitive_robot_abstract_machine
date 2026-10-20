from __future__ import annotations

from abc import ABC

from typing_extensions import Any, Self

from krrood.patterns.exceptions import InterfaceInstantiationError

# %% interfaces


class Interface(ABC):
    """
    A class that only declares what its subclasses provide.

    An interface cannot be instantiated, but its subclasses can. Its annotations declare
    that every subclass has these attributes with these types, whether the subclass
    stores them as fields or computes them as properties.

    .. note::
        Subclass :class:`Interface` when the class only declares the attributes and
        methods its subclasses provide, without fields or behaviour of its own. Subclass
        :class:`abc.ABC` instead when the base class also holds fields or behaviour that
        its subclasses inherit.
        Never make an interface a dataclass: that turns its declared attributes into
        fields. A base class that needs to be a dataclass subclasses :class:`abc.ABC`.
        Only a class that names :class:`Interface` among its own bases is an interface,
        so an interface extending another one names :class:`Interface` again.
    """

    def __new__(cls, *args: Any, **kwargs: Any) -> Self:
        if Interface.is_interface(cls):
            raise InterfaceInstantiationError(cls)
        return super().__new__(cls)

    @staticmethod
    def is_interface(class_: type) -> bool:
        """
        :param class_: The class to check.
        :return: Whether the class names :class:`Interface` among its own bases.
        """
        return Interface in class_.__bases__
