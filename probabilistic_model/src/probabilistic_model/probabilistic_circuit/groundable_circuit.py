from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from random_events.product_algebra import Event, VariableMap
from random_events.variable import Variable
from typing_extensions import TYPE_CHECKING, Dict, Iterable, Optional, Self, Tuple, Type

from probabilistic_model.probabilistic_model import ProbabilisticModel

if TYPE_CHECKING:
    from probabilistic_model.probabilistic_circuit.relational.exchangeable_grounding import (
        ExchangeablePartGrounder,
    )


@dataclass(eq=False)
class GroundableCircuit(ProbabilisticModel, ABC):
    """
    A circuit type that relational probabilistic circuits are fitted and grounded in.
    """

    @classmethod
    @abstractmethod
    def exchangeable_part_grounder_type(cls) -> Type[ExchangeablePartGrounder]:
        """
        :return: What grounds one exchangeable relation into a circuit of this type.
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def variable_to_index_map(self) -> Dict[Variable, int]:
        """
        :return: The index of every variable in the variables of the circuit.
        """
        raise NotImplementedError

    @abstractmethod
    def log_mode(self, check_determinism: bool = True) -> Tuple[Event, float]:
        """
        :param check_determinism: Whether to check that the circuit is deterministic.
        :return: The mode of the circuit and its log-likelihood.
        """
        raise NotImplementedError

    @abstractmethod
    def marginal_in_place(self, variables: Iterable[Variable]) -> Optional[Self]:
        """
        Marginalize the circuit to the variables in place.

        :param variables: The variables to keep.
        :return: The marginalized circuit, or ``None`` if it keeps no variable.
        """
        raise NotImplementedError

    @abstractmethod
    def update_variables(self, new_variables: VariableMap):
        """
        Replace variables of the circuit in place.

        :param new_variables: The new variable of every variable to replace.
        """
        raise NotImplementedError

    @abstractmethod
    def __deepcopy__(self, memo=None) -> Self:
        raise NotImplementedError
