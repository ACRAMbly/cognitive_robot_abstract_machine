"""
What the exchangeable parts of a relational circuit are fitted with, keyed by EQL
attribute accesses.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

from typing_extensions import Dict, Iterator, MutableMapping, Optional, Union

from krrood.entity_query_language.core.mapped_variable import Attribute
from krrood.entity_query_language.core.variable import Variable
from probabilistic_model.learning.jpt.jpt import JointProbabilityTree
from probabilistic_model.learning.learning_method import LearningMethod

PartKey = Union[Attribute, str]
"""
An attribute access from a variable of the class of a relational circuit to one of its
exchangeable parts, such as ``variable(SceneBuilding).floors.rooms``, or its path
``"floors.rooms"``.
"""


@dataclass
class PartLearningMethods(MutableMapping[PartKey, LearningMethod]):
    """
    A defaultdict of learning methods keyed by attribute accesses to exchangeable parts.

    An attribute access is stored under its path, since attribute accesses hash by
    identity and compare symbolically.
    """

    default: Optional[LearningMethod] = field(default_factory=JointProbabilityTree)
    """
    What a part absent from :attr:`methods` is fitted with, copied for every such part,
    or ``None`` to require every part to be in it.
    """

    methods: Dict[PartKey, LearningMethod] = field(default_factory=dict)
    """
    The learning method of every part, keyed by its path.
    """

    def __post_init__(self):
        self.methods = {
            self.path_of(key): method for key, method in self.methods.items()
        }

    @staticmethod
    def path_of(key: PartKey) -> str:
        """
        :param key: An attribute access on a variable, or a path.
        :return: The path of the key.
        """
        if isinstance(key, str):
            return key
        reversed_names = []
        while isinstance(key, Attribute):
            reversed_names.append(key._attribute_name_)
            key = key._child_
        if not reversed_names or not isinstance(key, Variable):
            raise TypeError(
                f"Expected an attribute access on an EQL variable, got {key!r}."
            )
        return ".".join(reversed(reversed_names))

    def __getitem__(self, key: PartKey) -> LearningMethod:
        path = self.path_of(key)
        if path not in self.methods:
            if self.default is None:
                raise KeyError(path)
            self.methods[path] = copy.deepcopy(self.default)
        return self.methods[path]

    def __setitem__(self, key: PartKey, method: LearningMethod):
        self.methods[self.path_of(key)] = method

    def __delitem__(self, key: PartKey):
        del self.methods[self.path_of(key)]

    def __contains__(self, key: PartKey) -> bool:
        return self.path_of(key) in self.methods

    def __iter__(self) -> Iterator[str]:
        return iter(self.methods)

    def __len__(self) -> int:
        return len(self.methods)

    def of_part(self, part: str) -> PartLearningMethods:
        """
        :param part: The field name of an exchangeable part.
        :return: The learning methods of the parts nested in the part, with the same
            default.
        """
        prefix = f"{part}."
        return PartLearningMethods(
            self.default,
            {
                path.removeprefix(prefix): method
                for path, method in self.methods.items()
                if path.startswith(prefix)
            },
        )
