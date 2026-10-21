from __future__ import annotations

import copy
from dataclasses import dataclass, fields

from typing_extensions import Any, Self

# %% deep copies of dataclasses


@dataclass(eq=False)
class DataclassDeepCopy:
    """
    A dataclass whose deep copy is constructed from deep copies of its constructor
    fields.

    Fields the constructor does not take are derived by the constructor of the copy, and
    an object the copied fields reach more than once is copied once.
    """

    def __deepcopy__(self, memo: dict[int, Any] | None = None) -> Self:
        # a field known only by its name can only be read with getattr
        return self.__class__(
            **{
                field_.name: copy.deepcopy(getattr(self, field_.name), memo)
                for field_ in fields(self)
                if field_.init
            }
        )
