from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class SourceRef:
    """Carries a reference to the Python source entity a fragment represents."""

    cls: type
    """The class this fragment refers to (always set)."""

    attribute: Optional[str] = None
    """Attribute name within *cls*; ``None`` means the fragment refers to the class itself."""

    @classmethod
    def for_type(cls, t) -> Optional["SourceRef"]:
        """Return SourceRef(cls=t) when t is a real type, else None."""
        return cls(cls=t) if isinstance(t, type) else None

    @classmethod
    def for_attribute(cls, owner, attr_name: str) -> Optional["SourceRef"]:
        """Return SourceRef(cls=owner, attribute=attr_name) when owner is a real type, else None."""
        return cls(cls=owner, attribute=attr_name) if isinstance(owner, type) else None
