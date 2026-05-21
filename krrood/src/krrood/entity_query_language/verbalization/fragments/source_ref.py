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
