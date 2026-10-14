from __future__ import annotations

from dataclasses import dataclass, field
from http import HTTPStatus

from krrood.exceptions import DataclassException


@dataclass
class PartFindingServerError(DataclassException):
    """
    Raised when a part finding server does not answer a query with the parts.
    """

    url: str = field(kw_only=True)
    """
    The address that was requested.
    """

    status_code: int = field(kw_only=True)
    """
    The status the server answered with.
    """

    def error_message(self) -> str:
        return f"The part finding server answered {self.status_code} for '{self.url}'."

    def suggest_correction(self) -> str:
        if self.status_code == HTTPStatus.REQUEST_ENTITY_TOO_LARGE:
            return "simplify the shape, or start the server with a higher face limit."
        if self.status_code == HTTPStatus.BAD_REQUEST:
            return (
                "ask only for part-whole relationships of the annotation type, and make "
                "sure client and server run the same version of semantic_digital_twin."
            )
        return "check that the part finding server is running and reachable."
