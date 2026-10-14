"""
What a part finding server and its clients say to each other over http.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from semantic_digital_twin.pipeline.part_finding.locating import LocatedPart


class PartFindingRoute(StrEnum):
    """
    The paths a part finding server answers.
    """

    PARTS = "/parts"
    """
    Takes a gzip-compressed
    :class:`~semantic_digital_twin.pipeline.part_finding.locating.PartQuery` and answers
    with a :class:`PartFindingAnswer`.
    """

    STATUS = "/status"
    """
    Answers with a :class:`PartFindingServerStatus`.
    """


class EnvironmentVariable(StrEnum):
    """
    The environment variables clients of a part finding server read.
    """

    PART_FINDING_SERVER_URL = "SEMANTIC_DIGITAL_TWIN_PART_FINDING_SERVER_URL"
    """
    The address of the server clients ask by default.
    """


class HeaderName(StrEnum):
    """
    The http headers the server and its clients read or write.
    """

    CONTENT_LENGTH = "Content-Length"
    CONTENT_ENCODING = "Content-Encoding"
    CONTENT_TYPE = "Content-Type"


class ContentType(StrEnum):
    """
    The content types the server and its clients send.
    """

    JSON = "application/json"


class ContentEncoding(StrEnum):
    """
    The encodings a query may be sent in.
    """

    GZIP = "gzip"


@dataclass
class PartFindingAnswer:
    """
    What the server answers a query with.
    """

    parts: list[LocatedPart]
    """
    The parts found.
    """

    locator: str
    """
    The fingerprint of the locator that found them, naming its models and settings.
    """

    seconds: float
    """
    How long finding them took, or reading them from the cache.
    """

    from_cache: bool
    """
    Whether the parts were found for the same query before.
    """


@dataclass
class PartFindingServerStatus:
    """
    What the server answers a status request with.
    """

    locator: str
    """
    The fingerprint of the locator the server finds parts with.
    """

    waiting_queries: int
    """
    How many queries are waiting for the locator, the one it is working on included.
    """
