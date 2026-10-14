"""
Asking a part finding server, such as the one on neem-4, where the parts of a shape are.
"""

from __future__ import annotations

import gzip
import json
from dataclasses import dataclass, field
from datetime import timedelta
from http import HTTPStatus

import requests

from krrood.adapters.json_serializer import from_json, to_json
from semantic_digital_twin.adapters.part_finding.exceptions import (
    PartFindingServerError,
)
from semantic_digital_twin.adapters.part_finding.protocol import (
    ContentEncoding,
    ContentType,
    HeaderName,
    PartFindingAnswer,
    PartFindingRoute,
    PartFindingServerStatus,
)
from semantic_digital_twin.pipeline.part_locating import (
    LocatedPart,
    PartLocator,
    PartQuery,
)


@dataclass
class RemotePartLocator(PartLocator):
    """
    Locates parts by asking a part finding server.
    """

    base_url: str = "http://neem-4.informatik.uni-bremen.de:8790"
    """
    The address of the server, which is reachable from within the university's network.
    """

    timeout: timedelta = timedelta(minutes=5)
    """
    How long to wait for an answer, including the time the query waits behind others.
    """

    session: requests.Session = field(default_factory=requests.Session)
    """
    The connection the queries are sent on.
    """

    def locate(self, query: PartQuery) -> list[LocatedPart]:
        return self.answer(query).parts

    def answer(self, query: PartQuery) -> PartFindingAnswer:
        """
        :param query: The shape and the parts wanted.
        :return: The server's whole answer, with the parts and how it found them.
        """
        url = self._url(PartFindingRoute.PARTS)
        response = self.session.post(
            url,
            data=gzip.compress(json.dumps(to_json(query)).encode()),
            headers={
                HeaderName.CONTENT_ENCODING: ContentEncoding.GZIP,
                HeaderName.CONTENT_TYPE: ContentType.JSON,
            },
            timeout=self.timeout.total_seconds(),
        )
        if response.status_code != HTTPStatus.OK:
            raise PartFindingServerError(url=url, status_code=response.status_code)
        return from_json(response.json())

    def status(self) -> PartFindingServerStatus:
        """
        :return: Which locator the server runs and how busy it is.
        """
        url = self._url(PartFindingRoute.STATUS)
        response = self.session.get(url, timeout=self.timeout.total_seconds())
        if response.status_code != HTTPStatus.OK:
            raise PartFindingServerError(url=url, status_code=response.status_code)
        return from_json(response.json())

    def _url(self, route: PartFindingRoute) -> str:
        """
        :param route: A path the server answers.
        :return: Its address.
        """
        return f"{self.base_url.rstrip('/')}{route}"
