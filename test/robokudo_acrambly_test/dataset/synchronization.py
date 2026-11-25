# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic, sleep

from typing_extensions import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from semantic_digital_twin.adapters.ros.messages import StreamPosition
    from semantic_digital_twin.adapters.ros.world_synchronizer import (
        WorldSynchronizer,
    )

# %% Bounded waiting


@dataclass(kw_only=True)
class DeliveryWait:
    """
    Bounded wait for ROS messages published within the same test process.
    """

    timeout_seconds: float = 10.0
    """
    Longest wait before the condition counts as never met.
    """

    poll_interval_seconds: float = 0.02
    """
    Pause between two checks of the condition.
    """

    def until(self, condition: Callable[[], bool]) -> bool:
        """
        Return whether the condition became true before the timeout.
        """
        deadline = monotonic() + self.timeout_seconds
        while monotonic() < deadline:
            if condition():
                return True
            sleep(self.poll_interval_seconds)
        return condition()


def has_buffered(*, synchronizer: WorldSynchronizer, position: StreamPosition) -> bool:
    """
    Return whether a held-back message at the stream position has arrived.
    """
    return any(
        message.meta_data == position.origin
        and message.sequence_number == position.sequence_number
        for message in list(synchronizer.missed_messages)
    )
