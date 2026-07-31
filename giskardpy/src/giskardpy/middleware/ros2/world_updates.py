from __future__ import annotations

from dataclasses import dataclass

from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelReloadSynchronizer,
    WorldSynchronizer,
)


@dataclass
class IncomingWorldUpdates:
    """
    The world updates other processes sent to Giskard.

    The updates arrive on a ros thread but are applied on the thread that owns the
    world, so that the world cannot change in the middle of a control cycle. This class
    answers the only two questions the loops of Giskard have about them: what may be
    applied right now, and whether the structure of the world is about to change.
    """

    world_synchronizer: WorldSynchronizer
    """
    Delivers the model and state updates of other processes.
    """

    model_reload_synchronizer: ModelReloadSynchronizer | None = None
    """
    Delivers requests to replace the whole world model.

    ``None`` when no database is configured, in which case a reload cannot be received.
    """

    @property
    def has_pending_model_change(self) -> bool:
        """
        Whether a change of the structure of the world is waiting to be applied.

        Anything compiled against the current structure becomes invalid once it is, so a
        running motion has to end before it can be applied.
        """
        if self.world_synchronizer.has_buffered_model_modification:
            return True
        if self.model_reload_synchronizer is None:
            return False
        return self.model_reload_synchronizer.has_pending_reload

    def apply_state_updates(self) -> None:
        """
        Apply the state that arrived before the next model change.
        """
        self.world_synchronizer.apply_missed_state_updates()

    def apply_all(self) -> None:
        """
        Apply everything that was received.

        Only safe while nothing is compiled against the structure of the world.
        """
        self.world_synchronizer.apply_missed_messages()
        if self.model_reload_synchronizer is None:
            return
        self.model_reload_synchronizer.apply_pending_reload()

    def acknowledge_receipt(self) -> None:
        """
        Acknowledge the updates that were received but not applied yet.

        Receipt is worth acknowledging on its own: a process publishing synchronously
        may continue as soon as its update arrived, instead of waiting for a motion to
        end.
        """
        self.world_synchronizer.acknowledge_missed_messages()
