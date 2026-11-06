# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from py_trees.common import Status

from robokudo.pipeline import Pipeline

# %% Completed pipeline captures


@dataclass
class AircraftBeliefCapture:
    """
    Complete a due aircraft capture and its threaded belief update.
    """

    pipeline: Pipeline
    """Aircraft pipeline with its observation source followed by the connector."""

    def complete(self) -> None:
        """
        Tick one due capture and wait for its local belief consumer.
        """
        self.pipeline.tick_once()
        if self.pipeline.status is not Status.RUNNING:
            return
        connector = self.pipeline.get_annotators()[-1]
        connector.compute_worker_thread.join()
        self.pipeline.tick_once()
