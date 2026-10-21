"""
Tests for what an event logger keeps alive.
"""

from __future__ import annotations

import gc
import weakref

from segmind.event_logger import EventLogger

# %% releasing a logger


def test_an_event_logger_nobody_refers_to_is_released():
    """
    A logger's events refer to the bodies of the world they happened in, so a logger
    kept beyond its run keeps that world alive.
    """
    event_logger = EventLogger()
    reference = weakref.ref(event_logger)

    del event_logger
    gc.collect()

    assert reference() is None
