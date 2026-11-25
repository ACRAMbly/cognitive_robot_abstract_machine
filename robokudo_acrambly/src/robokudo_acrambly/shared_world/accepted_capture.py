# %% Imports

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import StrEnum
from threading import Lock

import std_msgs.msg
from py_trees.common import Status
from rclpy.qos import DurabilityPolicy, QoSProfile
from typing_extensions import TYPE_CHECKING

from krrood.adapters.json_serializer import from_json, to_json
from krrood.ormatic.utils import classproperty
from robokudo.annotators.core import BaseAnnotator
from robokudo.types.scene import ObjectHypothesis
from robokudo.world import get_object_belief_states, world_instance
from semantic_digital_twin.adapters.ros.node_registry import ROSNodeRegistry
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer

if TYPE_CHECKING:
    from uuid import UUID

    from rclpy.node import Node
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription

    from robokudo.cas import CAS
    from semantic_digital_twin.adapters.ros.messages import StreamPosition


# %% Accepted capture message


class AcceptedCaptureTopic(StrEnum):
    """
    ROS topics carrying accepted captures beside the synchronized world.
    """

    ACCEPTED_CAPTURES = "/robokudo_acrambly/accepted_captures"
    """One message per capture from which at least one part was accepted."""


@dataclass(kw_only=True)
class AcceptedCapture:
    """
    The parts RoboKudo accepted from one capture, and when that capture was taken.

    Sent as ``std_msgs/String`` holding the JSON of this dataclass.
    """

    capture_time: int
    """Capture time of the observation, in integer Unix nanoseconds."""

    belief_ids: list[UUID]
    """Belief UUIDs of the parts accepted from this capture."""

    stream_position: StreamPosition
    """
    Position of RoboKudo's world synchronization stream once the accepted poses were
    published; a receiving world holds them when it has applied this position.
    """

    @classproperty
    def quality_of_service(cls) -> QoSProfile:
        """
        Volatile delivery: a receiver started late waits for the next capture.
        """
        return QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

    def to_message(self) -> std_msgs.msg.String:
        """
        Convert to the ROS message published on the accepted capture topic.
        """
        return std_msgs.msg.String(data=json.dumps(to_json(self)))

    @classmethod
    def from_message(cls, message: std_msgs.msg.String) -> AcceptedCapture:
        """
        Read an accepted capture from its ROS message.
        """
        return from_json(json.loads(message.data))


# %% Publishing annotator


@dataclass(eq=False)
class AcceptedCapturePublisher(BaseAnnotator):
    """
    Publish the parts the preceding connector accepted from the current capture.
    """

    publisher: Publisher = field(init=False)
    """Publisher on the accepted capture topic, created on the shared ROS node."""

    def __post_init__(self) -> None:
        """
        Initialize the annotator and its publisher on the process's shared ROS node.
        """
        BaseAnnotator.__init__(self, name=type(self).__name__)
        self.publisher = (
            ROSNodeRegistry()
            .get()
            .create_publisher(
                std_msgs.msg.String,
                AcceptedCaptureTopic.ACCEPTED_CAPTURES,
                AcceptedCapture.quality_of_service,
            )
        )

    def update(self) -> Status:
        cas = self.get_cas()
        belief_ids = self.find_accepted_belief_ids(cas=cas)
        if not belief_ids:
            return Status.SUCCESS
        capture = AcceptedCapture(
            capture_time=cas.data_timestamp,
            belief_ids=belief_ids,
            stream_position=WorldSynchronizer.of_world(
                world_instance()
            ).latest_published_position,
        )
        self.publisher.publish(capture.to_message())
        return Status.SUCCESS

    @staticmethod
    def find_accepted_belief_ids(*, cas: CAS) -> list[UUID]:
        """
        Return the beliefs whose latest hypothesis is one of the capture's hypotheses.
        """
        hypothesis_ids = {
            id(hypothesis)
            for hypothesis in cas.filter_annotations_by_type(ObjectHypothesis)
        }
        return [
            belief.uuid
            for belief in get_object_belief_states().values()
            if id(belief.latest_hypothesis) in hypothesis_ids
        ]


# %% Receiver


@dataclass(kw_only=True)
class PartCapture:
    """
    The newest accepted capture of one part.
    """

    capture_time: int
    """Capture time of the observation, in integer Unix nanoseconds."""

    stream_position: StreamPosition
    """Position of RoboKudo's stream holding the part's pose from this capture."""

    def is_applied(self, *, synchronizer: WorldSynchronizer) -> bool:
        """
        Return whether the world of the synchronizer holds the pose from this capture.
        """
        return synchronizer.has_applied(self.stream_position)


@dataclass(kw_only=True)
class AcceptedCaptureReceiver:
    """
    Record the newest accepted capture of every part, without history.
    """

    node: Node
    """ROS node receiving accepted captures."""

    part_captures: dict[UUID, PartCapture] = field(init=False, default_factory=dict)
    """Newest accepted capture per part belief UUID."""

    subscription: Subscription | None = field(init=False, default=None)
    """Subscription on the accepted capture topic."""

    lock: Lock = field(init=False, default_factory=Lock, repr=False)
    """Guards :attr:`part_captures` against the subscription thread."""

    def __post_init__(self) -> None:
        """
        Subscribe to the accepted capture topic.
        """
        self.subscription = self.node.create_subscription(
            std_msgs.msg.String,
            AcceptedCaptureTopic.ACCEPTED_CAPTURES,
            self.record_capture,
            AcceptedCapture.quality_of_service,
        )

    def record_capture(self, message: std_msgs.msg.String) -> None:
        """
        Make the received capture the newest capture of each part it lists.
        """
        capture = AcceptedCapture.from_message(message)
        part_capture = PartCapture(
            capture_time=capture.capture_time,
            stream_position=capture.stream_position,
        )
        with self.lock:
            for belief_id in capture.belief_ids:
                previous = self.part_captures.get(belief_id)
                if previous is None or previous.capture_time <= capture.capture_time:
                    self.part_captures[belief_id] = part_capture

    def find_part_capture(self, *, belief_id: UUID) -> PartCapture | None:
        """
        Return the newest accepted capture of a part, if one was received.
        """
        with self.lock:
            return self.part_captures.get(belief_id)

    def close(self) -> None:
        """
        Stop receiving accepted captures.
        """
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
