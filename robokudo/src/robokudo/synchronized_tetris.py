"""Receive accepted CAD beliefs and own their RViz publication."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum

from rclpy.node import Node
from rclpy.timer import Timer

from robokudo.descriptors.analysis_engines.scripted_tetris_sync import (
    SynchronizedTetrisIdentifier,
)
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    ShapeSource,
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.world import World

# %% receiving application


class ReceivingTopic(StrEnum):
    """RViz topics published exclusively by the receiver."""

    VISUAL = "scripted_tetris/visual"
    """Metric CAD mesh markers."""
    COLLISION = "scripted_tetris/collision"
    """Translucent enclosing collision boxes."""
    TRANSFORMS = "tf"
    """Transforms positioned in the receiving world's root."""


@dataclass
class SynchronizedTetrisReceiver:
    """Own native replication and visualization of an independent world."""

    node: Node
    """Borrowed node whose executor is managed by the application."""
    synchronization_topic: str = SynchronizedTetrisIdentifier.TOPIC
    """Dedicated stream carrying accepted world changes."""
    world: World = field(default_factory=World, init=False)
    """Independent world reconstructed from received changes."""
    synchronizer: WorldSynchronizer = field(init=False)
    """Native model and state subscription."""
    visualization_interval: timedelta = timedelta(milliseconds=100)
    """Period for discovering the root and refreshing static-scene transforms."""
    collision_opacity: float = 0.2
    """Opacity of the enclosing collision boxes displayed over CAD meshes."""
    visualization_timer: Timer = field(init=False)
    """Application-owned visualization refresh timer."""
    transform_publisher: TFPublisher | None = field(default=None, init=False)
    """Receiving world's sole transform publisher."""
    visual_publisher: VizMarkerPublisher | None = field(default=None, init=False)
    """CAD mesh markers retained for RViz subscribers."""
    collision_publisher: VizMarkerPublisher | None = field(default=None, init=False)
    """Enclosing collision-box markers."""
    closed: bool = field(default=False, init=False)
    """Whether all owned ROS resources have been released."""

    def __post_init__(self) -> None:
        """Subscribe before permitting source initialization."""
        self.synchronizer = WorldSynchronizer(
            _world=self.world, node=self.node, topic_name=self.synchronization_topic
        )
        self.visualization_timer = self.node.create_timer(
            self.visualization_interval.total_seconds(), self.publish_visualization
        )

    def publish_visualization(self) -> None:
        """Start visualization after root replication and refresh transforms."""
        if self.closed or not self.world.bodies:
            return
        if self.transform_publisher is None:
            self.transform_publisher = TFPublisher(
                _world=self.world,
                node=self.node,
                tf_topic=ReceivingTopic.TRANSFORMS,
            )
            self.visual_publisher = VizMarkerPublisher(
                _world=self.world,
                node=self.node,
                topic_name=ReceivingTopic.VISUAL,
                shape_source=ShapeSource.VISUAL_ONLY,
                tf_publisher=self.transform_publisher,
            )
            self.collision_publisher = VizMarkerPublisher(
                _world=self.world,
                node=self.node,
                topic_name=ReceivingTopic.COLLISION,
                shape_source=ShapeSource.COLLISION_ONLY,
                alpha=self.collision_opacity,
                tf_publisher=self.transform_publisher,
            )
        self.transform_publisher.on_state_change()

    def close(self) -> None:
        """Release timers, callbacks, and ROS entities before node destruction."""
        if self.closed:
            return
        self.closed = True
        self.node.destroy_timer(self.visualization_timer)
        for publisher in (self.collision_publisher, self.visual_publisher):
            if publisher is not None:
                publisher.stop()
                self.node.destroy_publisher(publisher.publisher)
        if self.transform_publisher is not None:
            self.transform_publisher.stop()
            self.node.destroy_publisher(self.transform_publisher.tf_pub)
        self.synchronizer.close()
