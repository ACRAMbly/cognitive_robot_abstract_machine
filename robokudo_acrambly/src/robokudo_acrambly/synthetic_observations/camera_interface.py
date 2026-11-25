"""
A camera interface that delivers scripted observations instead of sensor data, so a
pipeline reads them through RoboKudo's own collection reader.
"""

from __future__ import annotations

# %% Imports

from dataclasses import dataclass, field
from datetime import timedelta

from krrood.patterns.classproperty import classproperty
from robokudo.descriptors.camera_configs.base_camera_config import BaseCameraConfig
from robokudo.io.camera_interface import CameraInterface
from robokudo.types.annotation import Classification, PoseAnnotation
from robokudo.types.scene import ObjectHypothesis
from typing_extensions import TYPE_CHECKING

from robokudo_acrambly.synthetic_observations.playback import (
    Clock,
    Playback,
    SystemClock,
)

if TYPE_CHECKING:
    from robokudo.cas import CAS
    from semantic_digital_twin.world import World

    from robokudo_acrambly.synthetic_observations.scenario import (
        Scenario,
        ScenarioPart,
    )


# %% Synthetic observation interface


@dataclass(kw_only=True, eq=False)
class SyntheticObservationInterface(CameraInterface):
    """
    Delivers each pass of a scenario once its capture time has come, as the object
    hypotheses a segmentation followed by 6D pose estimation leaves in the CAS: per part,
    its classification and its pose in the camera optical frame.

    The CAS also gets the camera observation of the pass, with its scheduled capture time
    and no camera pose in the world. Nothing is added to the world.
    """

    camera_config: SyntheticCameraConfig
    """
    The scenario to deliver.
    """

    clock: Clock = field(default_factory=SystemClock)
    """
    The time the capture times of the passes are compared against.
    """

    playback: Playback = field(init=False)
    """
    The passes delivered so far and the next one.
    """

    def __post_init__(self) -> None:
        super().__init__(self.camera_config)
        self.playback = Playback(
            schedule=self.camera_config.scenario.schedule, clock=self.clock
        )

    def has_new_data(self) -> bool:
        """
        :return: Whether the capture time of the next pass has come; false after the
            last pass. The first call starts the observation sequence.
        """
        return self.playback.is_next_pass_due()

    def adds_missing_camera(self, *, world: World) -> bool:
        """
        Unlike :meth:`CameraInterface.adds_missing_camera`, never adds the camera.
        """
        return False

    def set_data(self, *, cas: CAS, world: World) -> None:
        scheduled_pass = self.playback.take_next_pass()
        scenario = self.camera_config.scenario
        self.store_camera_observation(
            cas=cas,
            world=world,
            camera_model=scenario.camera_model,
            camera_frame=scenario.camera_frame,
            timestamp_nanoseconds=scheduled_pass.capture_time_nanoseconds,
            modalities=(),
            world_T_camera=None,
        )
        cas.annotations.extend(
            self.create_object_hypothesis(part=part, elapsed=scheduled_pass.elapsed)
            for part in scenario.parts
        )

    def create_object_hypothesis(
        self, *, part: ScenarioPart, elapsed: timedelta
    ) -> ObjectHypothesis:
        """
        :param part: A part of the scenario.
        :param elapsed: How long after the start of the sequence the pass is captured.
        :return: The hypothesis about ``part`` in that pass, certain of its label.
        """
        pose = part.path.pose_at(elapsed=elapsed)
        source = type(self).__name__
        hypothesis = ObjectHypothesis()
        hypothesis.annotations.extend(
            [
                Classification(source=source, classname=part.label, confidence=1.0),
                PoseAnnotation(
                    source=source,
                    translation=pose.position.to_np()[:3].tolist(),
                    rotation=pose.quaternion.to_np().tolist(),
                ),
            ]
        )
        return hypothesis


# %% Camera configuration


@dataclass(kw_only=True)
class SyntheticCameraConfig(BaseCameraConfig):
    """
    Configuration of a camera that delivers the passes of a scenario.
    """

    @classproperty
    def registry_name(cls) -> str:
        """
        Identifier selecting this camera configuration.
        """
        return "synthetic_observations"

    interface_type: str = SyntheticObservationInterface.__name__
    """
    The camera interface that delivers the scenario.
    """
    scenario: Scenario
    """
    The scenario to deliver.
    """
