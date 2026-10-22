# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

import open3d
from sensor_msgs.msg import CameraInfo
from typing_extensions import TYPE_CHECKING

from robokudo.scripted_observations.clock import TimeUnit
from robokudo.utils.camera_model import (
    pinhole_camera_parameters_from_horizontal_field_of_view,
)

if TYPE_CHECKING:
    from robokudo.cas import CAS
    from robokudo.scripted_observations.scenario import CameraSettings


# %% Camera metadata


class DistortionModel(StrEnum):
    """
    ROS camera calibration convention for an undistorted pinhole camera.
    """

    PINHOLE = "plumb_bob"


@dataclass
class ObservationCamera:
    """
    Supply matching optical calibration and capture metadata to a CAS.
    """

    camera_settings: CameraSettings
    """
    Image dimensions, field of view, and optical reference frame.
    """

    def write_metadata(self, observation: CAS, capture_timestamp: int) -> None:
        """
        Attach calibration and scheduled capture time to an observation.
        """
        parameters = pinhole_camera_parameters_from_horizontal_field_of_view(
            self.camera_settings.image_width,
            self.camera_settings.image_height,
            self.camera_settings.horizontal_field_of_view_degrees,
        )
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_settings.camera_frame
        camera_info.header.stamp.sec, camera_info.header.stamp.nanosec = divmod(
            capture_timestamp, TimeUnit.NANOSECONDS_PER_SECOND
        )
        camera_info.width = parameters.width
        camera_info.height = parameters.height
        camera_info.distortion_model = DistortionModel.PINHOLE
        camera_info.d = [0.0] * 5
        camera_info.k = [
            parameters.focal_length_x,
            0.0,
            parameters.center_x,
            0.0,
            parameters.focal_length_y,
            parameters.center_y,
            0.0,
            0.0,
            1.0,
        ]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [
            parameters.focal_length_x,
            0.0,
            parameters.center_x,
            0.0,
            0.0,
            parameters.focal_length_y,
            parameters.center_y,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        observation.camera_frame = self.camera_settings.camera_frame
        observation.data_timestamp = capture_timestamp
        observation.camera_info = camera_info
        observation.camera_intrinsic = open3d.camera.PinholeCameraIntrinsic(
            parameters.width,
            parameters.height,
            parameters.focal_length_x,
            parameters.focal_length_y,
            parameters.center_x,
            parameters.center_y,
        )
        observation.pointcloud_camera_intrinsic = observation.camera_intrinsic
        observation.color2depth_ratio = (1, 1)
