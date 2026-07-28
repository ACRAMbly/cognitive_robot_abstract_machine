import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def test_cv_bridge_converts_depth_image() -> None:
    depth_image = np.array([[0.5, 1.0], [1.5, 2.0]], dtype=np.float32)
    depth_message = Image(
        height=depth_image.shape[0],
        width=depth_image.shape[1],
        encoding="32FC1",
        is_bigendian=False,
        step=depth_image.strides[0],
        data=depth_image.tobytes(),
    )

    converted_depth_image = CvBridge().imgmsg_to_cv2(
        depth_message, desired_encoding="32FC1"
    )

    np.testing.assert_array_equal(converted_depth_image, depth_image)
