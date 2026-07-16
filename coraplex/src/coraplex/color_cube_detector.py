#!/usr/bin/env python3
"""Color cube detector v2 — refined for the real Tracy setup.

Detects colored cubes (red/blue/yellow) in RGB-D and publishes 6D poses.

Refinements over v1:
  * cube_size-aware area filtering (expected pixel area from intrinsics+depth)
  * depth band filter (rejects background/floor false positives)
  * optional workspace filter: transforms detections into the `table` frame
    via live TF (uses the calibrated URDF) and rejects anything off the
    1.18 x 1.60 m tabletop
  * publishes CUBE CENTER (top-face depth + half cube size along view axis)
  * per-color topics in the optical frame (for the seam) AND in the table
    frame (for direct validation against the 10 cm grid)
  * red hue-wrap handled (two HSV ranges); no cv_bridge dependency

Orientation remains APPROXIMATE: yaw-only about the optical axis from the
contour's minAreaRect; roll=pitch=0 (cube assumed flat on the table).

Run (Tracy PC, ROS sourced, ROS_DOMAIN_ID=2, cram-env):
  python3 -m coraplex.color_cube_detector \
      --ros-args -p colors:="['red','blue','yellow']" -p cube_size:=0.05

Outputs:
  /foundationpose/object_pose                 (all cubes, optical frame)
  /foundationpose/object_pose/<color>         (optical frame -> the seam)
  /foundationpose/object_pose/<color>/table   (table frame -> validation)
"""
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

try:
    import tf2_ros
    HAVE_TF = True
except ImportError:  # workspace filter degrades gracefully
    HAVE_TF = False


# ---------------- image conversion (no cv_bridge) ----------------

def image_msg_to_numpy(msg) -> np.ndarray:
    enc = msg.encoding
    if enc in ("rgb8", "bgr8"):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if enc == "rgb8" else arr.copy()
    if enc == "mono8":
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width).copy()
    if enc == "16UC1":
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).copy()
    if enc == "32FC1":
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
    raise ValueError(f"Unsupported encoding: {enc}")


# ---------------- small math helpers ----------------

def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_mat(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


# ---------------- HSV bands (OpenCV H in 0..180) ----------------
# Starting points for lab lighting; override via ROS params if needed.
DEFAULT_HSV = {
    "red": [((0, 100, 60), (10, 255, 255)), ((170, 100, 60), (180, 255, 255))],
    "blue": [((95, 100, 50), (130, 255, 255))],
    "yellow": [((18, 100, 90), (35, 255, 255))],
    "green": [((40, 80, 50), (85, 255, 255))],  # available but off by default
}


class ColorCubeDetector(Node):
    def __init__(self):
        super().__init__("color_cube_detector")
        p = self.declare_parameter
        self.color_topic = p("color_topic", "/camera/color/image_raw").value
        self.depth_topic = p("depth_topic", "/camera/depth/image_raw").value
        self.info_topic = p("camera_info_topic", "/camera/color/camera_info").value
        self.output_topic = p("output_topic", "/foundationpose/object_pose").value
        self.colors: List[str] = list(p("colors", ["red", "blue", "yellow"]).value)
        self.cube_size = float(p("cube_size", 0.05).value)          # metres
        self.min_area = int(p("min_area", 300).value)               # px floor
        self.area_tol = float(p("area_tolerance", 3.0).value)       # x expected
        self.depth_scale = float(p("depth_scale", 0.0).value)       # 0 = auto
        self.depth_min = float(p("depth_min", 0.5).value)           # metres
        self.depth_max = float(p("depth_max", 1.2).value)
        self.rate = float(p("publish_rate", 5.0).value)
        self.center_offset = bool(p("adjust_to_center", True).value)
        # workspace filter (table frame, from calibrated TF)
        self.ws_filter = bool(p("workspace_filter", True).value)
        self.table_frame = p("table_frame", "table").value
        self.ws_x = (float(p("table_x_min", 0.0).value),
                     float(p("table_x_max", 1.18).value))
        self.ws_y = (float(p("table_y_min", -0.80).value),
                     float(p("table_y_max", 0.80).value))
        self.ws_z = (float(p("table_z_min", -0.05).value),
                     float(p("table_z_max", 0.25).value))
        self.ws_margin = float(p("table_margin", 0.02).value)

        self._color: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._K: Optional[np.ndarray] = None
        self._frame_id: str = "camera_color_optical_frame"

        self._tf_buf = None
        if self.ws_filter and HAVE_TF:
            self._tf_buf = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        elif self.ws_filter:
            self.get_logger().warn("tf2_ros unavailable; workspace filter OFF.")
            self.ws_filter = False

        self.create_subscription(Image, self.color_topic, self._on_color, 5)
        self.create_subscription(Image, self.depth_topic, self._on_depth, 5)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, 5)

        self._pub_all = self.create_publisher(PoseStamped, self.output_topic, 10)
        base = self.output_topic.rstrip("/")
        self._pub_color: Dict[str, object] = {}
        self._pub_table: Dict[str, object] = {}
        for c in self.colors:
            self._pub_color[c] = self.create_publisher(PoseStamped, f"{base}/{c}", 10)
            self._pub_table[c] = self.create_publisher(
                PoseStamped, f"{base}/{c}/table", 10)

        self.create_timer(1.0 / self.rate, self._process)
        self.get_logger().info(
            f"color_cube_detector v2: {self.color_topic} + {self.depth_topic} -> "
            f"{self.output_topic}[/<color>[/table]]; colors={self.colors}, "
            f"cube={self.cube_size} m, workspace_filter={self.ws_filter}")

    # -------- callbacks --------
    def _on_color(self, msg):
        try:
            self._color = image_msg_to_numpy(msg)
            self._frame_id = msg.header.frame_id or self._frame_id
        except Exception as e:
            self.get_logger().error(f"color convert failed: {e}")

    def _on_depth(self, msg):
        try:
            d = image_msg_to_numpy(msg)
            if self.depth_scale > 0:
                d = d.astype(np.float32) * self.depth_scale
            elif d.dtype == np.uint16:
                d = d.astype(np.float32) / 1000.0   # mm -> m
            self._depth = d
        except Exception as e:
            self.get_logger().error(f"depth convert failed: {e}")

    def _on_info(self, msg):
        if self._K is None:
            self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(
                f"intrinsics: fx={self._K[0,0]:.1f} fy={self._K[1,1]:.1f}")

    # -------- helpers --------
    def _median_depth(self, u: int, v: int, half: int = 3) -> Optional[float]:
        d = self._depth
        v0, v1 = max(0, v - half), min(d.shape[0], v + half + 1)
        u0, u1 = max(0, u - half), min(d.shape[1], u + half + 1)
        patch = d[v0:v1, u0:u1].astype(np.float32)
        vals = patch[np.isfinite(patch) & (patch > 0.05)]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    def _table_pose(self, pos, quat, stamp) -> Optional[PoseStamped]:
        """Transform an optical-frame pose into the table frame via live TF."""
        try:
            tfm = self._tf_buf.lookup_transform(
                self.table_frame, self._frame_id, Time())
        except Exception:
            return None
        t = tfm.transform.translation
        q = tfm.transform.rotation
        R = quat_to_mat(q.x, q.y, q.z, q.w)
        p = R @ np.array(pos) + np.array([t.x, t.y, t.z])
        qo = quat_mul((q.x, q.y, q.z, q.w), quat)
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.table_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = p.tolist()
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = qo
        return out

    # -------- main loop --------
    def _process(self):
        if self._color is None or self._depth is None or self._K is None:
            return
        bgr, K = self._color, self._K
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        stamp = self.get_clock().now().to_msg()

        for color in self.colors:
            bands = DEFAULT_HSV.get(color)
            if not bands:
                continue
            mask = None
            for lo, hi in bands:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best = None
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                u, v = M["m10"] / M["m00"], M["m01"] / M["m00"]
                z_top = self._median_depth(int(u), int(v))
                if z_top is None:
                    continue
                if not (self.depth_min <= z_top <= self.depth_max):
                    continue
                # cube-size plausibility: expected pixel area at this depth
                exp_side = fx * self.cube_size / z_top
                exp_area = exp_side * exp_side
                if not (exp_area / self.area_tol <= area
                        <= exp_area * self.area_tol):
                    continue
                if best is None or area > best[0]:
                    best = (area, u, v, z_top, cnt)
            if best is None:
                continue
            _, u, v, z_top, cnt = best
            z = z_top + (self.cube_size / 2.0 if self.center_offset else 0.0)
            pos = ((u - cx) * z / fx, (v - cy) * z / fy, z)
            rect = cv2.minAreaRect(cnt)
            yaw = math.radians(rect[2])  # approximate, about optical axis
            quat = quat_from_yaw(yaw)

            table_ps = None
            if self.ws_filter:
                table_ps = self._table_pose(pos, quat, stamp)
                if table_ps is not None:
                    tp = table_ps.pose.position
                    m = self.ws_margin
                    if not (self.ws_x[0] - m <= tp.x <= self.ws_x[1] + m and
                            self.ws_y[0] - m <= tp.y <= self.ws_y[1] + m and
                            self.ws_z[0] <= tp.z <= self.ws_z[1]):
                        self.get_logger().info(
                            f"{color}: rejected off-table at "
                            f"({tp.x:.2f},{tp.y:.2f},{tp.z:.2f})",
                            throttle_duration_sec=5.0)
                        continue

            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self._frame_id
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = quat
            self._pub_all.publish(ps)
            self._pub_color[color].publish(ps)
            if table_ps is not None:
                self._pub_table[color].publish(table_ps)


def main():
    rclpy.init()
    node = ColorCubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
