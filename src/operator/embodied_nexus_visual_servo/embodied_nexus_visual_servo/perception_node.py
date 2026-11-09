"""Publish independent YOLOE RGB-D target estimates for visual servo development."""

import math
import json
import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from embodied_nexus_visual_servo_interfaces.msg import VisualTarget

from .perception import (GeometryTracker, YOLOEDetector, estimate_clipped_geometry, estimate_geometry,
                         incomplete_box_extent, masked_depth_points, select_target, validate_frame_pair)


def image_array(message):
    """Decode supported ROS image encodings, including row padding and byte order.

    Args:
        message: ROS sensor image message.

    Returns:
        RGB uint8 or depth array, depending on the input encoding.
    """
    encoding = message.encoding.lower()
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    if encoding in ("rgb8", "bgr8"):
        image = rows[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        return image if encoding == "rgb8" else image[:, :, ::-1].copy()
    if encoding in ("16uc1", "mono16", "32fc1"):
        width = 4 if encoding == "32fc1" else 2
        dtype = (">" if message.is_bigendian else "<") + ("f4" if width == 4 else "u2")
        return rows[:, :message.width * width].copy().view(dtype).reshape(message.height, message.width)
    raise ValueError(f"unsupported image encoding: {message.encoding}")


def transform_matrix(message):
    """Convert a TransformStamped to a homogeneous matrix.

    Args:
        message: Transform from source coordinates to destination coordinates.

    Returns:
        Four-by-four homogeneous transform matrix.
    """
    q = message.transform.rotation
    quat = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid TF quaternion")
    x, y, z, w = quat / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    translation = message.transform.translation
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    if not np.isfinite(matrix).all():
        raise ValueError("invalid TF translation")
    return matrix


class PerceptionNode(Node):
    """Continuously publish vision-only target pose estimates."""

    def __init__(self):
        """Configure subscriptions, target detector and stamped TF lookup.

        Returns:
            None.
        """
        super().__init__("visual_servo_perception")
        self.declare_parameter("config_file", "")
        self.declare_parameter("target_class", "green cylinder")
        path = self.get_parameter("config_file").value or str(
            Path(get_package_share_directory("embodied_nexus_visual_servo")) / "config/visual_servo.yaml")
        self.config = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))["visual_servo"]
        prompt = self.get_parameter("target_class").value
        self.prompt = prompt
        self.target = select_target(self.config["targets"], prompt)
        self.detector = YOLOEDetector(self.config["weights"], self.target["embeddings"],
                                      prompt, self.config["confidence"])
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.processing = threading.Lock()
        self.frames = {}
        self.images = {"color": deque(maxlen=20), "depth": deque(maxlen=20)}
        self.last_depth_stamp = None
        self.previous_estimate = None
        self.reliable_box_extent = None
        self.tracker = GeometryTracker()
        self.debug_directory = os.environ.get('ENX_VISUAL_SERVO_DEBUG_DIR', '')
        self.debug_last = 0.0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        root = f"/embodied_nexus/cameras/{self.config['camera_id']}"
        for key, kind, suffix in (("color", Image, "color/image_raw"),
                                  ("depth", Image, "depth/image_raw"),
                                  ("color_info", CameraInfo, "color/camera_info"),
                                  ("depth_info", CameraInfo, "depth/camera_info")):
            self.create_subscription(kind, f"{root}/{suffix}", self._callback(key),
                                     qos_profile_sensor_data, callback_group=self.group)
        self.output = self.create_publisher(PoseStamped, "/embodied_nexus/visual_servo/target_pose", 10)
        self.target_output = self.create_publisher(VisualTarget, "/embodied_nexus/visual_servo/target", 10)
        self.create_service(Trigger, "/embodied_nexus/visual_servo/reset_tracker", self._reset_tracker,
                            callback_group=self.group)
        self.create_timer(0.1, self._process, callback_group=self.group)

    def _reset_tracker(self, request, response):
        """Forget the previous object's geometry before a new stationary observation.

        Args:
            request: Empty trigger request.
            response: Trigger response.

        Returns:
            Successful trigger response.
        """
        del request
        with self.processing:
            self.previous_estimate = None
            self.reliable_box_extent = None
            self.tracker = GeometryTracker()
            self.last_depth_stamp = None
            with self.lock:
                self.images["color"].clear()
                self.images["depth"].clear()
        response.success = True
        response.message = "vision tracker reset"
        return response

    def _callback(self, key):
        """Create a message callback for a particular camera stream.

        Args:
            key: Camera stream key.

        Returns:
            Callback that stores the latest message and receive time.
        """
        def receive(message):
            """Cache a camera message and its monotonic receive time.

            Args:
                message: ROS camera message.

            Returns:
                None.
            """
            with self.lock:
                item = (message, time.monotonic())
                if key in self.images:
                    self.images[key].append(item)
                else:
                    self.frames[key] = item
        return receive

    def _process(self):
        """Process a fresh synchronized pair and publish the estimated target pose.

        Returns:
            None.
        """
        if not self.processing.acquire(blocking=False):
            return
        rgb = mask = None
        confidence = None
        estimate = None
        clipped = None
        started = time.monotonic()
        failure = None
        try:
            with self.lock:
                frames = dict(self.frames)
                colors, depths = list(self.images["color"]), list(self.images["depth"])
            if not colors or not depths or any(key not in frames for key in ("color_info", "depth_info")):
                return
            pairs = [(color, depth) for color in colors for depth in depths
                     if abs(Time.from_msg(color[0].header.stamp).nanoseconds -
                            Time.from_msg(depth[0].header.stamp).nanoseconds) / 1e9
                     <= self.config["maximum_rgb_depth_delta_sec"]]
            if not pairs:
                return
            (color, color_received), (depth, depth_received) = max(
                pairs, key=lambda pair: min(pair[0][1], pair[1][1]))
            color_stamp = Time.from_msg(color.header.stamp)
            depth_stamp = Time.from_msg(depth.header.stamp)
            if self.last_depth_stamp == depth_stamp.nanoseconds:
                return
            self.last_depth_stamp = depth_stamp.nanoseconds
            validate_frame_pair(color_stamp.nanoseconds / 1e9, depth_stamp.nanoseconds / 1e9,
                                color_received, depth_received, time.monotonic(),
                                self.config["maximum_rgb_depth_delta_sec"], self.config["maximum_frame_age_sec"])
            rgb, depth_values = image_array(color), image_array(depth)
            mask, confidence = self.detector.segment(rgb)
            clipped = bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any())
            if clipped and self.previous_estimate is None:
                raise ValueError("clipped target has no prior complete observation")
            depth_info, color_info = frames["depth_info"][0], frames["color_info"][0]
            intrinsics = lambda info: [info.k[i] for i in (0, 4, 2, 5)]
            color_tf = self.tf.lookup_transform(color.header.frame_id, depth.header.frame_id,
                                                depth_stamp, timeout=Duration(seconds=0.4))
            base_tf = self.tf.lookup_transform(self.config["base_frame"], depth.header.frame_id,
                                               depth_stamp, timeout=Duration(seconds=0.4))
            points = masked_depth_points(depth_values, mask, intrinsics(depth_info), intrinsics(color_info),
                                         transform_matrix(color_tf),
                                         0.001 if depth.encoding.lower() in ("16uc1", "mono16") else 1.0,
                                         self.config["minimum_mask_points"], allow_clipped=clipped)
            matrix = transform_matrix(base_tf)
            points = points @ matrix[:3, :3].T + matrix[:3, 3]
            if not clipped:
                estimate = estimate_geometry(points, self.config["ground_height_base_m"],
                                             self.target["shape"], self.config["minimum_mask_points"])
                if self.previous_estimate is not None:
                    old = self.previous_estimate
                    clipped = (estimate["size"][0] < 0.7 * old["size"][0] or
                               (self.target['shape'] == 'box' and self.reliable_box_extent is not None and
                                incomplete_box_extent(estimate, self.reliable_box_extent)) or
                               abs((estimate["yaw"] - old["yaw"] + math.pi / 2) % math.pi - math.pi / 2)
                               > math.radians(10.0) or
                               np.linalg.norm(np.asarray(estimate["center"]) - old["center"]) > 0.015)
            if clipped:
                estimate = estimate_clipped_geometry(points, self.config["ground_height_base_m"],
                                                     self.previous_estimate, self.config["minimum_mask_points"])
            if time.monotonic() - min(color_received, depth_received) > self.config["maximum_frame_age_sec"]:
                raise ValueError("inference completed after the frame age limit")
            estimate = self.tracker.update(estimate, clipped)
            if estimate is None:
                return
            if not clipped:
                self.previous_estimate = estimate
                if self.target['shape'] == 'box':
                    self.reliable_box_extent = max(self.reliable_box_extent or 0.0, estimate['size'][0])
            result = PoseStamped()
            result.header.frame_id = self.config["base_frame"]
            result.header.stamp = depth.header.stamp
            result.pose.position.x, result.pose.position.y, result.pose.position.z = estimate["center"]
            result.pose.orientation.z = math.sin(estimate["yaw"] / 2)
            result.pose.orientation.w = math.cos(estimate["yaw"] / 2)
            self.output.publish(result)
            target = VisualTarget()
            target.pose = result
            target.size_m = [float(value) for value in estimate["size"]]
            target.confidence = float(confidence)
            target.target_class = self.prompt
            target.observation_complete = not clipped
            target.geometry_ready = True
            self.target_output.publish(target)
            self.get_logger().debug(f"target confidence={confidence:.3f} size={estimate['size']}")
        except Exception as exc:
            failure = str(exc)
            self.tracker.reject()
            self.get_logger().warning(f"visual target unavailable: {exc}", throttle_duration_sec=2.0)
        finally:
            if self.debug_directory and rgb is not None and time.monotonic() - self.debug_last >= 2.0:
                try:
                    import cv2
                    directory = Path(self.debug_directory)
                    directory.mkdir(parents=True, exist_ok=True)
                    stamp = str(time.time_ns())
                    cv2.imwrite(str(directory / f'{stamp}-rgb.png'), rgb[:, :, ::-1])
                    if mask is not None:
                        cv2.imwrite(str(directory / f'{stamp}-mask.png'), mask.astype(np.uint8) * 255)
                    with (directory / 'observations.jsonl').open('a', encoding='utf-8') as stream:
                        stream.write(json.dumps({'stamp': stamp, 'confidence': confidence, 'failure': failure,
                                                 'inference_and_geometry_sec': time.monotonic() - started,
                                                 'angle': self.detector.preferred_angle,
                                                 'clipped': None if clipped is None else bool(clipped),
                                                 'estimate': estimate}) + '\n')
                    self.debug_last = time.monotonic()
                except Exception as exc:
                    self.get_logger().warning(f'visual diagnostic recording failed: {exc}', throttle_duration_sec=10.0)
            self.processing.release()


def main(args=None):
    """Run the perception node with concurrent camera and TF callbacks.

    Args:
        args: Optional ROS command-line arguments.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
