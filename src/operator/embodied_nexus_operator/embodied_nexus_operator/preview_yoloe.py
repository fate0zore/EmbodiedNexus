"""Capture one RGB camera frame and save a YOLOE box-mask preview."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .detector import YOLOEBoxDetector
from .geometry import decode_image


class YOLOEPreview(Node):
    """Receive one public RGB image for detector-only visual inspection."""

    def __init__(self, camera_id: str) -> None:
        """Subscribe to the configured Nexus camera color image topic.

        Args:
            camera_id: Camera instance ID used in the public ROS topic path.

        Returns:
            None.
        """
        super().__init__("embodied_nexus_yoloe_preview")
        self.image: Image | None = None
        topic = f"/embodied_nexus/cameras/{camera_id}/color/image_raw"
        self.subscription = self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, message: Image) -> None:
        """Cache the first image received from the configured camera.

        Args:
            message: Public ROS color image.

        Returns:
            None.
        """
        if self.image is None:
            self.image = message


def _save_preview(image_message: Image, detector: YOLOEBoxDetector,
                  output_path: Path) -> dict:
    """Run segmentation and save a color overlay without reading simulator truth.

    Args:
        image_message: Public ROS color image message.
        detector: Local pretrained YOLOE text-prompt detector.
        output_path: Path where the annotated PNG image will be written.

    Returns:
        Detection confidence, mask pixel count, and output path.
    """
    image = decode_image(image_message)
    if image_message.encoding.lower() == "bgr8":
        bgr = image.copy()
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif image_message.encoding.lower() == "rgb8":
        rgb = image
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    else:
        raise ValueError(f"YOLOE preview requires rgb8 or bgr8, got {image_message.encoding}")
    mask, confidence = detector.segment(rgb)
    overlay = bgr.copy()
    overlay[mask] = (0, 0, 255)
    rendered = cv2.addWeighted(bgr, 0.62, overlay, 0.38, 0.0)
    rows, columns = np.nonzero(mask)
    if len(columns):
        cv2.rectangle(rendered, (int(columns.min()), int(rows.min())),
                      (int(columns.max()), int(rows.max())), (0, 255, 0), 2)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), rendered):
        raise OSError(f"could not write detector preview to {output_path}")
    return {"confidence": float(confidence), "mask_pixels": int(mask.sum()),
            "output": str(output_path)}


def main(args=None) -> int:
    """Capture and save one detector result for visual preflight review.

    Args:
        args: Optional command-line arguments.

    Returns:
        Process exit code, zero when a preview was written.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-id", default="camera_front")
    parser.add_argument("--class-prompt", default="red rectangular box")
    cache = Path("~/.cache/embodied-nexus/models").expanduser()
    parser.add_argument("--weights", type=Path, default=cache / "yoloe-11s-seg.pt")
    parser.add_argument("--embeddings", type=Path,
                        default=cache / "yoloe-red-rectangular-box-text.npz")
    parser.add_argument("--output", type=Path, default=Path("/tmp/yoloe-box-preview.png"))
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parsed, ros_args = parser.parse_known_args(args)
    detector = YOLOEBoxDetector(str(parsed.weights), str(parsed.embeddings),
                                class_prompt=parsed.class_prompt)
    rclpy.init(args=ros_args)
    node = YOLOEPreview(parsed.camera_id)
    deadline = time.monotonic() + float(parsed.timeout_sec)
    try:
        while node.image is None and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.image is None:
            raise TimeoutError("no RGB camera image arrived before the preview deadline")
        result = _save_preview(node.image, detector, parsed.output.expanduser().resolve())
        result["class_prompt"] = parsed.class_prompt
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except Exception as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=False))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

