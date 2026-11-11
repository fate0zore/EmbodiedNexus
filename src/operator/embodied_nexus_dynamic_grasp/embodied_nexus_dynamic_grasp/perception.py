"""YOLOE segmentation and RGB-D projection for dynamic target observations."""

import hashlib
import json
import math
from pathlib import Path

import numpy as np


def select_target(targets, target_class):
    """Resolve a configured target class.

    Args:
        targets: Mapping of class prompts to target settings.
        target_class: Requested prompt.

    Returns:
        The configured target settings.
    """
    if target_class not in targets:
        raise ValueError(f"unsupported target class: {target_class}")
    target = targets[target_class]
    if target.get("shape") not in ("box", "cylinder"):
        raise ValueError("target shape must be box or cylinder")
    return target



def validate_frame_pair(color_stamp, depth_stamp, color_received, depth_received,
                        now, max_delta, max_age):
    """Reject mismatched or stale RGB-D pairs using message and monotonic clocks.

    Args:
        color_stamp: Color timestamp in seconds.
        depth_stamp: Depth timestamp in seconds.
        color_received: Monotonic color receive time.
        depth_received: Monotonic depth receive time.
        now: Current monotonic time.
        max_delta: Maximum timestamp difference in seconds.
        max_age: Maximum receive age in seconds.

    Returns:
        None when the pair is valid.
    """
    values = (color_stamp, depth_stamp, color_received, depth_received, now, max_delta, max_age)
    if not all(math.isfinite(float(value)) for value in values) or max_delta < 0 or max_age <= 0:
        raise ValueError("invalid frame timestamps or limits")
    if abs(color_stamp - depth_stamp) > max_delta:
        raise ValueError("RGB-D timestamps do not match")
    age = now - min(color_received, depth_received)
    if age < 0 or age > max_age:
        raise ValueError("RGB-D pair is stale")



def masked_depth_points(depth, mask, depth_intrinsics, color_intrinsics, color_from_depth,
                        depth_scale=1.0, minimum_points=80, allow_clipped=False):
    """Project depth pixels into the color mask using calibrated camera geometry.

    Args:
        depth: Two-dimensional depth image.
        mask: Two-dimensional segmentation in color image coordinates.
        depth_intrinsics: Depth camera fx, fy, cx and cy.
        color_intrinsics: Color camera fx, fy, cx and cy.
        color_from_depth: Homogeneous transform from depth to color coordinates.
        depth_scale: Meters per depth unit.
        minimum_points: Minimum valid segmented points.
        allow_clipped: Whether an image-edge mask may be used for tracked geometry.

    Returns:
        Nx3 depth-frame points in meters.
    """
    depth = np.asarray(depth)
    mask = np.asarray(mask, dtype=bool)
    intrinsics = np.asarray([depth_intrinsics, color_intrinsics], dtype=float)
    transform = np.asarray(color_from_depth, dtype=float)
    if (depth.ndim != 2 or mask.ndim != 2 or intrinsics.shape != (2, 4)
            or not np.isfinite(intrinsics).all() or np.any(intrinsics[:, :2] <= 0)
            or transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not math.isfinite(float(depth_scale)) or depth_scale <= 0):
        raise ValueError("invalid RGB-D calibration or images")
    if int(mask.sum()) < minimum_points or (not allow_clipped and
            (mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any())):
        raise ValueError("target mask is too small or touches an image edge")
    meters = depth.astype(float) * depth_scale
    rows, cols = np.nonzero(np.isfinite(meters) & (meters >= 0.05) & (meters <= 3.0))
    z = meters[rows, cols]
    dfx, dfy, dcx, dcy = intrinsics[0]
    points = np.column_stack(((cols - dcx) * z / dfx, (rows - dcy) * z / dfy, z))
    color = points @ transform[:3, :3].T + transform[:3, 3]
    front = color[:, 2] > 1e-6
    cfx, cfy, ccx, ccy = intrinsics[1]
    u = np.rint(cfx * color[:, 0] / np.maximum(color[:, 2], 1e-6) + ccx).astype(int)
    v = np.rint(cfy * color[:, 1] / np.maximum(color[:, 2], 1e-6) + ccy).astype(int)
    inside = front & (u >= 0) & (u < mask.shape[1]) & (v >= 0) & (v < mask.shape[0])
    selected = np.zeros(len(points), dtype=bool)
    selected[inside] = mask[v[inside], u[inside]]
    result = points[selected]
    if len(result) < minimum_points:
        raise ValueError("insufficient valid masked depth points")
    return result



class YOLOEDetector:
    """Load a locally prepared YOLOE model for one configured class."""

    def __init__(self, weights, embeddings, prompt, confidence):
        """Validate local model assets and load the segmentation model.

        Args:
            weights: Path to the model checkpoint.
            embeddings: Path to the prompt embedding archive.
            prompt: Class prompt represented by the archive.
            confidence: Minimum detection confidence.

        Returns:
            None.
        """
        weights = Path(weights).expanduser()
        embeddings = Path(embeddings).expanduser()
        manifest = json.loads(embeddings.with_suffix(".manifest.json").read_text(encoding="utf-8"))
        if manifest.get("text_prompt", "").strip().lower() != prompt.strip().lower():
            raise ValueError("model manifest prompt does not match target class")
        for path, key in ((weights, "checkpoint_sha256"), (embeddings, "text_embeddings_sha256")):
            if hashlib.sha256(path.read_bytes()).hexdigest() != manifest.get(key):
                raise ValueError(f"model asset checksum mismatch: {path}")
        with np.load(embeddings, allow_pickle=False) as archive:
            classes = archive["classes"].astype(str).tolist()
            vectors = np.asarray(archive["embeddings"], dtype=np.float32)
        if classes != [prompt] or vectors.ndim != 3 or not np.isfinite(vectors).all():
            raise ValueError("invalid target embedding")
        if not 0.0 < float(confidence) <= 1.0:
            raise ValueError("confidence must be between zero and one")
        import torch
        from ultralytics import YOLOE
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = YOLOE(str(weights))
        self.model.model.to(device)
        self.model.set_classes(classes, torch.as_tensor(vectors, device=device))
        self.confidence = float(confidence)
        self.preferred_angle = 0
        self.retry_index = 0

    def segment(self, rgb):
        """Select the highest confidence segmentation from an RGB frame.

        Args:
            rgb: HxWx3 RGB uint8 image.

        Returns:
            Boolean mask and detection confidence.
        """
        import cv2
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("expected an RGB uint8 image")
        height, width = image.shape[:2]
        side = int(math.ceil(math.hypot(width, height)))
        left, top = (side - width) // 2, (side - height) // 2
        border = tuple(int(x) for x in np.median(image.reshape(-1, 3), axis=0)[::-1])
        canvas = cv2.copyMakeBorder(image[:, :, ::-1].copy(), top, side - height - top,
                                    left, side - width - left, cv2.BORDER_CONSTANT, value=border)
        # A full-turn padded input retains the original orientation while
        # providing context around an image-edge target.
        angles = list(dict.fromkeys([self.preferred_angle, 0, 45, -45, 90, 360]))
        # One inference per camera frame keeps the RGB-D freshness guarantee;
        # failed angles are retried on subsequent frames.
        for angle in [angles[self.retry_index % len(angles)]]:
            matrix = cv2.getRotationMatrix2D((side / 2, side / 2), angle, 1.0)
            context_pad = int(min(width, height) * 0.75)
            if angle == 0:
                rotated = image[:, :, ::-1].copy()
            elif angle == 360:
                rotated = cv2.copyMakeBorder(image[:, :, ::-1].copy(), context_pad, context_pad,
                                             context_pad, context_pad, cv2.BORDER_CONSTANT, value=border)
            else:
                rotated = cv2.warpAffine(canvas, matrix, (side, side), borderValue=border)
            result = self.model.predict(rotated, verbose=False, conf=self.confidence, max_det=20)[0]
            if result.boxes is None or result.masks is None or not len(result.boxes):
                continue
            confidences = result.boxes.conf.detach().cpu().numpy()
            index = int(np.argmax(confidences))
            mask = result.masks.data[index].detach().cpu().numpy()
            target_size = (rotated.shape[1], rotated.shape[0])
            if mask.shape != target_size[::-1]:
                mask = cv2.resize(mask.astype(np.float32), target_size, interpolation=cv2.INTER_NEAREST)
            if angle == 360:
                mask = mask[context_pad:context_pad + height, context_pad:context_pad + width]
            elif angle != 0:
                mask = cv2.warpAffine(mask.astype(np.float32), cv2.invertAffineTransform(matrix),
                                      (side, side), flags=cv2.INTER_NEAREST)
                mask = mask[top:top + height, left:left + width]
            mask = mask > 0.5
            if mask.any():
                self.preferred_angle = angle
                self.retry_index = 0
                return mask, float(confidences[index])
        self.retry_index = (self.retry_index + 1) % len(angles)
        raise RuntimeError("target not detected")
