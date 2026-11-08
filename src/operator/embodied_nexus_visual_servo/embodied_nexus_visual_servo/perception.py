"""YOLOE segmentation and geometry for independent visual servoing."""

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


def estimate_clipped_geometry(points, ground_height, previous, minimum_points=80):
    """Track a partially visible ground target from its previously observed end.

    Args:
        points: Current segmented base-frame points.
        ground_height: Ground height in the base frame.
        previous: Last complete or tracked estimate with fixed size and yaw.
        minimum_points: Minimum required inlier points.

    Returns:
        Updated estimate using the visible endpoint and previous object length.
    """
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    values = values[np.isfinite(values).all(axis=1)]
    values = values[(values[:, 2] > ground_height + 0.002) & (values[:, 2] < ground_height + 0.5)]
    if len(values) < minimum_points:
        raise ValueError("insufficient clipped target points")
    yaw = float(previous["yaw"])
    axis = np.array([math.cos(yaw), math.sin(yaw)])
    side = np.array([-axis[1], axis[0]])
    longitudinal, lateral = values[:, :2] @ axis, values[:, :2] @ side
    lo, hi = np.percentile(longitudinal, [2, 98])
    side_lo, side_hi = np.percentile(lateral, [2, 98])
    old = np.asarray(previous["center"][:2], dtype=float)
    length = float(previous["size"][0])
    old_along = float(old @ axis)
    candidates = [(abs(lo - (old_along - length / 2)), lo + length / 2),
                  (abs(hi - (old_along + length / 2)), hi - length / 2)]
    error, along = min(candidates)
    if error > 0.02:
        raise ValueError("neither clipped target endpoint matches the tracked target")
    sideways = (side_lo + side_hi) / 2
    if abs(sideways - float(old @ side)) > 0.02:
        raise ValueError("clipped target lateral position changed unexpectedly")
    height = float(np.percentile(values[:, 2], 98) - ground_height)
    if abs(height - float(previous["size"][2])) > 0.02:
        raise ValueError("clipped target height changed unexpectedly")
    xy = axis * along + side * sideways
    # A clipped view cannot reliably determine both extents; bound its correction
    # to the last complete observation instead of accumulating per-frame drift.
    xy = old + np.clip(xy - old, -0.005, 0.005)
    center_z = float(previous["center"][2]) + float(np.clip(
        ground_height + height / 2 - float(previous["center"][2]), -0.005, 0.005))
    return {"center": [float(xy[0]), float(xy[1]), center_z],
            "yaw": yaw, "size": list(previous["size"]), "point_count": len(values)}


def estimate_geometry(points, ground_height, shape, minimum_points=80):
    """Estimate ground-supported box or horizontal cylinder geometry from points.

    Args:
        points: Segmented Nx3 points in the base frame, in meters.
        ground_height: Ground height in the base frame, in meters.
        shape: Either box or cylinder.
        minimum_points: Minimum count of valid points.

    Returns:
        Mapping containing center, yaw, dimensions, and point count.
    """
    if shape not in ("box", "cylinder") or not math.isfinite(float(ground_height)):
        raise ValueError("invalid target shape or ground height")
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    values = values[np.isfinite(values).all(axis=1)]
    values = values[(values[:, 2] > ground_height + 0.002) & (values[:, 2] < ground_height + 0.5)]
    if len(values) < minimum_points:
        raise ValueError("insufficient target depth points")
    distance = np.linalg.norm(values[:, :2] - np.median(values[:, :2], axis=0), axis=1)
    values = values[distance <= np.percentile(distance, 97)]
    if len(values) < minimum_points:
        raise ValueError("insufficient inlier depth points")
    _, eigenvectors = np.linalg.eigh(np.cov(values[:, :2], rowvar=False))
    axis = eigenvectors[:, -1]
    side = np.array([-axis[1], axis[0]])
    projected = np.column_stack((values[:, :2] @ axis, values[:, :2] @ side))
    lo, hi = np.percentile(projected, [2, 98], axis=0)
    height = float(np.percentile(values[:, 2], 98) - ground_height)
    size = [float(hi[0] - lo[0]), float(hi[1] - lo[1]), height]
    if shape == "cylinder":
        size[1] = height
    if min(size) < 0.005 or max(size) > 0.5:
        raise ValueError("measured target dimensions are outside supported bounds")
    center = axis * ((lo[0] + hi[0]) / 2) + side * ((lo[1] + hi[1]) / 2)
    return {"center": [float(center[0]), float(center[1]), ground_height + height / 2],
            "yaw": math.atan2(float(axis[1]), float(axis[0])), "size": size,
            "point_count": len(values)}


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
        result = self.model.predict(image[:, :, ::-1].copy(), verbose=False,
                                    conf=self.confidence, max_det=20)[0]
        padding = 0
        if result.boxes is None or result.masks is None or not len(result.boxes):
            # Give a clipped near-field object more image context before retrying YOLOE.
            padding = max(1, min(image.shape[:2]) // 3)
            border = tuple(int(x) for x in np.median(image.reshape(-1, 3), axis=0)[::-1])
            padded = cv2.copyMakeBorder(image[:, :, ::-1].copy(), padding, padding, padding, padding,
                                        cv2.BORDER_CONSTANT, value=border)
            result = self.model.predict(padded, verbose=False, conf=self.confidence, max_det=20)[0]
            if result.boxes is None or result.masks is None or not len(result.boxes):
                raise RuntimeError("target not detected")
        confidences = result.boxes.conf.detach().cpu().numpy()
        index = int(np.argmax(confidences))
        mask = result.masks.data[index].detach().cpu().numpy()
        output_size = (image.shape[1] + 2 * padding, image.shape[0] + 2 * padding)
        if mask.shape != output_size[::-1]:
            mask = cv2.resize(mask.astype(np.float32), output_size,
                              interpolation=cv2.INTER_NEAREST)
        if padding:
            mask = mask[padding:padding + image.shape[0], padding:padding + image.shape[1]]
        mask = mask > 0.5
        return mask, float(confidences[index])
