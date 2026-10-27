from __future__ import annotations

import json
import re
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np


@dataclass
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: list[float]
    distortion_model: str = "plumb_bob"
    calibrated: bool = False

    @property
    def cv_distortion(self) -> np.ndarray:
        """返回 OpenCV 格式的相机畸变系数。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 np.ndarray。
        """
        values = self.distortion
        if self.distortion_model in {"plumb_bob", "brown_conrady"}:
            if len(values) not in (4, 5):
                raise ValueError(f"{self.distortion_model} requires 4 or 5 coefficients")
            # OpenCV order: k1, k2, p1, p2, k3.
            values = values[:4] + ([values[4]] if len(values) == 5 else [])
        elif self.distortion_model == "rational_polynomial":
            if len(values) != 8:
                raise ValueError("rational_polynomial requires 8 OpenCV ordered coefficients")
        else:
            raise ValueError(f"unsupported distortion model: {self.distortion_model}")
        return np.asarray(values, dtype=np.float64)

    @property
    def matrix(self) -> np.ndarray:
        """返回相机内参矩阵。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 np.ndarray。
        """
        return np.asarray(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def scaled(self, width: int, height: int) -> "CameraModel":
        """按目标分辨率缩放相机标定参数。

        Args:
            width: width，作为执行 scaled 对应的处理流程所需的输入。 类型为 int。
            height: height，作为执行 scaled 对应的处理流程所需的输入。 类型为 int。

        Returns:
            本次处理产生的结果值。 返回类型为 'CameraModel'。
        """
        if width == self.width and height == self.height:
            return self
        if width <= 0 or height <= 0 or abs(width / height - self.width / self.height) > 1e-6:
            raise ValueError("intrinsics resolution mismatch; only equal-aspect scaling is supported")
        sx, sy = width / self.width, height / self.height
        return CameraModel(
            width=width,
            height=height,
            fx=self.fx * sx,
            fy=self.fy * sy,
            cx=self.cx * sx,
            cy=self.cy * sy,
            distortion=list(self.distortion),
            distortion_model=self.distortion_model,
            calibrated=self.calibrated,
        )


@dataclass
class RgbdCalibration:
    color: CameraModel | None
    depth: CameraModel | None
    depth_scale_m: float
    depth_to_color_rotation: np.ndarray
    depth_to_color_translation_m: np.ndarray
    version: str = "uncalibrated"
    source: str = "default"
    issue: str = ""

    @property
    def usable_for_color_cloud(self) -> bool:
        """检查当前标定是否适用于彩色点云生成。

        Args:
            无参数。

        Returns:
            布尔值，表示执行 usable_for_color_cloud 对应的处理流程是否成功或成立。
        """
        return bool(
            self.color
            and self.depth
            and self.color.calibrated
            and self.depth.calibrated
            and self.depth_scale_m > 0
        )


def _model_from_entry(entry: dict[str, Any]) -> CameraModel:
    """从配置条目创建相机标定模型。

    Args:
        entry: 条目，作为执行 _model_from_entry 对应的处理流程所需的输入。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 CameraModel。
    """
    size = entry.get("image_size") or entry.get("resolution")
    intrinsic = entry.get("intrinsic") or entry
    distortion = entry.get("distortion") or entry.get("distortion_coefficients") or []
    model_name = str(entry.get("distortion_model", "plumb_bob")).lower()
    if not isinstance(size, (list, tuple)) or len(size) != 2:
        raise ValueError("calibration stream must declare image_size [width, height]")
    if isinstance(distortion, dict):
        order = ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")
        distortion = [float(distortion[k]) for k in order if k in distortion]
    # Arachne's factory exporter stores OBCameraDistortion in native k1..k6,p1,p2 order.
    if len(distortion) >= 8 and model_name == "modified_brown_conrady":
        distortion = [distortion[0], distortion[1], distortion[6], distortion[7], *distortion[2:6]]
        model_name = "rational_polynomial"
    elif model_name == "modified_brown_conrady":
        raise ValueError("modified_brown_conrady calibration needs all 8 native coefficients")
    return CameraModel(
        width=int(size[0]),
        height=int(size[1]),
        fx=float(intrinsic["fx"]),
        fy=float(intrinsic["fy"]),
        cx=float(intrinsic["cx"]),
        cy=float(intrinsic["cy"]),
        distortion=[float(v) for v in distortion],
        distortion_model=model_name,
        calibrated=True,
    )


def load_calibration(
    *, factory_file: str = "", custom_file: str = "",
    color_size: tuple[int, int] = (640, 480), depth_size: tuple[int, int] = (640, 480),
    depth_scale_m: float = 0.001, extrinsic_unit: str = "mm",
    allow_resolution_scaling: bool = False,
) -> RgbdCalibration:
    """加载calibration。

    Args:
        factory_file: factory文件，作为加载calibration所需的输入。 类型为 str。
        custom_file: custom文件，作为加载calibration所需的输入。 类型为 str。
        color_size: colorsize，作为加载calibration所需的输入。 类型为 tuple[int, int]。
        depth_size: depthsize，作为加载calibration所需的输入。 类型为 tuple[int, int]。
        depth_scale_m: depthscalem，作为加载calibration所需的输入。 类型为 float。
        extrinsic_unit: extrinsicunit，作为加载calibration所需的输入。 类型为 str。
        allow_resolution_scaling: allowresolutionscaling，作为加载calibration所需的输入。 类型为 bool。

    Returns:
        查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 RgbdCalibration。
    """
    path = custom_file or factory_file
    data: dict[str, Any] = {}
    source = "default"
    issues: list[str] = []
    if path:
        data = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("calibration root must be an object")
        source = str(Path(path).expanduser())
    streams = data.get("streams", data)
    color = _model_from_entry(streams["color"]) if "color" in streams else None
    depth = _model_from_entry(streams["depth"]) if "depth" in streams else None

    def match_resolution(model: CameraModel | None, name: str, size: tuple[int, int]) -> CameraModel | None:
        """查找分辨率与输入图像匹配的相机模型。

        Args:
            model: model，作为执行 match_resolution 对应的处理流程所需的输入。 类型为 CameraModel | None。
            name: 名称，作为执行 match_resolution 对应的处理流程所需的输入。 类型为 str。
            size: size，作为执行 match_resolution 对应的处理流程所需的输入。 类型为 tuple[int, int]。

        Returns:
            本次处理产生的结果值。 返回类型为 CameraModel | None。
        """
        if model is None or (model.width, model.height) == size:
            return model
        if not allow_resolution_scaling:
            issues.append(
                f"{name} calibration is {model.width}x{model.height} but the stream is {size[0]}x{size[1]}; "
                "geometry is disabled unless calibration.allow_resolution_scaling is true"
            )
            return None
        try:
            return model.scaled(*size)
        except ValueError as exc:
            issues.append(f"{name} calibration cannot be scaled to {size[0]}x{size[1]}: {exc}")
            return None

    color = match_resolution(color, "RGB", color_size)
    depth = match_resolution(depth, "depth", depth_size)
    transform = data.get("transform_depth_to_color") or data.get("depth_to_color") or {}
    rotation = np.asarray(transform.get("rot", np.eye(3).reshape(-1)), dtype=np.float64).reshape(3, 3)
    translation = np.asarray(transform.get("trans", [0.0, 0.0, 0.0]), dtype=np.float64).reshape(3)
    if extrinsic_unit == "mm":
        translation *= 0.001
    elif extrinsic_unit != "m":
        raise ValueError("depth_to_color_translation_unit must be mm or m")
    if depth_scale_m <= 0:
        raise ValueError("depth_scale_m must be positive")
    version = str(data.get("calibration_id") or data.get("exported_at") or "imported")
    return RgbdCalibration(color, depth, float(depth_scale_m), rotation, translation, version, source, "; ".join(issues))


def camera_info_dict(model: CameraModel, frame_id: str, stamp_sec: float) -> dict[str, Any]:
    """相机信息dict。

    Args:
        model: model，作为相机信息dict所需的输入。 类型为 CameraModel。
        frame_id: 坐标系标识符，作为相机信息dict所需的输入。 类型为 str。
        stamp_sec: 时间戳sec，作为相机信息dict所需的输入。 类型为 float。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    d = model.cv_distortion.tolist()
    return {
        "width": model.width,
        "height": model.height,
        "distortion_model": model.distortion_model,
        "d": d,
        "k": model.matrix.reshape(-1).tolist(),
        "r": np.eye(3).reshape(-1).tolist(),
        "p": [model.fx, 0.0, model.cx, 0.0, 0.0, model.fy, model.cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        "frame_id": frame_id,
        "stamp_sec": stamp_sec,
        "calibrated": model.calibrated,
    }


def capture_intrinsic_sample(image: np.ndarray, session_dir: Path, metadata: dict[str, Any]) -> dict[str, Any]:
    """采集intrinsic采集样本。

    Args:
        image: 图像，作为采集intrinsic采集样本所需的输入。 类型为 np.ndarray。
        session_dir: 会话dir，作为采集intrinsic采集样本所需的输入。 类型为 Path。
        metadata: metadata，作为采集intrinsic采集样本所需的输入。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    session_dir.mkdir(parents=True, exist_ok=True)
    indices = [int(match.group(1)) for path in session_dir.glob("frame_*.png") if (match := re.fullmatch(r"frame_(\d+)\.png", path.name))]
    index = max(indices, default=0) + 1
    name = f"frame_{index:04d}.png"
    if not cv2.imwrite(str(session_dir / name), image):
        raise RuntimeError("failed to write calibration image")
    manifest_path = session_dir / "capture_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else []
    row = {"file": name, "captured_at": time.time(), **metadata}
    manifest.append(row)
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return row


def inspect_apriltag_sample(image: np.ndarray, board_image_path: str) -> dict[str, Any]:
    """检测采集图像中的 AprilTag 标定板并提取样本信息。

    Args:
        image: 图像，作为执行 inspect_apriltag_sample 对应的处理流程所需的输入。 类型为 np.ndarray。
        board_image_path: board图像路径，作为执行 inspect_apriltag_sample 对应的处理流程所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    try:
        from pupil_apriltags import Detector
    except ImportError as exc:
        raise RuntimeError("pupil_apriltags is required for RGB calibration capture") from exc
    board = cv2.imread(str(Path(board_image_path).expanduser()), cv2.IMREAD_GRAYSCALE)
    if board is None:
        raise FileNotFoundError(f"cannot read board image: {board_image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    detector = Detector(families="tag36h11", nthreads=2, quad_decimate=1.0, refine_edges=True)
    board_ids = {int(item.tag_id) for item in detector.detect(board, estimate_tag_pose=False)}
    detections = detector.detect(gray, estimate_tag_pose=False)
    matched = sorted({int(item.tag_id) for item in detections} & board_ids)
    corners = 4 * len(matched)
    return {
        "usable": corners >= 4,
        "board_tag_count": len(board_ids),
        "detected_tag_ids": sorted(int(item.tag_id) for item in detections),
        "matched_tag_ids": matched,
        "corner_count": corners,
        "reason": "" if corners >= 4 else "no matching AprilTag corners were detected",
    }


def solve_rgb_intrinsics(
    session_dir: Path, *, board_image_path: str, board_width_m: float,
    board_height_m: float, min_frames: int = 10, max_rms_px: float = 0.5,
    max_outlier_px: float = 2.0, progress_callback=None,
    cancel_event: threading.Event | None = None,
) -> dict[str, Any]:
    """Solve an RGB camera model from captured AprilTag board images.

    Args:
        session_dir: 会话dir，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。 类型为 Path。
        board_image_path: board图像路径，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。 类型为 str。
        board_width_m: boardwidthm，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。 类型为 float。
        board_height_m: boardheightm，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。 类型为 float。
        min_frames: minframes，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。 类型为 int。
        max_rms_px: maxrmspx，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。 类型为 float。
        max_outlier_px: maxoutlierpx，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。 类型为 float。
        progress_callback: progresscallback，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。
        cancel_event: 取消事件，作为根据采集到的 AprilTag 标定板图像求解 RGB 相机内参所需的输入。 类型为 threading.Event | None。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    try:
        from pupil_apriltags import Detector
    except ImportError as exc:
        raise RuntimeError("pupil_apriltags is required for RGB calibration") from exc
    if board_width_m <= 0 or board_height_m <= 0:
        raise ValueError("board dimensions must be positive")
    manifest_path = session_dir / "capture_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError("calibration session contains no capture manifest")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    detector = Detector(families="tag36h11", nthreads=2, quad_decimate=1.0, refine_edges=True)
    board = cv2.imread(board_image_path, cv2.IMREAD_GRAYSCALE)
    if board is None:
        raise FileNotFoundError(f"cannot read board image: {board_image_path}")
    bh, bw = board.shape
    board_detections = detector.detect(board, estimate_tag_pose=False)
    board_corners: dict[int, np.ndarray] = {}
    for detection in board_detections:
        corners = np.asarray(detection.corners, dtype=np.float64)
        physical = np.zeros((4, 3), dtype=np.float64)
        physical[:, 0] = (corners[:, 0] - bw / 2.0) * board_width_m / bw
        physical[:, 1] = (bh / 2.0 - corners[:, 1]) * board_height_m / bh
        board_corners[int(detection.tag_id)] = physical
    if not board_corners:
        raise RuntimeError("no AprilTags detected in board reference image")

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    per_frame: list[dict[str, Any]] = []
    image_size: tuple[int, int] | None = None
    total_frames = len(manifest)
    for frame_index, row in enumerate(manifest, 1):
        if cancel_event is not None and cancel_event.is_set():
            raise InterruptedError("RGB calibration solve cancelled")
        path = session_dir / row["file"]
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            per_frame.append({"file": row.get("file"), "used": False, "reason": "unreadable"})
            if progress_callback:
                progress_callback(frame_index, total_frames)
            continue
        size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            per_frame.append({"file": row.get("file"), "used": False, "reason": "resolution mismatch"})
            if progress_callback:
                progress_callback(frame_index, total_frames)
            continue
        detections = detector.detect(image, estimate_tag_pose=False)
        obj, img = [], []
        for detection in detections:
            physical = board_corners.get(int(detection.tag_id))
            if physical is not None:
                obj.extend(physical)
                img.extend(np.asarray(detection.corners, dtype=np.float64))
        if len(img) < 4:
            per_frame.append({"file": row.get("file"), "used": False, "reason": "fewer than 4 corners"})
            if progress_callback:
                progress_callback(frame_index, total_frames)
            continue
        object_points.append(np.asarray(obj, np.float32))
        image_points.append(np.asarray(img, np.float32))
        per_frame.append({"file": row.get("file"), "used": True, "corners": len(img)})
        if progress_callback:
            progress_callback(frame_index, total_frames)
    if len(object_points) < max(3, min_frames):
        raise RuntimeError(f"usable frames {len(object_points)} below required {max(3, min_frames)}")

    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    errors: list[list[float]] = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)
        errors.append(np.linalg.norm(projected.reshape(-1, 2) - img, axis=1).tolist())
    if max_outlier_px > 0:
        keep_obj, keep_img = [], []
        for obj, img, err in zip(object_points, image_points, errors):
            mask = np.asarray(err) <= max_outlier_px
            if int(mask.sum()) >= 4:
                keep_obj.append(obj[mask])
                keep_img.append(img[mask])
        if sum(len(item) for item in keep_obj) >= 12:
            rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
                keep_obj, keep_img, image_size, matrix, distortion,
                flags=cv2.CALIB_USE_INTRINSIC_GUESS,
            )
    per_view = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)
        per_view.append(float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - img) ** 2, axis=1)))))
    candidate = {
        "calibration_id": str(uuid.uuid4()),
        "created_at": time.time(),
        "image_size": [int(image_size[0]), int(image_size[1])],
        "intrinsic": {"fx": float(matrix[0, 0]), "fy": float(matrix[1, 1]), "cx": float(matrix[0, 2]), "cy": float(matrix[1, 2])},
        "distortion": distortion.reshape(-1).astype(float).tolist(),
        "distortion_model": "plumb_bob" if distortion.size == 5 else "rational_polynomial",
        "rms_px": float(rms),
        "per_view_rms_px": per_view,
        "used_frames": len(object_points),
        "accepted": len(object_points) >= min_frames and float(rms) <= max_rms_px,
        "thresholds": {"min_frames": min_frames, "max_rms_px": max_rms_px, "max_outlier_px": max_outlier_px},
        "per_frame": per_frame,
    }
    (session_dir / f"candidate_{candidate['calibration_id']}.json").write_text(
        json.dumps(candidate, indent=2), encoding="utf-8"
    )
    return candidate


def apply_rgb_candidate(candidate: dict[str, Any], profile_file: Path) -> None:
    """应用rgbcandidate。

    Args:
        candidate: candidate，作为应用rgbcandidate所需的输入。 类型为 dict[str, Any]。
        profile_file: profile文件，作为应用rgbcandidate所需的输入。 类型为 Path。

    Returns:
        无返回值。
    """
    if not candidate.get("accepted"):
        raise ValueError("candidate did not meet configured quality thresholds")
    existing: dict[str, Any] = {}
    if profile_file.exists():
        existing = json.loads(profile_file.read_text(encoding="utf-8"))
    existing.setdefault("streams", {})["color"] = {
        "image_size": candidate["image_size"],
        "intrinsic": candidate["intrinsic"],
        "distortion": candidate["distortion"],
        "distortion_model": candidate["distortion_model"],
        "is_mirrored": False,
    }
    existing["calibration_id"] = candidate["calibration_id"]
    existing["source"] = "EmbodiedNexus RGB calibration"
    existing.setdefault("history", []).append(candidate)
    profile_file.parent.mkdir(parents=True, exist_ok=True)
    temp = profile_file.with_suffix(profile_file.suffix + ".tmp")
    temp.write_text(json.dumps(existing, indent=2), encoding="utf-8")
    temp.replace(profile_file)
