from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from .calibration import CameraModel, RgbdCalibration


def pointcloud_xyz(msg: Any) -> np.ndarray:
    """Read XYZ fields from a PointCloud2-like message without changing its layout.

    Args:
        msg: 待处理的消息对象。 类型为 Any。

    Returns:
        本次处理产生的结果值。 返回类型为 np.ndarray。
    """
    fields = {str(field.name): field for field in msg.fields}
    formats = []
    names = []
    offsets = []
    prefix = ">" if bool(msg.is_bigendian) else "<"
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or int(field.count) != 1:
            raise ValueError("PointCloud2 must contain scalar x, y and z fields")
        if int(field.datatype) == 7:
            dtype = prefix + "f4"
        elif int(field.datatype) == 8:
            dtype = prefix + "f8"
        else:
            raise ValueError(f"PointCloud2 {name} field must be FLOAT32 or FLOAT64")
        names.append(name)
        formats.append(dtype)
        offsets.append(int(field.offset))
    width, height, point_step = int(msg.width), int(msg.height), int(msg.point_step)
    row_step = int(msg.row_step) or width * point_step
    if width < 0 or height < 1 or point_step < 1 or row_step < width * point_step:
        raise ValueError("PointCloud2 layout is invalid")
    if len(msg.data) < row_step * height:
        raise ValueError("PointCloud2 data is shorter than its declared layout")
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})
    structured = np.ndarray(
        (height, width), dtype=dtype, buffer=msg.data,
        strides=(row_step, point_step),
    )
    return np.column_stack([structured[name].reshape(-1) for name in names]).astype(np.float64)


def sample_xyz_rgb(xyz_color: np.ndarray, color_bgr: np.ndarray, model: CameraModel) -> np.ndarray:
    """Project XYZ points already in the RGB optical frame and return RGB bytes.

    Args:
        xyz_color: xyzcolor，作为将 RGB 光学坐标系中的 XYZ 点投影到 RGB 图像并读取颜色所需的输入。 类型为 np.ndarray。
        color_bgr: colorbgr，作为将 RGB 光学坐标系中的 XYZ 点投影到 RGB 图像并读取颜色所需的输入。 类型为 np.ndarray。
        model: model，作为将 RGB 光学坐标系中的 XYZ 点投影到 RGB 图像并读取颜色所需的输入。 类型为 CameraModel。

    Returns:
        本次处理产生的结果值。 返回类型为 np.ndarray。
    """
    xyz = np.asarray(xyz_color, dtype=np.float64).reshape(-1, 3)
    rgb = np.zeros((len(xyz), 3), dtype=np.uint8)
    valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 1e-6)
    indices = np.flatnonzero(valid)
    if not indices.size:
        return rgb
    projected, _ = cv2.projectPoints(
        xyz[indices].reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
        model.matrix, model.cv_distortion,
    )
    uv = np.rint(projected.reshape(-1, 2)).astype(np.int64)
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < model.width) & (uv[:, 1] >= 0) & (uv[:, 1] < model.height)
    chosen = indices[inside]
    bgr = color_bgr[uv[inside, 1], uv[inside, 0], :3]
    rgb[chosen] = bgr[:, ::-1]
    return rgb


def depth_to_xyzrgb(
    depth_u16: np.ndarray,
    color_bgr: np.ndarray,
    calibration: RgbdCalibration,
    *,
    stride: int = 4,
    min_depth_m: float = 0.05,
    max_depth_m: float = 5.0,
    max_pair_delta_ms: float = 50.0,
    color_stamp: float | None = None,
    depth_stamp: float | None = None,
    max_points: int = 50000,
    roi: list[int] | tuple[int, int, int, int] | None = None,
) -> np.ndarray:
    """Return an N×6 float array [x,y,z,r,g,b] using SDK-exported RGB-D data.

    Args:
        depth_u16: depthu16，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 np.ndarray。
        color_bgr: colorbgr，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 np.ndarray。
        calibration: calibration，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 RgbdCalibration。
        stride: stride，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 int。
        min_depth_m: mindepthm，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 float。
        max_depth_m: maxdepthm，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 float。
        max_pair_delta_ms: maxpairdeltams，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 float。
        color_stamp: color时间戳，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 float | None。
        depth_stamp: depth时间戳，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 float | None。
        max_points: maxpoints，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 int。
        roi: roi，作为将深度图转换为包含 XYZ 和 RGB 的数组所需的输入。 类型为 list[int] | tuple[int, int, int, int] | None。

    Returns:
        本次处理产生的结果值。 返回类型为 np.ndarray。
    """
    if not calibration.usable_for_color_cloud:
        raise ValueError("valid color/depth calibration is required for XYZRGB")
    if color_stamp is not None and depth_stamp is not None:
        delta_ms = abs(color_stamp - depth_stamp) * 1000.0
        if delta_ms > max_pair_delta_ms:
            raise ValueError(f"RGB/depth time delta {delta_ms:.1f}ms exceeds limit")
    if depth_u16.shape != (calibration.depth.height, calibration.depth.width):
        raise ValueError("depth frame dimensions do not match active calibration")
    if color_bgr.shape[:2] != (calibration.color.height, calibration.color.width):
        raise ValueError("color frame dimensions do not match active calibration")
    step = max(int(stride), 1)
    if roi:
        if len(roi) != 4:
            raise ValueError("ROI must be [x, y, width, height]")
        x, y, width, height = (int(value) for value in roi)
        if x < 0 or y < 0 or width < 1 or height < 1 or x + width > depth_u16.shape[1] or y + height > depth_u16.shape[0]:
            raise ValueError("pointcloud ROI falls outside the depth frame")
    else:
        x, y, width, height = 0, 0, depth_u16.shape[1], depth_u16.shape[0]
    depth_region = depth_u16[y:y + height, x:x + width]
    rows, cols = np.mgrid[y:y + height:step, x:x + width:step]
    z = depth_region[::step, ::step].astype(np.float64) * calibration.depth_scale_m
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    u = cols[valid].astype(np.float64)
    v = rows[valid].astype(np.float64)
    z = z[valid]
    if not z.size:
        return np.empty((0, 6), dtype=np.float32)
    dmodel = calibration.depth
    # Invert the depth distortion before pinhole back-projection.
    pixels = np.column_stack((u, v)).reshape(-1, 1, 2)
    undistorted = cv2.undistortPoints(pixels, dmodel.matrix, dmodel.cv_distortion).reshape(-1, 2)
    xyz_depth = np.column_stack((undistorted[:, 0] * z, undistorted[:, 1] * z, z))
    xyz_color = xyz_depth @ calibration.depth_to_color_rotation.T + calibration.depth_to_color_translation_m
    positive = xyz_color[:, 2] > 1e-6
    xyz_depth, xyz_color = xyz_depth[positive], xyz_color[positive]
    if not xyz_color.size:
        return np.empty((0, 6), dtype=np.float32)
    cmodel = calibration.color
    projected, _ = cv2.projectPoints(
        xyz_color.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
        cmodel.matrix, cmodel.cv_distortion,
    )
    uv = np.rint(projected.reshape(-1, 2)).astype(np.int64)
    inside = (
        (uv[:, 0] >= 0) & (uv[:, 0] < cmodel.width)
        & (uv[:, 1] >= 0) & (uv[:, 1] < cmodel.height)
    )
    xyz_depth, uv = xyz_depth[inside], uv[inside]
    bgr = color_bgr[uv[:, 1], uv[:, 0], :3]
    rgb = bgr[:, ::-1].astype(np.float64)
    points = np.column_stack((xyz_depth, rgb)).astype(np.float32)
    if len(points) > max_points:
        take = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        points = points[take]
    return points


def pointcloud_metadata(fields: list[dict[str, Any]], point_count: int, frame_id: str, stamp_sec: float) -> dict[str, Any]:
    """点云metadata。

    Args:
        fields: fields，作为点云metadata所需的输入。 类型为 list[dict[str, Any]]。
        point_count: 点count，作为点云metadata所需的输入。 类型为 int。
        frame_id: 坐标系标识符，作为点云metadata所需的输入。 类型为 str。
        stamp_sec: 时间戳sec，作为点云metadata所需的输入。 类型为 float。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    return {
        "fields": fields,
        "point_count": int(point_count),
        "frame_id": frame_id,
        "stamp_sec": float(stamp_sec),
        "unit": "m",
    }
