"""RGB-D projection and top-down box pose estimation helpers."""

from __future__ import annotations

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def gripper_opening_upper_bound(description_dir):
    """Compute an upper bound on jaw opening from the installed robot mesh.

    Args:
        description_dir: Installed shared robot description package directory.

    Returns:
        Conservative upper bound in meters; not a calibrated usable opening.
    """
    root = Path(description_dir)
    text = subprocess.run(["xacro", str(root / "urdf/arachne.urdf.xacro"),
                           "gazebo_drive_wheels:=true", "with_mimic_joints:=false"],
                          check=True, capture_output=True, text=True).stdout
    robot = ET.fromstring(text)
    pivots, radii = [], []
    triangle = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    for side in ("left", "right"):
        joint = robot.find(f".//joint[@name='ms42dc_{side}_finger_joint']")
        collision = robot.find(f".//link[@name='ms42dc_{side}_finger_link']/collision")
        if joint is None or collision is None:
            raise ValueError("robot model is missing a gripper joint or collision mesh")
        pivot = np.fromstring(joint.find("origin").get("xyz"), sep=" ")
        origin = np.fromstring(collision.find("origin").get("xyz", "0 0 0"), sep=" ")
        mesh = collision.find("geometry/mesh")
        prefix = "package://embodied_nexus_sim_description/"
        filename = mesh.get("filename", "")
        if not filename.startswith(prefix):
            raise ValueError("unsupported gripper mesh URI")
        scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
        vertices = np.frombuffer((root / filename[len(prefix):]).read_bytes(), dtype=triangle,
                                 offset=84)["vertices"].reshape(-1, 3) * scale + origin
        pivots.append(pivot[:2])
        radii.append(float(np.linalg.norm(vertices[:, :2], axis=1).max()))
    return float(np.linalg.norm(pivots[1] - pivots[0]) + sum(radii))


def decode_image(message):
    """Decode a supported ROS image message into a contiguous NumPy array.

    Args:
        message: ROS sensor_msgs/Image message.

    Returns:
        Image array with shape HxW or HxWxC and native ROS encoding values.
    """
    encoding = str(message.encoding).lower()
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(int(message.height), int(message.step))
    if encoding in {"rgb8", "bgr8"}:
        return rows[:, : int(message.width) * 3].reshape(int(message.height), int(message.width), 3).copy()
    if encoding == "mono8":
        return rows[:, : int(message.width)].copy()
    if encoding in {"16uc1", "mono16"}:
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        return rows[:, : int(message.width) * 2].copy().view(dtype).reshape(
            int(message.height), int(message.width)).astype(np.uint16, copy=False)
    if encoding == "32fc1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        return rows[:, : int(message.width) * 4].copy().view(dtype).reshape(
            int(message.height), int(message.width)).astype(np.float32, copy=False)
    raise ValueError(f"unsupported ROS image encoding: {message.encoding}")


def depth_to_points(depth, mask, depth_info, color_info, color_from_depth,
                    depth_scale_m=0.001):
    """Project depth pixels through the color-camera model and select masked points.

    Args:
        depth: HxW depth array in millimeters for uint16 or meters for float32.
        mask: HxW binary target mask from instance segmentation.
        depth_info: CameraInfo message for the depth stream.
        color_info: CameraInfo message for the color stream.
        color_from_depth: TransformStamped from depth optical frame to color optical frame.
        depth_scale_m: Meters represented by one integer depth unit.

    Returns:
        Nx3 optical-frame points in meters.
    """
    depth_values = np.asarray(depth)
    target_mask = np.asarray(mask, dtype=bool)
    if depth_values.ndim != 2 or target_mask.ndim != 2:
        raise ValueError("depth image and segmentation mask must be two-dimensional")
    if not math.isfinite(float(depth_scale_m)) or float(depth_scale_m) <= 0:
        raise ValueError("depth_scale_m must be finite and positive")
    if depth_values.dtype == np.uint16:
        depth_m = depth_values.astype(np.float32) * float(depth_scale_m)
    else:
        depth_m = depth_values.astype(np.float32)
    dfx, dfy, dcx, dcy = (float(depth_info.k[index]) for index in (0, 4, 2, 5))
    cfx, cfy, ccx, ccy = (float(color_info.k[index]) for index in (0, 4, 2, 5))
    if not all(math.isfinite(value) and value > 0 for value in (dfx, dfy, cfx, cfy)):
        raise ValueError("depth CameraInfo has invalid focal lengths")
    valid = np.isfinite(depth_m) & (depth_m >= 0.05) & (depth_m <= 3.0)
    rows, columns = np.nonzero(valid)
    if not len(rows):
        return np.empty((0, 3), dtype=np.float64)
    z = depth_m[rows, columns].astype(np.float64)
    x = (columns.astype(np.float64) - dcx) * z / dfx
    y = (rows.astype(np.float64) - dcy) * z / dfy
    points_depth = np.column_stack((x, y, z))
    points_color = transform_points(points_depth, color_from_depth)
    front = points_color[:, 2] > 1e-6
    u = np.rint(cfx * points_color[:, 0] / np.maximum(points_color[:, 2], 1e-6) + ccx).astype(int)
    v = np.rint(cfy * points_color[:, 1] / np.maximum(points_color[:, 2], 1e-6) + ccy).astype(int)
    inside = front & (u >= 0) & (u < target_mask.shape[1]) & (v >= 0) & (v < target_mask.shape[0])
    selected = np.zeros(len(points_depth), dtype=bool)
    selected[inside] = target_mask[v[inside], u[inside]]
    return points_depth[selected]


def transform_points(points, transform):
    """Transform points using a geometry_msgs/TransformStamped value.

    Args:
        points: Nx3 coordinates in the transform source frame.
        transform: TransformStamped from source frame to target frame.

    Returns:
        Nx3 coordinates in the transform target frame.
    """
    values = np.asarray(points, dtype=np.float64)
    rotation = transform.transform.rotation
    x, y, z, w = float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)
    matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    translation = np.array([transform.transform.translation.x, transform.transform.translation.y,
                            transform.transform.translation.z], dtype=np.float64)
    return values @ matrix.T + translation


def rotate_vector(vector, quaternion_xyzw):
    """Rotate one XYZ vector by a quaternion without applying translation.

    Args:
        vector: Three-dimensional input vector.
        quaternion_xyzw: Quaternion in XYZW order.

    Returns:
        Rotated XYZ vector as a NumPy array.
    """
    value = np.asarray(vector, dtype=np.float64)
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if value.shape != (3,) or quaternion.shape != (4,):
        raise ValueError("vector and quaternion must have three and four values respectively")
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(value).all() or not np.isfinite(quaternion).all() or norm <= 1e-12:
        raise ValueError("vector and quaternion must contain finite values and quaternion must be nonzero")
    quaternion /= norm
    xyz = quaternion[:3]
    twice_cross = 2.0 * np.cross(xyz, value)
    return value + quaternion[3] * twice_cross + np.cross(xyz, twice_cross)


def estimate_box_pose(points_base, box_size, table_height_m, minimum_points=80):
    """Estimate a known-size box center and yaw from its segmented point cloud.

    Args:
        points_base: Segmented points in the robot base frame.
        box_size: Known box dimensions [x, y, z] in meters.
        table_height_m: Table top height in the robot base frame.
        minimum_points: Minimum inlier count required to return a pose estimate.

    Returns:
        Mapping with center, yaw, and retained point count.
    """
    points = np.asarray(points_base, dtype=np.float64)
    size = np.asarray(box_size, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or size.shape != (3,):
        raise ValueError("box pose estimation requires Nx3 points and three known dimensions")
    if len(points) < int(minimum_points):
        raise ValueError(f"segmented point cloud contains only {len(points)} valid points")
    low = float(table_height_m) - 0.025
    high = float(table_height_m) + float(size[2]) + 0.06
    points = points[(points[:, 2] >= low) & (points[:, 2] <= high)]
    if len(points) < int(minimum_points):
        raise ValueError("segmented points do not overlap the configured tabletop box region")
    median = np.median(points, axis=0)
    distances = np.linalg.norm(points[:, :2] - median[:2], axis=1)
    inliers = points[distances <= max(float(np.percentile(distances, 90)), 0.025)]
    if len(inliers) < int(minimum_points):
        raise ValueError("point cloud outlier filtering left too few box points")
    top_threshold = float(np.percentile(inliers[:, 2], 60))
    top = inliers[inliers[:, 2] >= top_threshold]
    horizontal = top[:, :2] if len(top) >= int(minimum_points) else inliers[:, :2]
    centered = horizontal - np.mean(horizontal, axis=0)
    covariance = centered.T @ centered / max(len(centered), 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    yaw = math.atan2(float(axis[1]), float(axis[0]))
    # Perspective makes the near side denser in pixel-projected point clouds;
    # the midpoint of robust extents is less biased than the point median.
    extents = np.percentile(horizontal, [5.0, 95.0], axis=0)
    center_xy = np.mean(extents, axis=0)
    observed_top = float(np.percentile(top[:, 2], 80)) if len(top) else float(np.percentile(inliers[:, 2], 80))
    center_z = observed_top - float(size[2]) / 2.0
    expected_center_z = float(table_height_m) + float(size[2]) / 2.0
    if abs(center_z - expected_center_z) > 0.05:
        raise ValueError("RGB-D box height estimate is inconsistent with the configured tabletop")
    return {"center": [float(center_xy[0]), float(center_xy[1]), float(center_z)],
            "yaw": yaw, "point_count": int(len(inliers))}


def measure_target_geometry(points_base, table_height_m, minimum_points=80, shape="box"):
    """Measure a tabletop object's visible point cloud in the base frame.

    Args:
        points_base: Segmented Nx3 points in meters.
        table_height_m: Known horizontal support height in the base frame.
        minimum_points: Minimum count of filtered object points.
        shape: Cylinder for a horizontal circular cylinder, otherwise box.

    Returns:
        Center, horizontal PCA yaw, dimensions, point counts and fit diagnostics.
    """
    points = np.asarray(points_base, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("target measurement requires Nx3 points")
    table = float(table_height_m)
    if not math.isfinite(table):
        raise ValueError("support height must be finite")
    input_count = len(points)
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(points[:, 2] > table + 0.002) & (points[:, 2] < table + 0.5)]
    if len(points) < minimum_points:
        raise ValueError("too few valid target points above the support surface")
    center = np.median(points[:, :2], axis=0)
    distances = np.linalg.norm(points[:, :2] - center, axis=1)
    points = points[distances <= np.percentile(distances, 97)]
    if len(points) < minimum_points:
        raise ValueError("too few target points after outlier filtering")
    covariance = np.cov(points[:, :2], rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, -1]
    yaw = math.atan2(float(axis[1]), float(axis[0]))
    lateral = np.array([-axis[1], axis[0]])
    local = np.column_stack((points[:, :2] @ axis, points[:, :2] @ lateral))
    lower, upper = np.percentile(local, [2, 98], axis=0)
    size_xy = upper - lower
    height = float(np.percentile(points[:, 2], 98) - table)
    if shape == "cylinder":
        size_xy[1] = height
    size = [float(size_xy[0]), float(size_xy[1]), height]
    if not np.isfinite(size).all() or min(size) < 0.005 or max(size) > 0.5:
        raise ValueError(f"measured target dimensions are invalid: {size}")
    xy = axis * ((lower[0] + upper[0]) / 2) + lateral * ((lower[1] + upper[1]) / 2)
    return {"center": [float(xy[0]), float(xy[1]), table + height / 2],
            "yaw": yaw, "size": size, "point_count": len(points),
            "measurement": {"input_points": input_count, "inlier_points": len(points),
                            "horizontal_eigenvalues": eigenvalues.tolist(),
                            "visible_width_m": float(upper[1] - lower[1]),
                            "shape": shape}}


def validate_target_mask(mask, minimum_pixels):
    """Validate segmentation coverage before projecting RGB-D pixels.

    Args:
        mask: Two-dimensional segmented target mask.
        minimum_pixels: Minimum number of segmented pixels.

    Returns:
        Number of segmented pixels; raises ValueError if the mask is unusable.
    """
    target = np.asarray(mask, dtype=bool)
    if target.ndim != 2 or min(target.shape) < 2:
        raise ValueError("target mask must be a two-dimensional image")
    count = int(target.sum())
    if count < minimum_pixels:
        raise ValueError("target mask contains too few pixels")
    if target[0, :].any() or target[-1, :].any() or target[:, 0].any() or target[:, -1].any():
        raise ValueError("target segmentation touches the image boundary")
    return count


def grasp_closing_yaw(object_yaw, box_size, symmetric_box_grasp_yaw=0.0):
    """Choose a top-down jaw closing axis for a box's shorter horizontal side.

    Args:
        object_yaw: PCA yaw of the box's longest horizontal edge, in radians.
        box_size: Known box dimensions [x, y, z] in meters.
        symmetric_box_grasp_yaw: Fixed closing axis used when the horizontal
            dimensions are equal within 5 mm.

    Returns:
        Closing-axis yaw in radians, normalized modulo pi to [-pi/2, pi/2).
    """
    size = np.asarray(box_size, dtype=np.float64)
    yaw = float(object_yaw)
    fallback = float(symmetric_box_grasp_yaw)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0.0):
        raise ValueError("box_size must contain three positive finite dimensions")
    if not math.isfinite(yaw) or not math.isfinite(fallback):
        raise ValueError("object and symmetric grasp yaw must be finite")
    closing_yaw = fallback if abs(float(size[0] - size[1])) <= 0.005 else yaw + math.pi / 2.0
    return (closing_yaw + math.pi / 2.0) % math.pi - math.pi / 2.0


def top_down_quaternion(yaw):
    """Return an XYZW quaternion with tool Z pointing down and X along box yaw.

    Args:
        yaw: Desired gripper closing-axis angle in the base XY plane, in radians.

    Returns:
        Unit XYZW quaternion.
    """
    half = float(yaw) / 2.0
    return [math.cos(half), math.sin(half), 0.0, 0.0]
