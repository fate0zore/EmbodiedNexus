"""Load the shared Gemini 335 camera calibration for MuJoCo."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np


CAMERA_FROM_OPTICAL = np.array([
    [0.0, 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
])


def load_camera_calibration(description_dir: Path, camera_dir: Path) -> dict[str, Any]:
    """Load the same camera mount and factory RGB-D calibration as Gazebo.

    Args:
        description_dir: Installed shared robot description package directory.
        camera_dir: Installed camera package directory.

    Returns:
        Mount transforms and ideal-rectified stream intrinsics for color and depth.
    """
    mount = json.loads((description_dir / "config/gemini335_extrinsics.json").read_text(encoding="utf-8"))
    factory = json.loads((camera_dir / "config/gemini335_factory_calibration.json").read_text(encoding="utf-8"))

    color_optical = np.asarray(mount["matrix_4x4"], dtype=float)
    depth_to_color = np.eye(4)
    depth_to_color[:3, :3] = np.asarray(factory["transform_depth_to_color"]["rot"], dtype=float).reshape(3, 3)
    depth_to_color[:3, 3] = np.asarray(factory["transform_depth_to_color"]["trans"], dtype=float) / 1000.0
    optical_poses = {"color": color_optical, "depth": color_optical @ depth_to_color}

    streams: dict[str, dict[str, float | int]] = {}
    for channel in ("color", "depth"):
        stream = factory["streams"][channel]
        width, height = (int(value) for value in stream["image_size"])
        intrinsic = stream["intrinsic"]
        streams[channel] = {
            "width": width,
            "height": height,
            "fx": float(intrinsic["fx"]),
            "fy": float(intrinsic["fy"]),
            "cx": float(intrinsic["cx"]),
            "cy": float(intrinsic["cy"]),
        }

    sizes = {(stream["width"], stream["height"]) for stream in streams.values()}
    if len(sizes) != 1:
        raise ValueError("MuJoCo RGB-D rendering requires matching color and depth resolutions")

    return {
        "mount_parent": mount["parent_frame"],
        "mount_name": mount["name"],
        "factory_calibration_id": factory["calibration_id"],
        "camera_poses": {
            channel: optical_pose @ np.linalg.inv(CAMERA_FROM_OPTICAL)
            for channel, optical_pose in optical_poses.items()
        },
        "streams": streams,
    }


def apply_camera_calibration(urdf: str, calibration: dict[str, Any]) -> str:
    """Set color and depth camera frames from the shared calibrated poses.

    Args:
        urdf: Expanded Arachne robot description XML.
        calibration: Camera calibration returned by ``load_camera_calibration``.

    Returns:
        Robot description XML with calibrated camera frame joint origins.
    """
    robot = ET.fromstring(urdf)
    parent_frame = calibration["mount_parent"]
    for channel in ("color", "depth"):
        joint_name = f"camera_{channel}_frame_joint"
        joint = robot.find(f"joint[@name='{joint_name}']")
        parent = joint.find("parent") if joint is not None else None
        if parent is None or parent.get("link") != parent_frame:
            raise ValueError(f"{joint_name} must use calibrated parent frame {parent_frame!r}")
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        xyz, rpy = _pose_xyz_rpy(calibration["camera_poses"][channel])
        origin.set("xyz", " ".join(f"{value:.12g}" for value in xyz))
        origin.set("rpy", " ".join(f"{value:.12g}" for value in rpy))
    return ET.tostring(robot, encoding="unicode")


def _pose_xyz_rpy(matrix: np.ndarray) -> tuple[list[float], list[float]]:
    """Convert a homogeneous transform to URDF xyz and fixed-axis RPY values.

    Args:
        matrix: Homogeneous transform matrix.

    Returns:
        Translation and roll-pitch-yaw lists in meters and radians.
    """
    pitch = math.atan2(-matrix[2, 0], math.hypot(matrix[0, 0], matrix[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-matrix[0, 1], matrix[1, 1])
    else:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return matrix[:3, 3].tolist(), [roll, pitch, yaw]
