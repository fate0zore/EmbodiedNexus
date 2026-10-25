"""Camera geometry from the installed factory profile and active Yahboom mount."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory


# REP 103: optical +Z is the camera's forward +X; optical +X points right.
CAMERA_FROM_OPTICAL = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                                [0., -1., 0., 0.], [0., 0., 0., 1.]])


def camera_geometry(width: int, height: int) -> dict:
    camera_share = Path(get_package_share_directory("embodied_nexus_camera"))
    description_share = Path(get_package_share_directory("embodied_nexus_sim_description"))
    factory = json.loads((camera_share / "config/gemini335_factory_calibration.json").read_text())
    mount = json.loads((description_share / "config/gemini335_extrinsics.json").read_text())
    color_optical = np.asarray(mount["matrix_4x4"], dtype=float)
    depth_to_color = np.eye(4)
    depth_to_color[:3, :3] = np.asarray(factory["transform_depth_to_color"]["rot"]).reshape(3, 3)
    depth_to_color[:3, 3] = np.asarray(factory["transform_depth_to_color"]["trans"]) / 1000.0
    calibration = copy.deepcopy(factory)
    calibration["calibration_id"] = "yahboom-gemini335-sim-rectified"
    calibration["source"] = "Yahboom active config; ideal rectified simulation"
    poses = {"color": color_optical, "depth": color_optical @ depth_to_color}
    result = {"calibration": calibration, "mount_parent": mount["parent_frame"]}
    for channel in ("color", "depth"):
        stream = calibration["streams"][channel]
        source_width, source_height = stream["image_size"]
        if width * source_height != height * source_width:
            raise ValueError("simulation camera resolution must preserve the calibrated aspect ratio")
        intrinsic = stream["intrinsic"]
        for name, factor in (("fx", width / source_width), ("cx", width / source_width),
                             ("fy", height / source_height), ("cy", height / source_height)):
            intrinsic[name] *= factor
        stream["image_size"] = [width, height]
        # Gazebo generates ideal rectified images, not the raw distorted sensor image.
        stream["distortion"], stream["distortion_model"] = [0.0] * 5, "plumb_bob"
        result[channel] = {"intrinsic": intrinsic,
                           "pose": poses[channel] @ np.linalg.inv(CAMERA_FROM_OPTICAL)}
    return result


def pose_xyz_rpy(matrix: np.ndarray) -> tuple[list[float], list[float]]:
    pitch = math.atan2(-matrix[2, 0], math.hypot(matrix[0, 0], matrix[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll, yaw = 0.0, math.atan2(-matrix[0, 1], matrix[1, 1])
    else:
        roll, yaw = math.atan2(matrix[2, 1], matrix[2, 2]), math.atan2(matrix[1, 0], matrix[0, 0])
    return matrix[:3, 3].tolist(), [roll, pitch, yaw]
