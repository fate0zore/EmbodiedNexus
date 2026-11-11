"""Pure motion control calculations for a dynamic target."""

import math

import numpy as np

from .geometry import matrix_quaternion
from .tracking import _unit_quaternion


def target_grasp_orientation(orientation, shape, previous):
    """Choose a continuous tool orientation following the target's 3-D axes.

    Args:
        orientation: Observed target orientation in xyzw order.
        shape: Either cylinder or box.
        previous: Previous tool orientation in xyzw order.

    Returns:
        Unit tool orientation in xyzw order, choosing the nearest equivalent grasp.

    Raises:
        ValueError: If the target or previous orientation is invalid.
    """
    def rotation_matrix(quaternion):
        """Convert an xyzw quaternion to a three-dimensional rotation matrix.

        Args:
            quaternion: Quaternion in xyzw order.

        Returns:
            Three-by-three unit rotation matrix.
        """
        x, y, z, w = _unit_quaternion(quaternion)
        return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

    target = rotation_matrix(orientation)
    previous_q = _unit_quaternion(previous)
    if shape == "cylinder":
        long_axis = target[:, 2]
        down = np.array([0., 0., -1.])
        normal = down - long_axis * np.dot(down, long_axis)
        if np.linalg.norm(normal) < 0.15:
            previous_normal = rotation_matrix(previous_q)[:, 2]
            normal = previous_normal - long_axis * np.dot(previous_normal, long_axis)
        if np.linalg.norm(normal) < 1e-6:
            previous_side = rotation_matrix(previous_q)[:, 0]
            normal = previous_side - long_axis * np.dot(previous_side, long_axis)
        if np.linalg.norm(normal) < 1e-6:
            raise ValueError("cylinder approach orientation is unobservable")
        normal /= np.linalg.norm(normal)
        normal_signs = (1.0,)
    elif shape == "box":
        long_axis = target[:, 0]
        normal = target[:, 2]
        # Prefer a downward-facing approach when the face normal has an
        # observable vertical component; preserve continuity near vertical faces.
        normal_signs = ((-1.0,) if normal[2] > 0.15 else
                        (1.0,) if normal[2] < -0.15 else (-1.0, 1.0))
    else:
        raise ValueError("unsupported target shape")
    candidates = []
    for axis_sign in (-1.0, 1.0):
        for normal_sign in normal_signs:
            tool_y = axis_sign * long_axis
            tool_z = normal_sign * normal
            tool_x = np.cross(tool_y, tool_z)
            tool_x /= np.linalg.norm(tool_x)
            candidate = matrix_quaternion(np.column_stack((tool_x, tool_y, tool_z)))
            candidates.append(candidate if np.dot(candidate, previous_q) >= 0 else -candidate)
    return max(candidates, key=lambda candidate: float(np.dot(candidate, previous_q)))


def tracking_velocity(target_position, target_velocity, tool_position, offset, gain, speed_limit):
    """Calculate a bounded translational velocity toward a moving offset pose.

    Args:
        target_position: Target center in the fixed frame, in meters.
        target_velocity: Estimated target velocity in the same frame, in meters per second.
        tool_position: Measured tool position in the same frame, in meters.
        offset: Desired tool offset from the target, in meters in the fixed frame.
        gain: Nonnegative position-error gain in inverse seconds.
        speed_limit: Maximum commanded linear speed in meters per second.

    Returns:
        Pair of three-element arrays containing commanded velocity and position error.

    Raises:
        ValueError: If an input is invalid or non-finite.
    """
    values = [np.asarray(value, dtype=float) for value in
              (target_position, target_velocity, tool_position, offset)]
    if any(value.shape != (3,) or not np.all(np.isfinite(value)) for value in values):
        raise ValueError("invalid tracking vector")
    if not math.isfinite(gain) or gain < 0 or not math.isfinite(speed_limit) or speed_limit <= 0:
        raise ValueError("invalid tracking gain or speed limit")
    target, feedforward, tool, offset = values
    error = target + offset - tool
    velocity = feedforward + gain * error
    speed = np.linalg.norm(velocity)
    if speed > speed_limit:
        velocity *= speed_limit / speed
    return velocity, error
