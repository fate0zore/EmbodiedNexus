"""Pure control calculations for a lease-protected visual servo."""

import math

import numpy as np


def orientation_error(target, actual):
    """Compute the shortest angle between two orientations.

    Args:
        target: Desired quaternion in XYZW order.
        actual: Measured quaternion in XYZW order.

    Returns:
        Rotation error in radians.
    """
    target, actual = np.asarray(target, dtype=float), np.asarray(actual, dtype=float)
    if (target.shape != (4,) or actual.shape != (4,) or not np.isfinite(target).all()
            or not np.isfinite(actual).all() or np.linalg.norm(target) < 1e-9
            or np.linalg.norm(actual) < 1e-9):
        raise ValueError('invalid orientation quaternion')
    cosine = abs(float(np.dot(target / np.linalg.norm(target), actual / np.linalg.norm(actual))))
    return 2.0 * math.acos(min(1.0, cosine))


def continuous_grasp_orientation(target_yaw, reference):
    """Select the equivalent parallel-jaw orientation nearest the reference.

    Args:
        target_yaw: Object principal-axis angle in radians, modulo pi.
        reference: Previous grasp quaternion in XYZW order.

    Returns:
        The nearest equivalent downward-facing grasp quaternion in XYZW order.
    """
    if not math.isfinite(float(target_yaw)):
        raise ValueError('invalid target yaw')
    closing = float(target_yaw) + math.pi / 2
    candidates = [[math.cos((closing + offset) / 2),
                   math.sin((closing + offset) / 2), 0.0, 0.0]
                  for offset in (0.0, math.pi)]
    return min(candidates, key=lambda candidate: orientation_error(candidate, reference))


def cartesian_velocity(target, actual, gain, max_speed, tolerance):
    """Calculate bounded linear velocity toward a grasp-frame position.

    Args:
        target: Target XYZ in the planning frame, in meters.
        actual: Current XYZ in the planning frame, in meters.
        gain: Proportional gain in inverse seconds.
        max_speed: Maximum linear speed in meters per second.
        tolerance: Position tolerance in meters.

    Returns:
        A tuple containing a three-element velocity list and convergence flag.
    """
    target = np.asarray(target, dtype=float)
    actual = np.asarray(actual, dtype=float)
    if (target.shape != (3,) or actual.shape != (3,) or
            not np.isfinite(target).all() or not np.isfinite(actual).all() or
            not all(math.isfinite(float(x)) and x > 0 for x in (gain, max_speed, tolerance))):
        raise ValueError("invalid visual servo control inputs")
    error = target - actual
    distance = float(np.linalg.norm(error))
    if distance <= tolerance:
        return [0.0, 0.0, 0.0], True
    velocity = error * gain
    norm = float(np.linalg.norm(velocity))
    if norm > max_speed:
        velocity *= max_speed / norm
    return velocity.tolist(), False


def angular_velocity(target, actual, gain, max_speed):
    """Calculate bounded angular correction between XYZW orientations.

    Args:
        target: Desired unit quaternion in XYZW order.
        actual: Measured quaternion in XYZW order.
        gain: Angular proportional gain per second.
        max_speed: Maximum angular speed in radians per second.

    Returns:
        Three angular velocity components in the planning frame.
    """
    target, actual = np.asarray(target, dtype=float), np.asarray(actual, dtype=float)
    if (target.shape != (4,) or actual.shape != (4,) or not np.isfinite(target).all()
            or not np.isfinite(actual).all() or np.linalg.norm(target) < 1e-9
            or np.linalg.norm(actual) < 1e-9 or not math.isfinite(float(gain))
            or not math.isfinite(float(max_speed)) or gain <= 0 or max_speed <= 0):
        raise ValueError("invalid orientation control inputs")
    target, actual = target / np.linalg.norm(target), actual / np.linalg.norm(actual)
    tv, tw = target[:3], target[3]
    av, aw = -actual[:3], actual[3]
    error_xyz = tw * av + aw * tv + np.cross(tv, av)
    error_w = tw * aw - float(tv @ av)
    if error_w < 0:
        error_xyz = -error_xyz
    result = 2.0 * gain * error_xyz
    norm = float(np.linalg.norm(result))
    if norm > max_speed:
        result *= max_speed / norm
    return result.tolist()


def map_servo_velocities(names, velocities, configured_names, velocity_limit):
    """Map MoveIt Servo joint velocities to the Nexus device joint order.

    Args:
        names: Joint names in the Servo output.
        velocities: Velocity values in the Servo output, in radians per second.
        configured_names: Nexus arm joint order.
        velocity_limit: Maximum absolute accepted velocity in radians per second.

    Returns:
        Joint velocity list ordered for the device interface.
    """
    names = list(names)
    expected = list(configured_names)
    values = np.asarray(velocities, dtype=float)
    if (not names or len(set(names)) != len(names) or len(set(expected)) != len(expected)
            or set(names) != set(expected) or values.shape != (len(names),)
            or not np.isfinite(values).all() or not math.isfinite(float(velocity_limit))
            or velocity_limit <= 0 or np.any(np.abs(values) > velocity_limit)):
        raise ValueError("invalid or out-of-limit MoveIt Servo joint velocities")
    by_name = dict(zip(names, values))
    return [float(by_name[name]) for name in expected]


class JointVelocityTracker:
    """Track integrated Servo joint motion against measured arm positions."""

    def __init__(self, positions, gain, limit):
        """Initialize the joint reference from measured positions.

        Args:
            positions: Initial measured joint positions in radians.
            gain: Position-error correction gain per second.
            limit: Maximum output magnitude in radians per second.

        Returns:
            None.
        """
        self.reference = np.asarray(positions, dtype=float).copy()
        if (self.reference.ndim != 1 or not len(self.reference) or not np.isfinite(self.reference).all()
                or not math.isfinite(float(gain)) or gain <= 0
                or not math.isfinite(float(limit)) or limit <= 0):
            raise ValueError("invalid joint tracker inputs")
        self.gain, self.limit = float(gain), float(limit)

    def step(self, positions, velocity, dt):
        """Integrate the command and correct measured joint tracking error.

        Args:
            positions: Current measured joint positions in configured order.
            velocity: Collision-checked Servo velocities in the same order.
            dt: Elapsed simulation time in seconds.

        Returns:
            Bounded joint velocity command in radians per second.
        """
        actual, desired = np.asarray(positions, dtype=float), np.asarray(velocity, dtype=float)
        if (actual.shape != self.reference.shape or desired.shape != self.reference.shape or
                not np.isfinite(actual).all() or not np.isfinite(desired).all() or
                not math.isfinite(float(dt)) or dt <= 0 or dt > 0.5):
            raise ValueError("invalid joint tracker update")
        self.reference += desired * dt
        error = self.reference - actual
        if np.max(np.abs(error)) > 0.15:
            raise RuntimeError("joint velocity tracking error exceeded 0.15 rad")
        command = desired + self.gain * error
        peak = float(np.max(np.abs(command)))
        if peak > self.limit:
            command *= self.limit / peak
        return command.tolist()
