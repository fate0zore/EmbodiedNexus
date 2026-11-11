"""Timestamped 3-D target motion estimation without simulator truth."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def _unit_quaternion(value):
    """Normalize an xyzw quaternion.

    Args:
        value: Four quaternion components in xyzw order.

    Returns:
        A unit NumPy quaternion.

    Raises:
        ValueError: If the input is not a finite nonzero quaternion.
    """
    quaternion = np.asarray(value, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("invalid quaternion")
    length = float(np.linalg.norm(quaternion))
    if length < 1e-9:
        raise ValueError("zero quaternion")
    return quaternion / length


def _multiply(left, right):
    """Multiply xyzw quaternions.

    Args:
        left: Left quaternion.
        right: Right quaternion.

    Returns:
        Quaternion product in xyzw order.
    """
    x, y, z, w = left
    a, b, c, d = right
    return np.array([w*a + x*d + y*c - z*b, w*b - x*c + y*d + z*a,
                     w*c + x*b - y*a + z*d, w*d - x*a - y*b - z*c])


def _axis(quaternion):
    """Return the cylinder's local z axis in the observation frame.

    Args:
        quaternion: Unit xyzw orientation.

    Returns:
        Three-dimensional unit axis.
    """
    x, y, z, w = quaternion
    return np.array([2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y)])


def equivalent_orientation(current, previous, shape):
    """Choose an orientation compatible with the object's geometric symmetry.

    Args:
        current: Observed orientation in xyzw order.
        previous: Previous accepted orientation in xyzw order.
        shape: Either box or cylinder.

    Returns:
        Unit xyzw quaternion with a continuous sign and equivalent box axis choice.

    Raises:
        ValueError: If an orientation or shape is invalid.
    """
    current = _unit_quaternion(current)
    previous = _unit_quaternion(previous)
    if shape == "box":
        symmetries = ([0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0])
        candidates = [_multiply(current, symmetry) for symmetry in symmetries]
        current = max(candidates, key=lambda value: abs(float(np.dot(value, previous))))
    elif shape != "cylinder":
        raise ValueError("unsupported target shape")
    if np.dot(current, previous) < 0:
        current = -current
    return current


def _rotation_delta(current, previous, shape):
    """Calculate observable world-frame rotation from two orientations.

    Args:
        current: Current unit orientation.
        previous: Previous unit orientation.
        shape: Either box or cylinder.

    Returns:
        Three-dimensional rotation vector in radians.
    """
    if shape == "cylinder":
        old_axis, new_axis = _axis(previous), _axis(current)
        if np.dot(old_axis, new_axis) < 0:
            new_axis = -new_axis
        cross = np.cross(old_axis, new_axis)
        sine = float(np.linalg.norm(cross))
        return np.zeros(3) if sine < 1e-12 else cross / sine * math.atan2(sine, float(np.dot(old_axis, new_axis)))
    inverse = previous * np.array([-1.0, -1.0, -1.0, 1.0])
    delta = _multiply(current, inverse)
    if delta[3] < 0:
        delta = -delta
    magnitude = float(np.linalg.norm(delta[:3]))
    return np.zeros(3) if magnitude < 1e-12 else delta[:3] / magnitude * (2 * math.atan2(magnitude, delta[3]))


@dataclass(frozen=True)
class TargetEstimate:
    """Target estimate derived exclusively from accepted visual observations."""

    stamp: float
    position: tuple
    orientation: tuple
    linear_velocity: tuple
    angular_velocity: tuple
    stationary: bool


class MotionTracker:
    """Track visual motion and require a full continuous stationary window."""

    def __init__(self, shape, window_sec=1.0, max_gap_sec=0.3, linear_limit=0.005,
                 angular_limit=math.radians(3), position_limit=0.005,
                 orientation_limit=math.radians(3), velocity_alpha=0.5):
        """Configure the motion estimator and stopping criteria.

        Args:
            shape: Object geometry, box or cylinder.
            window_sec: Required duration of continuous stationary observations.
            max_gap_sec: Maximum interval between accepted observations.
            linear_limit: Maximum filtered linear speed in meters per second.
            angular_limit: Maximum observable angular speed in radians per second.
            position_limit: Maximum displacement within the window in meters.
            orientation_limit: Maximum observable rotation within the window in radians.
            velocity_alpha: Exponential velocity filter weight for the new sample.

        Returns:
            None.

        Raises:
            ValueError: If a configuration value is invalid.
        """
        if shape not in ("box", "cylinder"):
            raise ValueError("unsupported target shape")
        limits = (window_sec, max_gap_sec, linear_limit, angular_limit, position_limit, orientation_limit)
        if not all(math.isfinite(x) and x > 0 for x in limits) or not 0 < velocity_alpha <= 1:
            raise ValueError("invalid tracking limits")
        self.shape, self.window_sec, self.max_gap_sec = shape, window_sec, max_gap_sec
        self.linear_limit, self.angular_limit = linear_limit, angular_limit
        self.position_limit, self.orientation_limit = position_limit, orientation_limit
        self.alpha = velocity_alpha
        self.reset()

    def reset(self):
        """Discard all observations and stationary evidence.

        Returns:
            None.
        """
        self.samples = deque()
        self.last = None
        self.linear = np.zeros(3)
        self.angular = np.zeros(3)

    def update(self, stamp, position, orientation):
        """Consume a fresh, complete visual observation.

        Args:
            stamp: Capture timestamp in seconds on the ROS clock.
            position: Target center in the fixed observation frame in meters.
            orientation: Target orientation in xyzw order.

        Returns:
            A TargetEstimate. The first observation cannot establish stationarity.

        Raises:
            ValueError: If data are invalid or the timestamp does not advance.
        """
        position = np.asarray(position, dtype=float)
        if not math.isfinite(stamp) or position.shape != (3,) or not np.all(np.isfinite(position)):
            self.reset()
            raise ValueError("invalid observation")
        try:
            orientation = _unit_quaternion(orientation)
        except ValueError:
            self.reset()
            raise
        if self.last is not None:
            dt = stamp - self.last[0]
            if dt <= 0:
                self.reset()
                raise ValueError("observation timestamp did not advance")
            if dt > self.max_gap_sec:
                self.reset()
            else:
                orientation = equivalent_orientation(orientation, self.last[2], self.shape)
                velocity = (position - self.last[1]) / dt
                angular = _rotation_delta(orientation, self.last[2], self.shape) / dt
                self.linear = self.alpha * velocity + (1 - self.alpha) * self.linear
                self.angular = self.alpha * angular + (1 - self.alpha) * self.angular
        self.last = (stamp, position.copy(), orientation.copy())
        self.samples.append((stamp, position.copy(), orientation.copy(), self.linear.copy(), self.angular.copy()))
        cutoff = stamp - self.window_sec
        while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        stationary = False
        if len(self.samples) >= 2 and self.samples[0][0] <= cutoff:
            anchor = self.samples[0]
            stationary = all(
                np.linalg.norm(sample[3]) < self.linear_limit
                and np.linalg.norm(sample[4]) < self.angular_limit
                and np.linalg.norm(sample[1] - anchor[1]) < self.position_limit
                and np.linalg.norm(_rotation_delta(sample[2], anchor[2], self.shape)) < self.orientation_limit
                for sample in self.samples)
        return TargetEstimate(stamp, tuple(position), tuple(orientation), tuple(self.linear),
                              tuple(self.angular), bool(stationary))
