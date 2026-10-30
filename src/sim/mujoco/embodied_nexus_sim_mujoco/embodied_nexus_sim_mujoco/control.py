"""Simulator-independent command helpers for the MuJoCo adapter."""

from dataclasses import dataclass
import math


WHEEL_RADIUS = 0.16459
WHEEL_SEPARATION = 0.58306
COMMAND_TIMEOUT = 0.5


def wheel_speeds(linear, angular):
    """Map a chassis twist to left and right wheel angular velocities.

    Args:
        linear: Forward chassis velocity in meters per second.
        angular: Yaw velocity in radians per second.

    Returns:
        Tuple of left and right angular velocities in radians per second.
    """
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise ValueError("velocity commands must be finite")
    return ((linear - angular * WHEEL_SEPARATION / 2) / WHEEL_RADIUS,
            (linear + angular * WHEEL_SEPARATION / 2) / WHEEL_RADIUS)


@dataclass
class Trajectory:
    """One measured-position-to-target joint interpolation."""

    start: tuple[float, ...]
    target: tuple[float, ...]
    start_time: float
    duration: float

    def sample(self, now):
        """Interpolate the command at simulation time.

        Args:
            now: Current simulation time in seconds.

        Returns:
            Tuple of joint position targets.
        """
        fraction = min(1.0, max(0.0, (now - self.start_time) / self.duration))
        return tuple(a + fraction * (b - a) for a, b in zip(self.start, self.target))


def make_trajectory(names, positions, expected_names, measured, now, duration):
    """Validate a single-point trajectory and capture measured start positions.

    Args:
        names: Names supplied by the command.
        positions: Position targets corresponding to names.
        expected_names: Complete ordered joint names for the controller.
        measured: Current joint positions in expected order.
        now: Current simulation time in seconds.
        duration: Target arrival time in seconds.

    Returns:
        A validated trajectory with targets ordered for the controller.
    """
    if len(names) != len(expected_names) or len(set(names)) != len(names) or set(names) != set(expected_names):
        raise ValueError("trajectory must name each controller joint exactly once")
    if len(positions) != len(names) or len(measured) != len(expected_names):
        raise ValueError("trajectory positions do not match joint names")
    if not math.isfinite(duration) or duration <= 0 or not all(math.isfinite(v) for v in positions):
        raise ValueError("trajectory duration and positions must be finite")
    mapping = dict(zip(names, positions))
    return Trajectory(tuple(measured), tuple(mapping[name] for name in expected_names), now, duration)
