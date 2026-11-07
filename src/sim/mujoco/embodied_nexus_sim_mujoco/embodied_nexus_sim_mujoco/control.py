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
    """Piecewise-linear joint trajectory from measured state through waypoints."""

    start: tuple[float, ...]
    waypoints: tuple[tuple[float, ...], ...]
    times: tuple[float, ...]
    start_time: float

    def sample(self, now):
        """Interpolate the active trajectory segment at simulation time.

        Args:
            now: Current simulation time in seconds.

        Returns:
            Tuple of joint position targets.
        """
        elapsed = max(0.0, float(now) - self.start_time)
        previous_time = 0.0
        previous = self.start
        for waypoint, waypoint_time in zip(self.waypoints, self.times):
            if elapsed <= waypoint_time:
                fraction = min(1.0, max(0.0, (elapsed - previous_time) / (waypoint_time - previous_time)))
                return tuple(a + fraction * (b - a) for a, b in zip(previous, waypoint))
            previous_time = waypoint_time
            previous = waypoint
        return self.waypoints[-1]

    @property
    def duration(self):
        """Return the final waypoint time in seconds."""
        return self.times[-1]


def make_trajectory(names, positions, expected_names, measured, now, duration):
    """Validate a trajectory and capture measured start positions.

    Args:
        names: Names supplied by the command.
        positions: One position vector or a sequence of position vectors.
        expected_names: Complete ordered joint names for the controller.
        measured: Current joint positions in expected order.
        now: Current simulation time in seconds.
        duration: One duration or increasing waypoint arrival times in seconds.

    Returns:
        A validated trajectory with waypoint positions ordered for the controller.
    """
    if len(names) != len(expected_names) or len(set(names)) != len(names) or set(names) != set(expected_names):
        raise ValueError("trajectory must name each controller joint exactly once")
    if len(measured) != len(expected_names):
        raise ValueError("measured positions do not match controller joints")
    if positions and isinstance(positions[0], (int, float)):
        position_rows = [positions]
    else:
        position_rows = list(positions)
    times = [float(duration)] if isinstance(duration, (int, float)) else [float(value) for value in duration]
    if not position_rows or len(position_rows) != len(times):
        raise ValueError("trajectory waypoint positions and times must be non-empty and have equal lengths")
    if any(len(row) != len(names) or not all(math.isfinite(float(v)) for v in row) for row in position_rows):
        raise ValueError("trajectory positions do not match joint names or contain non-finite values")
    if not all(math.isfinite(value) and value > 0 for value in times):
        raise ValueError("trajectory waypoint times must be finite and positive")
    if any(later <= earlier for earlier, later in zip(times, times[1:])):
        raise ValueError("trajectory waypoint times must be strictly increasing")
    mapping = dict(zip(names, positions))
    ordered = tuple(tuple(dict(zip(names, row))[name] for name in expected_names) for row in position_rows)
    return Trajectory(tuple(measured), ordered, tuple(times), now)
