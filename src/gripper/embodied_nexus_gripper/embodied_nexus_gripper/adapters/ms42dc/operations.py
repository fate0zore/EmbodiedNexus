"""MS42DC motion and feedback semantics."""

from dataclasses import dataclass

from ...config import finite_number
from ...models import GripperFeedback


@dataclass(frozen=True)
class MotionPlan:
    """Validated MS42DC operation, including its command and reported target."""

    direction: int
    angle_tenths: int
    target_degrees: float


class MS42DCOperations:
    """Translate common gripper actions into the MS42DC motor protocol."""

    def normalize_feedback(self, feedback, config):
        """Convert an MS42DC frame into the common gripper feedback contract.

        Args:
            feedback: Raw MS42DC feedback, or already normalized simulator feedback.
            config: Effective gripper configuration.

        Returns:
            Model-neutral feedback values used by the shared controller.
        """
        if isinstance(feedback, GripperFeedback):
            return feedback
        moving_value = config["feedback"]["moving_value"]
        reached_value = config["feedback"]["reached_value"]
        moving = None if moving_value is None else feedback.reached == int(moving_value)
        reached = None if reached_value is None else feedback.reached == int(reached_value)
        stationary = (feedback.speed_raw == 0 and reached is True) if reached is not None else None
        return GripperFeedback(moving=moving, reached=reached, stationary=stationary)

    def reconnect_delay(self, config):
        """Return the reconnect interval for this model.

        Args:
            config: Effective gripper configuration.

        Returns:
            Reconnect interval in seconds.
        """
        return float(config["serial"]["reconnect_backoff_sec"])

    def feedback_configured(self, config):
        """Check that completion feedback semantics have been confirmed.

        Args:
            config: Effective gripper configuration.

        Returns:
            Whether completion feedback can be interpreted safely.
        """
        return bool(config["feedback"]["confirmed"] and config["feedback"]["reached_value"] is not None)

    def completion_allowed(self, config, moving_seen):
        """Check the configured transition requirement.

        Args:
            config: Effective gripper configuration.
            moving_seen: Whether a moving sample has been observed.

        Returns:
            Whether a completion sample can terminate the operation.
        """
        return not bool(config["feedback"]["require_moving_transition"]) or moving_seen

    def descriptor(self, config, backend):
        """Describe this model's supported actions and measured feedback.

        Args:
            config: Effective gripper configuration.
            backend: Selected physical or simulated backend.

        Returns:
            Capability and feedback descriptors.
        """
        maximum = int(config["motion"]["max_relative_angle_tenths"]) / 10.0
        simulated = bool(getattr(backend, "feedback_is_simulated", False))
        return {
            "capabilities": [
                {"name": "open", "parameters": [], "units": None},
                {"name": "close", "parameters": [], "units": None},
                {"name": "move_relative", "parameters": [{
                    "name": "angle_degrees", "unit": "deg", "min": -maximum, "max": maximum,
                }], "units": None},
                {"name": "stop", "parameters": [], "units": None},
                {"name": "enable_motion", "parameters": [], "units": None},
            ],
            "feedback": {
                "position_degrees": None,
                "opening_width_m": None, "finger_position": None, "force_n": None,
                "completion_confirmed": bool(config["feedback"]["confirmed"]),
                "source": "simulated_joint_state" if simulated else "device_protocol",
            },
        }

    def feedback_fields(self, feedback):
        """Expose model-neutral measured state through the common API.

        Args:
            feedback: Normalized feedback, or None if unavailable.

        Returns:
            Public position-independent motion feedback fields.
        """
        return {
            "feedback_moving": bool(feedback and feedback.moving),
            "feedback_reached": bool(feedback and feedback.reached),
            "feedback_stationary": bool(feedback and feedback.stationary),
        }

    def plan(self, config, command, angle_degrees):
        """Validate and translate a requested action.

        Args:
            config: Validated gripper configuration.
            command: Open, close or relative action.
            angle_degrees: Requested relative motor angle in degrees.

        Returns:
            A model-specific motion plan.
        """
        command = str(command).strip().lower()
        if command not in {"open", "close", "relative"}:
            raise ValueError("command must be open, close, or relative")
        requested = finite_number(angle_degrees, "angle_degrees")
        if command == "open":
            direction = 0
            angle_raw = int(config["motion"]["open_angle_tenths"])
            target_degrees = -angle_raw / 10.0
        elif command == "close":
            direction = 1
            angle_raw = int(config["motion"]["close_angle_tenths"])
            target_degrees = angle_raw / 10.0
        else:
            if requested == 0:
                raise ValueError("relative angle_degrees must be nonzero")
            angle_raw = int(round(abs(requested) * 10.0))
            maximum = int(config["motion"]["max_relative_angle_tenths"])
            if angle_raw < 1 or angle_raw > maximum:
                raise ValueError(f"relative angle must be in [{-maximum / 10.0:g}, {maximum / 10.0:g}] degrees")
            direction = 1 if requested > 0 else 0
            target_degrees = requested
        if angle_raw < 1:
            raise ValueError(f"{command} angle is configured as zero")
        return MotionPlan(direction, angle_raw, target_degrees)

    def execute(self, backend, plan, config):
        """Send a validated plan through the selected backend.

        Args:
            backend: Device, mock or simulator backend.
            plan: Validated MS42DC motion plan.
            config: Effective gripper configuration.

        Returns:
            None.
        """
        backend.command(plan.direction, plan.angle_tenths, int(config["control"]["speed_tenths"]))

    def stopped(self, feedback, config):
        """Evaluate whether a fresh sample confirms a stationary motor.

        Args:
            feedback: Decoded motor feedback.
            config: Effective gripper configuration.

        Returns:
            Whether the sample confirms the configured stopped state.
        """
        return bool(feedback and feedback.stationary)

    def reached(self, feedback, config):
        """Evaluate a fresh completion sample.

        Args:
            feedback: Decoded motor feedback.
            config: Effective gripper configuration.

        Returns:
            Whether the configured completion value is present.
        """
        return bool(feedback and feedback.reached)

    def moving(self, feedback, config):
        """Evaluate a fresh moving sample.

        Args:
            feedback: Decoded motor feedback.
            config: Effective gripper configuration.

        Returns:
            Whether the configured moving value is present.
        """
        return bool(feedback and feedback.moving)
