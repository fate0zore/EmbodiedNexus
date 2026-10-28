"""Translate semantic arm operations to Aubo i5 backend commands."""


class AuboOperations:
    """Translate arm operations for the Aubo i5 and its simulated backends."""

    def __init__(self, backend):
        """Save the model backend.

        Args:
            backend: Backend supporting the Aubo command protocol.

        Returns:
            None.
        """
        self.backend = backend

    def command_accepted(self, result):
        """Interpret a model command result as accepted or rejected.

        Args:
            result: Result returned by an Aubo controller command.

        Returns:
            Whether the controller accepted the command.
        """
        return result in (0, None)

    def stream_result_supported(self, result):
        """Check whether a streaming result is accepted or asks for retry.

        Args:
            result: Result returned by an Aubo streaming command.

        Returns:
            Whether the result is a recognized streaming outcome.
        """
        return self.command_accepted(result) or result == 2

    def stream_retry_next_cycle(self, result):
        """Interpret the model-specific streaming retry outcome.

        Args:
            result: Result returned by an Aubo streaming command.

        Returns:
            Whether the controller requests a retry on the next cycle.
        """
        return result == 2

    def command_error(self, operation, result):
        """Describe a rejected command using the model's result convention.

        Args:
            operation: Common operation name.
            result: Result returned by an Aubo controller command.

        Returns:
            Model-specific diagnostic text.
        """
        names = {"streaming": "servo mode", "stream_target": "servoJoint",
                 "move_joints": "moveJoint", "lifecycle": "lifecycle command",
                 "teach": "teach mode"}
        return f"Aubo {names.get(operation, operation)} rejected command: result={result}"

    def optional_metadata(self):
        """Read tool payload and TCP offset when available.

        Returns:
            A pair containing payload and TCP offset, either of which may be None.
        """
        return (self.backend.read_optional("RobotConfig.getPayload"),
                self.backend.read_optional("RobotConfig.getTcpOffset"))

    def motion_permitted(self, state):
        """Evaluate Aubo mode and safety feedback.

        Args:
            state: Current common arm state.

        Returns:
            Whether the reported mode and safety state permit motion.
        """
        return (str(state.get("mode", "")).lower() == "running" and
                str(state.get("safety_mode", "")).lower() in {"normal", "reducedmode"})

    def velocity_step_seconds(self, config):
        """Return the configured joint streaming integration step.

        Args:
            config: Effective device configuration.

        Returns:
            Step duration in seconds.
        """
        return float(config["motion"]["servo_joint_command_period_sec"])

    def set_streaming(self, enabled):
        """Enable or disable streaming mode.

        Args:
            enabled: Whether streaming should be enabled.

        Returns:
            Controller command result.
        """
        return self.backend.execute("MotionControl.setServoMode", [enabled])

    def streaming_enabled(self):
        """Read streaming-mode status.

        Returns:
            Controller streaming-mode status.
        """
        return self.backend.execute("MotionControl.isServoModeEnabled")

    def stream_target(self, target, motion):
        """Send a joint streaming target using model parameters.

        Args:
            target: Joint position target in radians.
            motion: Motion configuration containing streaming parameters.

        Returns:
            Controller command result.
        """
        return self.backend.execute("MotionControl.servoJoint", [target,
            float(motion["servo_joint_velocity_rad_sec"]),
            float(motion["servo_joint_acceleration_rad_sec2"]),
            float(motion["servo_joint_period_sec"]),
            float(motion["servo_joint_lookahead_sec"]), int(motion["servo_joint_gain"])])

    def prepare_move(self):
        """Select joint move mode before a position operation.

        Returns:
            Controller command result.
        """
        return self.backend.execute("MotionControl.setServoModeSelect", [0])

    def move_joints(self, target, acceleration, speed):
        """Send a joint position operation.

        Args:
            target: Joint positions in radians.
            acceleration: Joint acceleration limit.
            speed: Joint speed limit.

        Returns:
            Controller command result.
        """
        return self.backend.execute("MotionControl.moveJoint", [target, acceleration, speed, 0.0, 0.0])

    def lifecycle(self, command):
        """Send a power or startup command.

        Args:
            command: One of power_on, startup or power_off.

        Returns:
            Controller command result.
        """
        if command == "power_on":
            return self.backend.execute("RobotManage.poweron")
        if command == "startup":
            return self.backend.execute("RobotManage.startup")
        if command == "power_off":
            error = None
            for method in ("RobotManage.poweroff", "RobotManage.powerOff"):
                try:
                    return self.backend.execute(method)
                except Exception as exc:
                    error = exc
            raise RuntimeError(f"Aubo power_off RPC unavailable: {error}")
        raise ValueError(f"unsupported lifecycle command: {command}")

    def mode_matches(self, mode, phase):
        """Interpret an Aubo mode for a lifecycle phase.

        Args:
            mode: Latest controller mode value.
            phase: One of powered_off, powered_on or running.

        Returns:
            Whether the controller has reached the requested phase.
        """
        expected = {"powered_off": {"PowerOff", "Poweroff", "poweroff"},
                    "powered_on": {"Idle", "Running"}, "running": {"Running"}}
        return mode in expected[phase]

    def teach(self, config, enabled):
        """Set the configured teach mode.

        Args:
            config: Effective arm configuration.
            enabled: Whether teach mode is enabled.

        Returns:
            Controller command result.
        """
        method = config["teach"]["method"]
        if method == "handguide":
            return self.backend.execute("RobotManage.handguideMode", [[], []]) if enabled else self.backend.execute("RobotManage.exitHandguideMode")
        return self.backend.execute(f"RobotManage.{method}", [bool(enabled)])
