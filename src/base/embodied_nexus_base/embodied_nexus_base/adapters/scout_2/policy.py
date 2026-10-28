"""Scout 2.0 interpretation of system and control-mode feedback."""


class Scout2Policy:
    """Interpret Scout state fields without exposing their codes to control logic."""

    def state_fields(self, system):
        """Convert raw system feedback to the published fields.

        Args:
            system: Latest Scout system feedback, or None.

        Returns:
            Public system fields including hardware emergency-stop state.
        """
        if system is None:
            return {"battery_voltage": None, "hardware_estop": None,
                    "fault_active": None, "control_mode_confirmed": None}
        vehicle_state = int(system["vehicle_state"])
        battery = system.get("battery_voltage")
        return {"battery_voltage": float(battery) if battery is not None else None,
                "hardware_estop": vehicle_state == 1,
                "fault_active": int(system["error_code"]) != 0,
                "control_mode_confirmed": int(system["control_mode"]) == 1}

    def motion_permitted(self, state):
        """Evaluate whether fresh Scout feedback permits motion.

        Args:
            state: Public device state.

        Returns:
            Whether the reported vehicle state and errors permit motion.
        """
        return state["hardware_estop"] is False and state["fault_active"] is False

    def control_confirmed(self, state):
        """Evaluate whether Scout accepted remote control mode.

        Args:
            state: Public device state.

        Returns:
            Whether control mode is confirmed by fresh feedback.
        """
        return bool(state["system_feedback_valid"] and state["control_mode_confirmed"] is True)
