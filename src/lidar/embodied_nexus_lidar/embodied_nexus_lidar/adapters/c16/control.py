"""C16 motor control translation and UDP transport."""

import socket
import time

from .protocol import build_motor_packet, build_speed_packet, inspect_difop


class C16Control:
    """Translate model-neutral motor requests to the C16 protocol."""

    speeds_hz = (5, 10, 20)

    def __init__(self):
        """Initialize the latest C16 status cache.

        Returns:
            None.
        """
        self._packet = b""
        self._received_at = 0.0

    def ingest_status(self, packet):
        """Validate and store a newly received C16 status packet.

        Args:
            packet: Raw status data.

        Returns:
            None.
        """
        raw = bytes(packet)
        self.inspect(raw)
        self._packet = raw[:1206]
        self._received_at = time.monotonic()

    def status_age(self):
        """Return the age of the last status packet.

        Returns:
            Age in seconds, or None before receiving a packet.
        """
        return time.monotonic() - self._received_at if self._received_at else None

    def fresh_status(self, config):
        """Return a fresh validated status packet for command construction.

        Args:
            config: Effective lidar configuration.

        Returns:
            Fresh status payload.
        """
        age = self.status_age()
        if age is None or age > float(config["control"]["difop_stale_sec"]):
            raise RuntimeError(f"fresh DIFOP data unavailable (age={age})")
        self.inspect(self._packet)
        return self._packet

    def supports_motor(self, config):
        """Check if the selected backend owns motor control.

        Args:
            config: Effective lidar configuration.

        Returns:
            Whether motor control is available.
        """
        return config["backend"] in {"c16", "mock"}

    def is_simulated(self, config):
        """Check whether control feedback is synthesized.

        Args:
            config: Effective lidar configuration.

        Returns:
            Whether the mock backend supplies simulated feedback.
        """
        return config["backend"] == "mock"

    def simulated_stop_error(self, config):
        """Return an injected mock stop error when configured.

        Args:
            config: Effective lidar configuration.

        Returns:
            Error message, or an empty string.
        """
        return "mock injected lidar stop failure" if config["mock"]["fail_stop"] else ""

    def simulated_disconnect(self, config):
        """Check whether the mock source has injected a disconnect.

        Args:
            config: Effective lidar configuration.

        Returns:
            Whether a mock disconnect is active.
        """
        return bool(config["mock"]["disconnect"])

    def default_frame(self, config):
        """Return the configured sensor frame.

        Args:
            config: Effective lidar configuration.

        Returns:
            Sensor coordinate-frame ID.
        """
        return str(config["driver"]["frame_id"])

    def connected(self, config, cloud_age):
        """Evaluate connection status for the selected backend.

        Args:
            config: Effective lidar configuration.
            cloud_age: Latest point-cloud age in seconds, or None.

        Returns:
            Whether the selected source is connected and fresh.
        """
        if self.is_simulated(config):
            return not config["mock"]["disconnect"]
        if config["backend"] == "c16":
            age = self.status_age()
            return age is not None and age <= float(config["control"]["difop_stale_sec"])
        return cloud_age is not None and cloud_age <= float(config["streams"]["stale_sec"])

    def inspect(self, packet):
        """Validate a C16 status packet and decode the observed motor speed.

        Args:
            packet: Raw DIFOP data.

        Returns:
            Decoded C16 status.
        """
        return inspect_difop(packet)

    def motor_command(self, packet, enabled, speed_hz=None):
        """Construct a C16 motor start or stop request.

        Args:
            packet: Fresh DIFOP data.
            enabled: Whether to start the motor.
            speed_hz: Optional requested speed in Hz.

        Returns:
            Encoded UDP payload.
        """
        return build_motor_packet(packet, enabled=enabled, speed_hz=speed_hz)

    def speed_command(self, packet, speed_hz):
        """Construct a motor speed request.

        Args:
            packet: Fresh DIFOP data.
            speed_hz: Requested speed in Hz.

        Returns:
            Encoded UDP payload.
        """
        return build_speed_packet(packet, speed_hz)

    def send(self, packet, config):
        """Send one C16 command to its configured endpoint.

        Args:
            packet: Encoded UDP payload.
            config: Effective configuration containing backend and endpoint.

        Returns:
            None.
        """
        if self.is_simulated(config):
            if config["mock"]["fail_command"]:
                raise OSError("mock injected lidar command failure")
            return
        if not self.supports_motor(config):
            raise RuntimeError("motor control is unsupported by backend=ros2_topics")
        driver = config["driver"]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sent = sock.sendto(packet, (driver["device_ip"], int(driver["command_port"])))
        if sent != len(packet):
            raise OSError(f"short lidar UDP send: {sent}/{len(packet)} bytes")

    def mock_packet(self):
        """Provide a valid simulated C16 status packet.

        Returns:
            A synthetic DIFOP packet.
        """
        packet = bytearray(1206)
        packet[:4] = b"\xA5\xFF\x00\x5A"
        packet[8:10] = b"\x02\x58"
        packet[1198] = 0x04
        return bytes(packet)
