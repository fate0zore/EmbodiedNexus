"""C16 model provider."""

import copy

from .control import C16Control


class C16Model:
    """Provide C16 configuration, control and ROS driver integration."""

    def default_config(self):
        """Return independent configuration defaults.

        Returns:
            Lidar configuration dictionary.
        """
        from .config import DEFAULT_CONFIG
        return copy.deepcopy(DEFAULT_CONFIG)

    def runtime_paths(self):
        """Return fields that can change while running.

        Returns:
            Set of configuration paths.
        """
        from .config import RUNTIME_PATHS
        return set(RUNTIME_PATHS)

    def validate_config(self, config):
        """Validate this model's configuration.

        Args:
            config: Effective lidar configuration.

        Returns:
            None.
        """
        from .config import validate_config
        validate_config(config)

    def create_control(self):
        """Create the model's motor control adapter.

        Returns:
            C16 control adapter.
        """
        return C16Control()

    def uses_driver(self, config):
        """Check whether this configuration launches the physical driver.

        Args:
            config: Effective lidar configuration.

        Returns:
            Whether the C16 driver is required.
        """
        return config["backend"] == "c16"

    def driver_node(self, config, namespace):
        """Construct the physical driver launch action when required.

        Args:
            config: Effective lidar configuration.
            namespace: Device ROS namespace.

        Returns:
            Driver launch node, or None for a non-hardware backend.
        """
        if not self.uses_driver(config):
            return None
        from .launch import create_driver_node
        return create_driver_node(config, namespace)

    def packet_type(self):
        """Resolve the physical driver's status packet message type.

        Returns:
            ROS message class for a C16 configuration packet.
        """
        from lslidar_msgs.msg import LslidarPacket
        return LslidarPacket

    def status_topic(self, namespace):
        """Return the physical driver's status topic.

        Args:
            namespace: Device ROS namespace.

        Returns:
            Status topic name.
        """
        return f"{namespace}/_driver/difop_packet"

    def status_payload(self, message):
        """Extract the C16 status payload from a driver message.

        Args:
            message: C16 driver packet message.

        Returns:
            Raw configuration packet bytes.
        """
        return bytes(message.data)
