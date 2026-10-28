"""Aubo i5 model provider for the common arm service."""

from importlib.metadata import entry_points

from .backend import AuboBackend
from .operations import AuboOperations


class AuboI5Model:
    """Construct Aubo adapters for physical, mock and ROS simulation backends."""

    def validate_config(self, config):
        """Validate the model's configuration.

        Args:
            config: Effective arm configuration.

        Returns:
            None.
        """
        from .config import validate_config
        validate_config(config)

    def create_backend(self, config):
        """Create the requested Aubo backend.

        Args:
            config: Effective arm configuration.

        Returns:
            Physical, mock or simulation backend.
        """
        if config["backend"] == "mock":
            from .mock import MockBackend
            return MockBackend(config)
        if config["backend"] == "aubo_jsonrpc":
            return AuboBackend(config)
        if config["backend"] == "ros2_sim":
            providers = list(entry_points().select(group="embodied_nexus_arm.backends", name="aubo_i5.ros2_sim"))
            if len(providers) != 1:
                raise RuntimeError("backend=ros2_sim requires exactly one embodied_nexus_sim provider")
            return providers[0].load()(config)
        raise ValueError(f"unsupported arm backend {config['backend']!r}")

    def create_operations(self, backend):
        """Create the model's semantic operation adapter.

        Args:
            backend: Physical, mock or simulator backend.

        Returns:
            Aubo semantic operations.
        """
        return AuboOperations(backend)
