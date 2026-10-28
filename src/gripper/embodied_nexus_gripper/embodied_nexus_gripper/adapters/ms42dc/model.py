"""MS42DC model provider."""

import copy
from importlib.metadata import entry_points

from .backend import MS42DCSerialBackend
from .operations import MS42DCOperations


class MS42DCModel:
    """Select MS42DC configuration, backend and operation adapter."""

    def default_config(self):
        """Return independent model defaults.

        Returns:
            Configuration dictionary.
        """
        from .config import DEFAULT_CONFIG
        return copy.deepcopy(DEFAULT_CONFIG)

    def runtime_paths(self):
        """Return fields that may change without restarting.

        Returns:
            Set of configuration paths.
        """
        from .config import _RUNTIME_PATHS
        return set(_RUNTIME_PATHS)

    def validate_config(self, config):
        """Validate model configuration.

        Args:
            config: Effective device configuration.

        Returns:
            None.
        """
        from .config import validate_config
        validate_config(config)

    def create_backend(self, config):
        """Create the configured device or simulator backend.

        Args:
            config: Effective device configuration.

        Returns:
            Model backend.
        """
        if config["backend"] == "mock":
            from .mock import MockBackend
            return MockBackend(config)
        if config["backend"] == "serial":
            return MS42DCSerialBackend(config)
        if config["backend"] == "ros2_sim":
            providers = list(entry_points().select(group="embodied_nexus_gripper.backends", name="ms42dc.ros2_sim"))
            if len(providers) != 1:
                raise RuntimeError("backend=ros2_sim requires exactly one embodied_nexus_sim provider")
            return providers[0].load()(config)
        raise ValueError(f"unsupported gripper backend {config['backend']!r}")

    def create_operations(self):
        """Create model-specific gripper operations.

        Returns:
            MS42DC operation adapter.
        """
        return MS42DCOperations()
