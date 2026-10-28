"""Scout 2.0 model provider."""

import copy
from importlib.metadata import entry_points

from .backend import ScoutWaveshareSerialBackend
from .policy import Scout2Policy


class Scout2Model:
    """Select Scout 2.0 configuration, backend and feedback policy."""

    def default_config(self):
        """Return the model's default configuration.

        Returns:
            Independent configuration dictionary.
        """
        from .config import DEFAULT_CONFIG
        return copy.deepcopy(DEFAULT_CONFIG)

    def runtime_paths(self):
        """Return fields that can be changed while the service is running.

        Returns:
            Set of configuration paths.
        """
        from .config import RUNTIME_PATHS
        return set(RUNTIME_PATHS)

    def validate_config(self, config):
        """Validate this model's configuration.

        Args:
            config: Effective device configuration.

        Returns:
            None.
        """
        from .config import validate_config
        validate_config(config)

    def create_backend(self, config, serial_factory=None):
        """Construct the requested backend.

        Args:
            config: Effective device configuration.
            serial_factory: Optional serial factory for tests.

        Returns:
            Model backend.
        """
        if config["backend"] == "mock":
            from .mock import MockBackend
            return MockBackend(config)
        if config["backend"] == "scout_waveshare_serial":
            return ScoutWaveshareSerialBackend(config, serial_factory=serial_factory)
        if config["backend"] == "ros2_sim":
            providers = list(entry_points().select(group="embodied_nexus_base.backends", name="scout_2.ros2_sim"))
            if len(providers) != 1:
                raise RuntimeError("backend=ros2_sim requires exactly one embodied_nexus_sim provider")
            return providers[0].load()(config)
        raise ValueError(f"unsupported base backend {config['backend']!r}")

    def create_policy(self):
        """Create the model's feedback policy.

        Returns:
            Scout feedback policy.
        """
        return Scout2Policy()
