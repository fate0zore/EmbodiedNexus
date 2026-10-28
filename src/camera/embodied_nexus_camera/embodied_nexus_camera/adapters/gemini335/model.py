"""Gemini335 model provider."""

import copy

class Gemini335Model:
    """Provide Gemini335 configuration validation and capture."""

    def default_config(self):
        """Return independent model defaults.

        Returns:
            Camera configuration dictionary.
        """
        from .config import DEFAULT_CONFIG
        return copy.deepcopy(DEFAULT_CONFIG)

    def runtime_paths(self):
        """Return fields that may be reconfigured while running.

        Returns:
            Set of configuration paths.
        """
        from .config import _RECONFIGURABLE_PATHS
        return set(_RECONFIGURABLE_PATHS)

    def validate_config(self, config):
        """Validate the model-specific camera configuration.

        Args:
            config: Effective camera configuration.

        Returns:
            None.
        """
        from .config import validate_config
        validate_config(config)

    def start_capture(self, config, stop, hub, accept_color, accept_depth, logger, spawn):
        """Start the model's physical color and depth capture workers.

        Args:
            config: Effective camera configuration.
            stop: Shared capture stop event.
            hub: Stream hub receiving errors.
            accept_color: Color frame callback.
            accept_depth: Depth frame callback.
            logger: Capture logger.
            spawn: Function that starts and tracks a named worker.

        Returns:
            None.
        """
        if config["source"]["backend"] != "gemini_v4l2":
            raise ValueError(f"unsupported camera backend {config['source']['backend']!r}")
        from .capture import Gemini335Capture
        capture = Gemini335Capture(config, stop, hub, accept_color, accept_depth, logger)
        spawn(capture.capture_color, "camera-color-capture")
        spawn(capture.capture_depth, "camera-depth-capture")
