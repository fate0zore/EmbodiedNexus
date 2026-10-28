from __future__ import annotations

import copy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from embodied_nexus_camera.config import DEFAULT_CONFIG, load_config, validate_config
from embodied_nexus_camera.model_registry import resolve_model


class ModelRegistrationTests(unittest.TestCase):
    """Exercise model selection without an additional production device."""

    def test_unknown_backend_is_rejected_by_model_adapter(self):
        """Reject a source backend that Gemini335 does not implement.

        Returns:
            None.
        """
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["source"]["backend"] = "unsupported_backend"
        with self.assertRaisesRegex(ValueError, "unsupported camera backend"):
            resolve_model(config["model"]).start_capture(config, None, None, None, None, None, None)

    def test_registered_model_owns_defaults_and_validation(self):
        """Load independent instances from a model provider; reject unknown models.

        Returns:
            None.
        """
        defaults = copy.deepcopy(DEFAULT_CONFIG)
        defaults["model"] = "test_model"
        provider = MagicMock()
        provider.default_config.side_effect = lambda: copy.deepcopy(defaults)
        entry = MagicMock()
        entry.load.return_value = lambda: provider
        entries = MagicMock()
        entries.select.side_effect = lambda **kwargs: [entry] if kwargs["name"] == "test_model" else []
        with patch("embodied_nexus_camera.model_registry.entry_points", return_value=entries):
            with tempfile.TemporaryDirectory() as directory:
                first_path = Path(directory) / "first.yaml"
                second_path = Path(directory) / "second.yaml"
                first_path.write_text("model: test_model\ncamera_id: first\n", encoding="utf-8")
                second_path.write_text("model: test_model\ncamera_id: second\n", encoding="utf-8")
                first = load_config(first_path)
                second = load_config(second_path)
            self.assertEqual(first["camera_id"], "first")
            self.assertEqual(second["camera_id"], "second")
            self.assertIsNot(first, second)
            self.assertGreaterEqual(provider.validate_config.call_count, 2)
            self.assertIs(resolve_model("test_model"), provider)
            first["model"] = "unknown"
            with self.assertRaises(ValueError):
                validate_config(first)
