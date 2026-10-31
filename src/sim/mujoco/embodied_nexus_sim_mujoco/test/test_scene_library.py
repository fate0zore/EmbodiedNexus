"""Tests for scene YAML storage and validation rules."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


class SceneLibraryTests(unittest.TestCase):
    """Exercise safe scene file and mesh asset handling without ROS startup."""

    def make_library(self, root):
        """Construct a scene library shell for isolated file-management tests.

        Args:
            root: Temporary scene directory.

        Returns:
            A SceneLibrary instance with filesystem paths initialized.
        """
        try:
            from embodied_nexus_sim_mujoco.scene import SceneLibrary
        except ImportError as exc:
            self.skipTest(f"MuJoCo scene dependencies are unavailable: {exc}")
        library = object.__new__(SceneLibrary)
        library.directory = Path(root)
        library.assets_directory = Path(root) / "assets"
        library.directory.mkdir(parents=True, exist_ok=True)
        library.assets_directory.mkdir(parents=True, exist_ok=True)
        return library

    def scene(self, *, objects=None):
        """Return a minimal valid scene mapping for file-management tests.

        Args:
            objects: Optional scene object list.

        Returns:
            A valid YAML-compatible scene mapping.
        """
        return {"name": "Test", "ground": {"size": [10, 10, 0.1]},
                "objects": objects or [{"id": "box", "shape": "box", "position": [0, 0, 0.5],
                                        "rotation": [0, 0, 0], "size": [1, 1, 1],
                                        "rgba": [0.2, 0.3, 0.4, 1], "motion": "static"}]}

    def test_upload_asset_and_scene_relative_mesh_validation(self):
        """Store a mesh below its scene asset directory and reject traversal paths.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as temporary:
            library = self.make_library(temporary)
            result = library.upload_asset("test", "part.obj", b"v 0 0 0\n")
            self.assertEqual(result["path"], "assets/test/part.obj")
            mesh = {"id": "part", "shape": "mesh", "position": [0, 0, 0], "rotation": [0, 0, 0],
                    "rgba": [1, 1, 1, 1], "motion": "static", "mesh": result["path"]}
            library._validate_mapping("test", self.scene(objects=[mesh]))
            mesh["mesh"] = "assets/test/../../outside.obj"
            with self.assertRaisesRegex(ValueError, "assets/test"):
                library._validate_mapping("test", self.scene(objects=[mesh]))

    def test_invalid_scene_does_not_replace_existing_yaml(self):
        """Keep the previous scene file when validation rejects a candidate.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as temporary:
            library = self.make_library(temporary)
            target = Path(temporary) / "test.yaml"
            target.write_text("name: Existing\nobjects: []\n", encoding="utf-8")
            with patch.object(library, "validate_scene", side_effect=ValueError("invalid candidate")):
                with self.assertRaisesRegex(ValueError, "invalid candidate"):
                    library.save_scene("test", self.scene())
            self.assertIn("Existing", target.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
