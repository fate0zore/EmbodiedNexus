"""Tests for scene YAML storage and validation rules."""

import tempfile
import threading
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
        library._operation_lock = threading.RLock()
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

    def test_scene_validates_optional_arm_position_actuator_parameters(self):
        """Accept positive scene-specific arm gains and reject invalid values.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as temporary:
            library = self.make_library(temporary)
            scene = self.scene()
            scene["arm_position_actuator"] = {"kp": 1500.0, "kv": 80.0, "max_force": 500.0}
            library._validate_mapping("test", scene)
            scene["arm_position_actuator"]["kp"] = 0.0
            with self.assertRaisesRegex(ValueError, "arm_position_actuator.kp"):
                library._validate_mapping("test", scene)

    def test_ground_and_object_friction_validation(self):
        """Accept explicit contact friction and reject invalid coefficients.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as temporary:
            library = self.make_library(temporary)
            scene = self.scene()
            scene['ground']['friction'] = [0.003, 0.0001, 0.00001]
            scene['objects'][0]['friction'] = [0.003, 0.0001, 0.00001]
            library._validate_mapping('test', scene)
            scene['objects'][0]['friction'] = [-1, 0.01, 0.01]
            with self.assertRaisesRegex(ValueError, 'friction'):
                library._validate_mapping('test', scene)

    def test_fixed_base_and_friction_solver_options_are_validated(self):
        """Accept a stationary scene and reject malformed solver settings.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as temporary:
            library = self.make_library(temporary)
            scene = self.scene()
            scene.update(fixed_base=True, noslip_iterations=10)
            library._validate_mapping("test", scene)
            scene["fixed_base"] = "true"
            with self.assertRaisesRegex(ValueError, "fixed_base"):
                library._validate_mapping("test", scene)
            scene["fixed_base"] = True
            scene["noslip_iterations"] = -1
            with self.assertRaisesRegex(ValueError, "noslip_iterations"):
                library._validate_mapping("test", scene)

    def test_scene_validates_finger_collision_boxes(self):
        """Accept valid link-local boxes and reject nonpositive dimensions.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as temporary:
            library = self.make_library(temporary)
            scene = self.scene()
            scene["finger_contact_boxes"] = {"left": [{
                "position": [0, 0, 0], "rotation": [0, 0, 0.38],
                "half_size": [0.01, 0.03, 0.008]}]}
            library._validate_mapping("test", scene)
            scene["finger_contact_boxes"]["left"][0]["half_size"][0] = 0
            with self.assertRaisesRegex(ValueError, "half_size"):
                library._validate_mapping("test", scene)

    def test_scene_validates_finger_contact_material(self):
        """Reject invalid friction and solver settings before a scene is saved.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as temporary:
            library = self.make_library(temporary)
            scene = self.scene()
            material = {"friction": [2.0, 0.02, 0.001], "condim": 4,
                        "solref": [0.03, 1.0], "solimp": [0.9, 0.95, 0.002, 0.5, 2.0]}
            scene["finger_contact_material"] = material
            library._validate_mapping("test", scene)
            for key, invalid in (("friction", [-1, 0, 0]), ("condim", 2),
                                 ("solref", [0, 1]), ("solimp", [0.95, 0.9, 0.002, 0.5, 2])):
                with self.subTest(key=key):
                    original = material[key]
                    material[key] = invalid
                    with self.assertRaisesRegex(ValueError, "finger_contact_material"):
                        library._validate_mapping("test", scene)
                    material[key] = original

    def test_scene_validates_contact_cone(self):
        """Accept supported friction cones and reject unrecognized values.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as temporary:
            library = self.make_library(temporary)
            scene = self.scene()
            scene["contact_cone"] = "elliptic"
            library._validate_mapping("test", scene)
            scene["contact_cone"] = "invalid"
            with self.assertRaisesRegex(ValueError, "contact_cone"):
                library._validate_mapping("test", scene)

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
