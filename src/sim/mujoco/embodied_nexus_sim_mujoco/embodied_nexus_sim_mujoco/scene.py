"""Manage named MuJoCo environment scenes and their mesh assets."""

from __future__ import annotations

import math
import re
import subprocess
import threading
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Any

import mujoco
import yaml

from .camera_calibration import apply_camera_calibration, load_camera_calibration
from .model import DEFAULT_SCENE, make_scene


_SCENE_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,63}$")
_OBJECT_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,47}$")
_ASSET_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
_SHAPES = {"box", "sphere", "cylinder", "mesh"}
_COMMON_FIELDS = {"id", "shape", "position", "rotation", "rgba", "motion"}
_ARM_JOINTS = {
    "aubo_shoulder_joint", "aubo_upperArm_joint", "aubo_foreArm_joint",
    "aubo_wrist1_joint", "aubo_wrist2_joint", "aubo_wrist3_joint",
}


def _package_share_directory(package: str) -> Path:
    """Resolve an installed ROS package share directory.

    Args:
        package: ROS package name registered in the ament index.

    Returns:
        Absolute package share directory path.
    """
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


class SceneLibrary:
    """Read, validate, and persist scenes for the Core browser editor."""

    def __init__(self, scene_directory: str | Path, ros_node: Any = None) -> None:
        """Create a scene library rooted at the supplied workspace directory.

        Args:
            scene_directory: Repository directory containing scene YAML files.
            ros_node: Optional ROS node used to call the live MuJoCo apply service.

        Returns:
            None.
        """
        if not str(scene_directory):
            scene_directory = (_package_share_directory("embodied_nexus_sim_mujoco")
                               / "config" / "scenes")
        self.directory = Path(scene_directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self._operation_lock = threading.RLock()
        self._apply_client = None
        if ros_node is not None:
            from embodied_nexus_sim_mujoco_interfaces.srv import ApplyScene
            self._apply_client = ros_node.create_client(
                ApplyScene, "/sim/arachne/mujoco/apply_scene")
        self.assets_directory = self.directory / "assets"
        self.assets_directory.mkdir(parents=True, exist_ok=True)
        self.description_directory = _package_share_directory("embodied_nexus_sim_description").resolve()
        camera_directory = _package_share_directory("embodied_nexus_camera").resolve()
        self.camera_calibration = load_camera_calibration(self.description_directory, camera_directory)
        source = self.description_directory / "urdf" / "arachne.urdf.xacro"
        result = subprocess.run(
            ["xacro", str(source), "with_mimic_joints:=false", "with_ros2_control:=false",
             "gazebo_drive_wheels:=false", "with_ee_camera:=true", "with_lidar:=true"],
            check=True, capture_output=True, text=True,
        )
        self.robot_description = apply_camera_calibration(result.stdout, self.camera_calibration)

    def list_scenes(self) -> list[dict[str, str]]:
        """Return the identifiers and display names of saved scenes.

        Args:
            None.

        Returns:
            Scene summaries sorted by identifier.
        """
        scenes = []
        for path in sorted(self.directory.glob("*.yaml")):
            try:
                data = yaml.safe_load(path.read_text(encoding="utf-8"))
            except (OSError, yaml.YAMLError):
                continue
            scenes.append({"id": path.stem, "name": str(data.get("name", path.stem))
                           if isinstance(data, dict) else path.stem})
        return scenes

    def get_scene(self, scene_id: str) -> dict[str, Any]:
        """Load a named scene from its YAML file.

        Args:
            scene_id: Stable scene filename identifier.

        Returns:
            The parsed scene mapping.
        """
        path = self._scene_path(scene_id)
        if not path.is_file():
            raise FileNotFoundError(f"scene not found: {scene_id}")
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"scene {scene_id!r} must contain a YAML mapping")
        self.validate_scene(scene_id, data)
        return data

    def validate_scene(self, scene_id: str, scene: Any) -> dict[str, Any]:
        """Validate a scene schema, referenced assets, and compiled MJCF model.

        Args:
            scene_id: Stable scene filename identifier.
            scene: Candidate scene mapping from the browser editor.

        Returns:
            A validation result containing validity and a summary.
        """
        self._validate_id(scene_id)
        self._validate_mapping(scene_id, scene)
        xml = make_scene(self.robot_description, self.description_directory,
                         self.camera_calibration, scene, self.directory)
        model = mujoco.MjModel.from_xml_string(xml)
        return {"valid": True, "body_count": model.nbody, "geom_count": model.ngeom}

    def save_scene(self, scene_id: str, scene: Any) -> dict[str, Any]:
        """Validate and atomically save a named scene as YAML.

        Args:
            scene_id: Stable scene filename identifier.
            scene: Scene mapping supplied by the editor.

        Returns:
            The saved scene summary.
        """
        with self._operation_lock:
            return self._save_scene_unlocked(scene_id, scene)

    def save_and_apply_scene(self, scene_id: str, scene: Any) -> dict[str, Any]:
        """Validate and save a scene, then ask the running MuJoCo node to apply it.

        Args:
            scene_id: Stable scene filename identifier.
            scene: Current scene draft supplied by the browser editor.

        Returns:
            The saved scene summary with application status and model counts.
        """
        with self._operation_lock:
            saved = self._save_scene_unlocked(scene_id, scene)
            try:
                applied = self.apply_scene(scene_id)
            except Exception as exc:
                return {**saved, "saved": True, "applied": False, "error": str(exc)}
            return {**saved, "saved": True, "applied": True, **applied}

    def apply_scene(self, scene_id: str) -> dict[str, Any]:
        """Request that the running MuJoCo node compile and activate a saved scene.

        Args:
            scene_id: Stable scene filename identifier already saved on disk.

        Returns:
            The compiled body and geometry counts and service message.
        """
        if self._apply_client is None:
            raise RuntimeError("MuJoCo live-apply service client is unavailable")
        from embodied_nexus_sim_mujoco_interfaces.srv import ApplyScene
        if not self._apply_client.wait_for_service(timeout_sec=2.0):
            raise TimeoutError("MuJoCo live-apply service is not available")
        request = ApplyScene.Request()
        request.scene_id = scene_id
        future = self._apply_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(120.0):
            raise TimeoutError("MuJoCo live-apply response timed out; runtime state is unknown")
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message or "MuJoCo rejected the scene")
        return {
            "scene_id": scene_id,
            "body_count": int(response.body_count),
            "geom_count": int(response.geom_count),
            "message": response.message,
        }

    def _save_scene_unlocked(self, scene_id: str, scene: Any) -> dict[str, Any]:
        """Validate and atomically write a scene while the operation lock is held.

        Args:
            scene_id: Stable scene filename identifier.
            scene: Scene mapping to validate and persist.

        Returns:
            The saved scene summary.
        """
        self.validate_scene(scene_id, scene)
        path = self._scene_path(scene_id)
        payload = yaml.safe_dump(scene, allow_unicode=True, sort_keys=False)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                "w", encoding="utf-8", newline="\n", dir=self.directory,
                prefix=f".{scene_id}.", suffix=".tmp", delete=False,
            ) as stream:
                temporary_path = Path(stream.name)
                stream.write(payload)
                stream.flush()
            temporary_path.replace(path)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()
        return {"id": scene_id, "name": str(scene["name"])}

    def delete_scene(self, scene_id: str) -> None:
        """Delete one scene YAML and its scene-specific uploaded assets.

        Args:
            scene_id: Stable scene filename identifier.

        Returns:
            None.
        """
        if scene_id == "default":
            raise ValueError("the default scene cannot be deleted")
        path = self._scene_path(scene_id)
        if not path.is_file():
            raise FileNotFoundError(f"scene not found: {scene_id}")
        path.unlink()
        asset_directory = self.assets_directory / scene_id
        if asset_directory.is_dir():
            for asset in asset_directory.iterdir():
                if asset.is_file() or asset.is_symlink():
                    asset.unlink()
            asset_directory.rmdir()

    def upload_asset(self, scene_id: str, filename: str, contents: bytes) -> dict[str, str]:
        """Store an STL or OBJ upload in the scene's managed asset directory.

        Args:
            scene_id: Stable scene filename identifier.
            filename: Uploaded asset basename.
            contents: Raw uploaded bytes.

        Returns:
            A scene-relative asset reference for YAML.
        """
        self._validate_id(scene_id)
        if not _ASSET_NAME.fullmatch(filename) or Path(filename).suffix.lower() not in {".stl", ".obj"}:
            raise ValueError("mesh uploads must use a safe .stl or .obj filename")
        if not contents:
            raise ValueError("mesh upload is empty")
        asset_directory = self.assets_directory / scene_id
        asset_directory.mkdir(parents=True, exist_ok=True)
        target = (asset_directory / filename).resolve()
        if not target.is_relative_to(self.assets_directory.resolve()):
            raise ValueError("mesh asset path escapes the scene asset directory")
        target.write_bytes(contents)
        return {"path": f"assets/{scene_id}/{filename}"}

    def read_asset(self, scene_id: str, filename: str) -> tuple[bytes, str]:
        """Read one scene-owned mesh asset for browser preview.

        Args:
            scene_id: Stable scene filename identifier.
            filename: Uploaded asset basename.

        Returns:
            The asset bytes and its filename extension.
        """
        self._validate_id(scene_id)
        if not _ASSET_NAME.fullmatch(filename) or Path(filename).suffix.lower() not in {".stl", ".obj"}:
            raise ValueError("invalid scene asset filename")
        target = (self.assets_directory / scene_id / filename).resolve(strict=True)
        if not target.is_relative_to(self.assets_directory.resolve()) or not target.is_file():
            raise FileNotFoundError(f"scene asset not found: {filename}")
        return target.read_bytes(), target.suffix.lower()

    def robot_model(self) -> str:
        """Return the expanded calibrated URDF used in the MuJoCo preview.

        Args:
            None.

        Returns:
            Expanded robot URDF XML text.
        """
        return self.robot_description

    def _scene_path(self, scene_id: str) -> Path:
        """Resolve a validated scene identifier to its YAML file path.

        Args:
            scene_id: Stable scene filename identifier.

        Returns:
            The scene YAML path contained by the library directory.
        """
        self._validate_id(scene_id)
        return self.directory / f"{scene_id}.yaml"

    @staticmethod
    def _validate_id(scene_id: str) -> None:
        """Reject scene identifiers that are not safe repository filenames.

        Args:
            scene_id: Candidate scene identifier.

        Returns:
            None; raises ValueError when the identifier is invalid.
        """
        if not isinstance(scene_id, str) or not _SCENE_ID.fullmatch(scene_id):
            raise ValueError("scene IDs must start with a letter and contain only letters, digits, '_' or '-'")

    def _validate_mapping(self, scene_id: str, scene: Any) -> None:
        """Validate scene values and ensure every asset stays in its scene folder.

        Args:
            scene_id: Stable scene filename identifier.
            scene: Candidate scene mapping.

        Returns:
            None; raises ValueError when any field is invalid.
        """
        if not isinstance(scene, dict) or set(scene) - {
                "name", "ground", "objects", "initial_arm_joint_positions_rad",
                "arm_position_actuator", "finger_contact_center_offset_m",
                "finger_contact_box_half_size_m", "finger_contact_boxes", "finger_contact_material",
                "contact_cone", "finger_flex_pads", "finger_collision_geometry"}:
            raise ValueError(
                "scene must contain only name, ground, objects, initial_arm_joint_positions_rad, "
                "arm_position_actuator, finger_contact_center_offset_m, finger_contact_box_half_size_m, "
                "finger_contact_boxes, finger_contact_material, contact_cone, finger_flex_pads, "
                "and finger_collision_geometry")
        if not isinstance(scene.get("name"), str) or not scene["name"].strip():
            raise ValueError("scene name must be a non-empty string")
        if scene.get("contact_cone", "pyramidal") not in ("pyramidal", "elliptic"):
            raise ValueError("contact_cone must be pyramidal or elliptic")
        if scene.get("finger_collision_geometry", "boxes") not in ("mesh", "boxes"):
            raise ValueError("finger_collision_geometry must be mesh or boxes")
        pads = scene.get("finger_flex_pads", {})
        if not isinstance(pads, dict) or set(pads) - {"enabled", "style", "size_m", "grid", "mass_kg", "young_pa", "poisson", "damping", "radius_m", "fin_ray_edge_time_constant_s", "fin_ray_out_of_plane_stiffness_n_m"}:
            raise ValueError("finger_flex_pads contains unsupported fields")
        if pads:
            if pads.get("style", "pad") not in ("pad", "fin_ray"):
                raise ValueError("finger_flex_pads.style must be pad or fin_ray")
            if not isinstance(pads.get("enabled"), bool):
                raise ValueError("finger_flex_pads.enabled must be boolean")
            self._positive_vector(pads.get("size_m"), 3, "finger_flex_pads.size_m")
            grid = pads.get("grid")
            if not isinstance(grid, list) or len(grid) != 3 or any(type(n) is not int or n < 2 for n in grid):
                raise ValueError("finger_flex_pads.grid must contain three integers >= 2")
            for key in ("mass_kg", "young_pa", "damping", "radius_m"):
                self._positive_number(pads.get(key), f"finger_flex_pads.{key}")
            for key in ("fin_ray_edge_time_constant_s", "fin_ray_out_of_plane_stiffness_n_m"):
                if key in pads:
                    self._positive_number(pads[key], f"finger_flex_pads.{key}")
            poisson = pads.get("poisson")
            if isinstance(poisson, bool) or not isinstance(poisson, (int, float)) or not 0 < poisson < 0.5:
                raise ValueError("finger_flex_pads.poisson must be between 0 and 0.5")
            if pads["enabled"] and any(not scene.get("finger_contact_boxes", {}).get(side)
                                       for side in ("left", "right")):
                raise ValueError("enabled finger_flex_pads require contact boxes for both fingers")
            if pads["enabled"] and scene.get("finger_collision_geometry") == "mesh":
                raise ValueError("flexible finger pads require box collision geometry")
        initial_arm_pose = scene.get("initial_arm_joint_positions_rad", {})
        if not isinstance(initial_arm_pose, dict):
            raise ValueError("initial_arm_joint_positions_rad must be a joint-to-position mapping")
        unknown_arm_joints = set(initial_arm_pose) - _ARM_JOINTS
        if unknown_arm_joints:
            raise ValueError(
                "initial_arm_joint_positions_rad contains unknown joints: "
                f"{sorted(unknown_arm_joints)}")
        for joint_name, position in initial_arm_pose.items():
            if (isinstance(position, bool) or not isinstance(position, (int, float)) or
                    not math.isfinite(position)):
                raise ValueError(
                    f"initial_arm_joint_positions_rad.{joint_name} must be a finite number")
        arm_actuator = scene.get("arm_position_actuator", {})
        if not isinstance(arm_actuator, dict) or set(arm_actuator) - {"kp", "kv", "max_force"}:
            raise ValueError("arm_position_actuator must contain only kp, kv, and max_force")
        for parameter, value in arm_actuator.items():
            self._positive_number(value, f"arm_position_actuator.{parameter}")
        self._finite_vector(scene.get("finger_contact_center_offset_m", [0.0, 0.0, 0.0]), 3,
                            "finger_contact_center_offset_m")
        if "finger_contact_box_half_size_m" in scene:
            self._positive_vector(scene["finger_contact_box_half_size_m"], 3,
                                  "finger_contact_box_half_size_m")
        material = scene.get("finger_contact_material", {})
        if not isinstance(material, dict) or set(material) - {"friction", "condim", "solref", "solimp"}:
            raise ValueError("finger_contact_material must contain only friction, condim, solref, and solimp")
        if "friction" in material:
            friction = self._finite_vector(material["friction"], 3, "finger_contact_material.friction")
            if any(value < 0 for value in friction):
                raise ValueError("finger_contact_material.friction values must be nonnegative")
        if "condim" in material and (type(material["condim"]) is not int or material["condim"] not in {1, 3, 4, 6}):
            raise ValueError("finger_contact_material.condim must be 1, 3, 4, or 6")
        if "solref" in material:
            self._positive_vector(material["solref"], 2, "finger_contact_material.solref")
        if "solimp" in material:
            solimp = self._finite_vector(material["solimp"], 5, "finger_contact_material.solimp")
            if not (0 < solimp[0] <= solimp[1] < 1 and solimp[2] > 0 and
                    0 < solimp[3] < 1 and solimp[4] >= 1):
                raise ValueError("finger_contact_material.solimp must have 0 < dmin <= dmax < 1, "
                                 "width > 0, 0 < midpoint < 1, and power >= 1")
        boxes = scene.get("finger_contact_boxes", {})
        if not isinstance(boxes, dict) or set(boxes) - {"left", "right"}:
            raise ValueError("finger_contact_boxes must contain only left and right lists")
        for side, parts in boxes.items():
            if not isinstance(parts, list) or not parts:
                raise ValueError(f"finger_contact_boxes.{side} must be a non-empty list")
            for index, part in enumerate(parts):
                field = f"finger_contact_boxes.{side}[{index}]"
                if not isinstance(part, dict) or set(part) != {"position", "rotation", "half_size"}:
                    raise ValueError(f"{field} must contain position, rotation, and half_size")
                self._finite_vector(part["position"], 3, f"{field}.position")
                self._finite_vector(part["rotation"], 3, f"{field}.rotation")
                self._positive_vector(part["half_size"], 3, f"{field}.half_size")
        ground = scene.get("ground", {})
        if not isinstance(ground, dict) or set(ground) - {"size", "rgba"}:
            raise ValueError("ground must contain only size and rgba")
        self._positive_vector(ground.get("size", [20.0, 20.0, 0.1]), 3, "ground.size")
        self._rgba(ground.get("rgba", [0.75, 0.75, 0.75, 1.0]), "ground.rgba")
        objects = scene.get("objects", [])
        if not isinstance(objects, list):
            raise ValueError("objects must be a list")
        identifiers = set()
        for index, obj in enumerate(objects):
            field = f"objects[{index}]"
            if not isinstance(obj, dict):
                raise ValueError(f"{field} must be a mapping")
            identifier = obj.get("id")
            if not isinstance(identifier, str) or not _OBJECT_ID.fullmatch(identifier):
                raise ValueError(f"{field}.id is invalid")
            if identifier in identifiers:
                raise ValueError(f"duplicate scene object ID: {identifier}")
            identifiers.add(identifier)
            shape = obj.get("shape")
            if shape not in _SHAPES:
                raise ValueError(f"{field}.shape must be one of {sorted(_SHAPES)}")
            missing = _COMMON_FIELDS - set(obj)
            if missing:
                raise ValueError(f"{field} is missing required fields: {sorted(missing)}")
            motion = obj.get("motion", "static")
            if motion not in {"static", "dynamic"}:
                raise ValueError(f"{field}.motion must be static or dynamic")
            allowed = _COMMON_FIELDS | ({"size"} if shape == "box" else set())
            allowed |= ({"radius"} if shape in {"sphere", "cylinder"} else set())
            allowed |= ({"height"} if shape == "cylinder" else set())
            allowed |= ({"mesh", "scale"} if shape == "mesh" else set())
            allowed |= {"mass_kg"} if motion == "dynamic" else set()
            if set(obj) - allowed:
                raise ValueError(f"{field} contains fields that do not match shape {shape}")
            self._positive_vector(obj.get("position", [0.0, 0.0, 0.0]), 3, f"{field}.position", allow_negative=True)
            self._finite_vector(obj.get("rotation", [0.0, 0.0, 0.0]), 3, f"{field}.rotation")
            self._rgba(obj.get("rgba", [0.6, 0.6, 0.6, 1.0]), f"{field}.rgba")
            if shape == "box":
                self._positive_vector(obj.get("size"), 3, f"{field}.size")
            elif shape in {"sphere", "cylinder"}:
                self._positive_number(obj.get("radius"), f"{field}.radius")
                if shape == "cylinder":
                    self._positive_number(obj.get("height"), f"{field}.height")
            else:
                mesh_ref = obj.get("mesh")
                if not isinstance(mesh_ref, str):
                    raise ValueError(f"{field}.mesh must be a scene-relative asset path")
                posix_path = PurePosixPath(mesh_ref)
                if (posix_path.is_absolute() or "\\" in mesh_ref or ".." in posix_path.parts or
                        len(posix_path.parts) != 3 or posix_path.parts[:2] != ("assets", scene_id) or
                        Path(mesh_ref).suffix.lower() not in {".stl", ".obj"}):
                    raise ValueError(f"{field}.mesh must reference assets/{scene_id}/<file>.stl or .obj")
                asset_path = (self.directory / Path(*posix_path.parts)).resolve(strict=True)
                if not asset_path.is_relative_to(self.assets_directory.resolve()) or not asset_path.is_file():
                    raise ValueError(f"{field}.mesh file does not exist inside this scene's asset directory")
                self._positive_vector(obj.get("scale", [1.0, 1.0, 1.0]), 3, f"{field}.scale")
            if motion == "dynamic":
                self._positive_number(obj.get("mass_kg"), f"{field}.mass_kg")

    @staticmethod
    def _finite_vector(value: Any, length: int, field: str, allow_negative: bool = True) -> list[float]:
        """Validate a finite numeric vector and return normalized float values.

        Args:
            value: Candidate sequence.
            length: Required element count.
            field: Field name used in errors.
            allow_negative: Whether negative values are allowed.

        Returns:
            The validated values as floats.
        """
        if (not isinstance(value, (list, tuple)) or len(value) != length or
                any(isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item)
                    for item in value)):
            raise ValueError(f"{field} must contain {length} finite numbers")
        numbers = [float(item) for item in value]
        if not allow_negative and any(item <= 0 for item in numbers):
            raise ValueError(f"{field} values must be positive")
        return numbers

    @classmethod
    def _positive_vector(cls, value: Any, length: int, field: str,
                         allow_negative: bool = False) -> list[float]:
        """Validate a finite vector, requiring positive entries unless positional.

        Args:
            value: Candidate sequence.
            length: Required element count.
            field: Field name used in errors.
            allow_negative: Allow negative values for position coordinates.

        Returns:
            The validated values as floats.
        """
        return cls._finite_vector(value, length, field, allow_negative=allow_negative)

    @staticmethod
    def _positive_number(value: Any, field: str) -> float:
        """Validate one finite positive numeric value.

        Args:
            value: Candidate number.
            field: Field name used in errors.

        Returns:
            The validated value as a float.
        """
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{field} must be a positive finite number")
        return float(value)

    @classmethod
    def _rgba(cls, value: Any, field: str) -> list[float]:
        """Validate an RGBA color vector in the MuJoCo 0..1 range.

        Args:
            value: Candidate RGBA sequence.
            field: Field name used in errors.

        Returns:
            The validated RGBA components as floats.
        """
        rgba = cls._finite_vector(value, 4, field)
        if any(component < 0.0 or component > 1.0 for component in rgba):
            raise ValueError(f"{field} values must be in [0, 1]")
        return rgba
