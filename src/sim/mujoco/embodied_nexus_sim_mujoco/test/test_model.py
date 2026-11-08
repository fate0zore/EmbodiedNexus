"""Tests for the shared-URDF to MuJoCo scene conversion."""

import unittest
import importlib.util
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from embodied_nexus_sim_mujoco.model import make_scene


class SceneConversionTests(unittest.TestCase):
    """Verify joint and geometry conversion without a ROS installation."""

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is required")
    def test_stationary_base_and_noslip_are_scene_specific(self):
        """Compile an optional fixed base and friction solver without changing defaults.

        Returns:
            None.
        """
        import mujoco
        urdf = """<robot name="test"><link name="base_link"><inertial>
          <origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1"/>
          </inertial></link></robot>"""
        fixed = mujoco.MjModel.from_xml_string(make_scene(urdf, ".", scene_config={
            "name": "stationary", "fixed_base": True, "noslip_iterations": 10}))
        self.assertEqual(fixed.neq, 1)
        self.assertEqual(fixed.opt.noslip_iterations, 10)
        default = mujoco.MjModel.from_xml_string(make_scene(urdf, "."))
        self.assertEqual(default.neq, 0)
        self.assertEqual(default.opt.noslip_iterations, 0)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is required")
    def test_configured_elliptic_friction_cone_compiles(self):
        """Apply the scene friction cone and retain the default for other scenes.

        Returns:
            None.
        """
        import mujoco
        urdf = """<robot name="test"><link name="base_link"><inertial>
          <origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1"/>
          </inertial></link></robot>"""
        model = mujoco.MjModel.from_xml_string(make_scene(urdf, ".", scene_config={
            "name": "elliptic", "contact_cone": "elliptic"}))
        self.assertEqual(model.opt.cone, mujoco.mjtCone.mjCONE_ELLIPTIC)
        default = mujoco.MjModel.from_xml_string(make_scene(urdf, "."))
        self.assertEqual(default.opt.cone, mujoco.mjtCone.mjCONE_PYRAMIDAL)

    def test_convert_joint_and_geometry(self):
        """Preserve source joint limits and collision box dimensions.

        Returns:
            None.
        """
        urdf = """<robot name="test"><link name="base_link"/>
        <link name="arm"><collision><geometry><box size="2 4 6"/></geometry></collision></link>
        <joint name="arm_joint" type="revolute"><parent link="base_link"/>
        <child link="arm"/><axis xyz="0 0 1"/><limit lower="-1" upper="2"/></joint></robot>"""
        scene = ET.fromstring(make_scene(urdf, "."))
        self.assertEqual(scene.find(".//joint[@name='arm_joint']").get("range"), "-1 2")
        self.assertEqual(scene.find(".//body[@name='arm']/geom").get("size"), "1.0 2.0 3.0")

    def test_scene_config_converts_actual_sizes_and_dynamic_bodies(self):
        """Convert configured workcell geometry and dynamic mass to MJCF attributes.

        Returns:
            None.
        """
        urdf = """<robot name="test"><link name="base_link"/>
        <link name="arm"/><joint name="arm_joint" type="fixed">
        <parent link="base_link"/><child link="arm"/></joint></robot>"""
        configured = {"name": "Test", "ground": {"size": [8, 6, 0.1]}, "objects": [
            {"id": "workbench", "shape": "box", "position": [1, 2, 0.4], "rotation": [0, 0, 0],
             "size": [2, 1, 0.8], "rgba": [0.2, 0.3, 0.4, 1], "motion": "static"},
            {"id": "ball", "shape": "sphere", "position": [0, 0, 1], "rotation": [0, 0, 0],
             "radius": 0.25, "rgba": [1, 0, 0, 1], "motion": "dynamic", "mass_kg": 0.7},
        ]}
        mjcf = ET.fromstring(make_scene(urdf, ".", scene_config=configured))
        self.assertEqual(mjcf.find("./worldbody/geom[@name='workbench_geom']").get("size"), "1.0 0.5 0.4")
        self.assertEqual(mjcf.find("./worldbody/body[@name='ball_body']/freejoint").get("name"), "ball_free")
        self.assertEqual(mjcf.find("./worldbody/body[@name='ball_body']/geom").get("mass"), "0.7")
        self.assertEqual(mjcf.find("./worldbody/geom[@name='ground']").get("size"), "8 6 0.1")

    def test_scene_config_sets_aubo_position_actuator_gains(self):
        """Apply scene-specific position gains to the Aubo arm actuators.

        Returns:
            None.
        """
        urdf = """<robot name="test"><link name="base_link"/><link name="arm"/>
        <joint name="aubo_foreArm_joint" type="revolute"><parent link="base_link"/>
        <child link="arm"/><axis xyz="0 0 1"/><limit lower="-4.7" upper="4.7"/></joint></robot>"""
        configured = {"name": "Actuator Test", "arm_position_actuator":
                      {"kp": 1500.0, "kv": 80.0, "max_force": 500.0}}
        mjcf = ET.fromstring(make_scene(urdf, ".", scene_config=configured))
        actuator = mjcf.find("./actuator/position[@name='aubo_foreArm_joint']")
        self.assertEqual(actuator.get("kp"), "1500.0")
        self.assertEqual(actuator.get("kv"), "80.0")
        self.assertEqual(actuator.get("forcerange"), "-500 500")

    def test_scene_config_offsets_simplified_finger_contact_geometry(self):
        """Shift only the simplified contact geom by the scene's local offset.

        Returns:
            None.
        """
        urdf = """<robot name="test">
          <link name="base_link"/><link name="gripper_base"/>
          <link name="ms42dc_left_finger_link"><collision>
            <origin xyz="0 0.02 0"/><geometry><box size="0.02 0.04 0.02"/></geometry>
          </collision></link>
          <joint name="gripper_mount" type="fixed"><parent link="base_link"/>
            <child link="gripper_base"/></joint>
          <joint name="left_finger_mount" type="fixed"><parent link="gripper_base"/>
            <child link="ms42dc_left_finger_link"/></joint>
        </robot>"""
        configured = {"name": "Contact Offset", "finger_contact_center_offset_m": [0.0, 0.015, 0.0],
                      "finger_contact_box_half_size_m": [0.010, 0.045, 0.011]}
        mjcf = ET.fromstring(make_scene(urdf, ".", scene_config=configured))
        contact = mjcf.find(".//geom[@name='ms42dc_left_finger_link_contact_0']")
        self.assertEqual(contact.get("pos"), "0.0 0.035 0.0")
        self.assertEqual(contact.get("size"), "0.01 0.045 0.011")

    def test_scene_config_supports_multiple_oriented_finger_boxes(self):
        """Preserve each link-local collision box position, size, and rotation.

        Returns:
            None.
        """
        urdf = """<robot name="test"><link name="base_link"/>
          <link name="ms42dc_left_finger_link"><collision><geometry>
            <box size="0.02 0.04 0.02"/></geometry></collision></link>
          <joint name="finger" type="fixed"><parent link="base_link"/>
            <child link="ms42dc_left_finger_link"/></joint></robot>"""
        parts = [{"position": [0.01, 0.02, 0.03], "rotation": [0, 0, 0.38],
                  "half_size": [0.01, 0.03, 0.008]},
                 {"position": [0, 0, 0], "rotation": [0, 0, 0],
                  "half_size": [0.02, 0.02, 0.01]}]
        mjcf = ET.fromstring(make_scene(urdf, ".", scene_config={
            "name": "Parts", "finger_contact_boxes": {"left": parts}}))
        first = mjcf.find(".//geom[@name='ms42dc_left_finger_link_contact_0_0']")
        second = mjcf.find(".//geom[@name='ms42dc_left_finger_link_contact_0_1']")
        self.assertEqual(first.get("pos"), "0.01 0.02 0.03")
        self.assertEqual(first.get("euler"), "0.0 0.0 0.38")
        self.assertEqual(first.get("size"), "0.01 0.03 0.008")
        self.assertIsNotNone(second)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is required")
    def test_finger_material_controls_actual_object_contact(self):
        """Apply rubber contact parameters to both simplified and URDF finger geometry.

        Returns:
            None.
        """
        import mujoco

        urdf = """<robot name="test"><link name="base_link"><inertial>
          <origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1"/></inertial></link>
          <link name="ms42dc_left_finger_link"><inertial><origin xyz="0 0 0"/>
          <mass value="0.1"/><inertia ixx="0.001" iyy="0.001" izz="0.001"/></inertial><collision><geometry>
          <box size="0.046 0.104 0.024"/></geometry></collision></link>
          <joint name="finger" type="fixed"><parent link="base_link"/>
          <child link="ms42dc_left_finger_link"/></joint></robot>"""
        material = {"friction": [2.0, 0.02, 0.001], "condim": 4,
                    "solref": [0.03, 1.0], "solimp": [0.9, 0.95, 0.002, 0.5, 2.0]}
        configured = {"name": "Rubber", "finger_contact_material": material, "objects": [{
            "id": "ball", "shape": "sphere", "position": [0.04, 0, 0.236],
            "radius": 0.03, "motion": "dynamic", "mass_kg": 0.1}]}
        for simplified in (True, False):
            with self.subTest(simplified=simplified):
                model = mujoco.MjModel.from_xml_string(
                    make_scene(urdf, ".", scene_config=configured,
                               simplified_finger_collision=simplified))
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ball_geom")
                contacts = [data.contact[i] for i in range(data.ncon)
                            if ball in (data.contact[i].geom1, data.contact[i].geom2)]
                self.assertTrue(contacts)
                contact = contacts[0]
                self.assertEqual(contact.dim, 4)
                self.assertAlmostEqual(contact.friction[0], 2.0)
                self.assertAlmostEqual(contact.friction[2], 0.02)
                self.assertAlmostEqual(contact.solref[0], 0.03)
                self.assertAlmostEqual(contact.solimp[2], 0.002)

    @unittest.skipUnless(shutil.which("xacro") and importlib.util.find_spec("mujoco"),
                         "ROS xacro and MuJoCo are required")
    def test_green_cylinder_uses_original_finger_mesh_and_material(self):
        """Compile both original finger meshes with the configured contact material.

        Returns:
            None.
        """
        import subprocess
        import mujoco
        import numpy as np
        import yaml

        package = Path(__file__).resolve().parents[1]
        description = package.parents[1] / "common/third_party/embodied_nexus_sim_description"
        scene_dir = package / "config/scenes"
        scene = yaml.safe_load((scene_dir / "scene-1793463913737.yaml").read_text(encoding="utf-8"))
        urdf = subprocess.run(["xacro", str(description / "urdf/arachne.urdf.xacro"),
                               "gazebo_drive_wheels:=true", "with_mimic_joints:=false"],
                              check=True, capture_output=True, text=True).stdout
        xml = make_scene(urdf, description, scene_config=scene, scene_directory=scene_dir)
        mjcf = ET.fromstring(xml)
        for side in ("left", "right"):
            body = mjcf.find(f".//body[@name='ms42dc_{side}_finger_link']")
            geom = next(item for item in body.findall("geom") if item.get("contype") == "2")
            mesh = mjcf.find(f".//asset/mesh[@name='{geom.get('mesh')}']")
            self.assertEqual(Path(mesh.get("file")).name, f"ms42dc_{side}_finger.stl")
            self.assertEqual(mesh.get("scale"), "0.001 0.001 0.001")
        model = mujoco.MjModel.from_xml_string(xml)
        self.assertEqual(model.nflex, 0)
        for side in ("left", "right"):
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"ms42dc_{side}_finger_link")
            geoms = [i for i in range(model.ngeom) if model.geom_bodyid[i] == body and model.geom_contype[i]]
            self.assertEqual(len(geoms), 1)
            geom = geoms[0]
            self.assertEqual(model.geom_type[geom], mujoco.mjtGeom.mjGEOM_MESH)
            self.assertEqual(model.geom_priority[geom], 1)
            np.testing.assert_allclose(model.geom_friction[geom], [5, 0.02, 0.001])
            np.testing.assert_allclose(model.geom_solref[geom], [0.008, 1])
            self.assertEqual(model.geom_condim[geom], 4)
            mesh = model.geom_dataid[geom]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            # MuJoCo recenters and rotates mesh vertices on compilation; their
            # bounding box in mesh coordinates need not match the source STL.
            self.assertGreater(count, 100)
            self.assertGreater(np.linalg.norm(np.ptp(model.mesh_vert[start:start + count], axis=0)), 0.1)
            # Place the cylinder in the finger's convex collision volume to
            # check the actual contact parameters; this is not a grasp test.
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            cylinder_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cylinder-1_free")
            data.qpos[model.jnt_qposadr[cylinder_joint]:model.jnt_qposadr[cylinder_joint] + 3] = data.geom_xpos[geom]
            mujoco.mj_forward(model, data)
            cylinder = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cylinder-1_geom")
            contacts = [contact for contact in data.contact if geom in contact.geom and cylinder in contact.geom]
            self.assertTrue(contacts)
            self.assertAlmostEqual(contacts[0].friction[0], 5.0)
            self.assertAlmostEqual(contacts[0].solref[0], 0.008)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is required")
    def test_compile_configured_static_dynamic_and_uploaded_mesh_objects(self):
        """Compile a scene with static and dynamic primitives and an OBJ collision mesh.

        Returns:
            None.
        """
        import tempfile
        import mujoco

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            assets = root / "assets" / "test"
            assets.mkdir(parents=True)
            (assets / "tetra.obj").write_text(
                "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n",
                encoding="utf-8",
            )
            urdf = """<robot name="test"><link name="base_link"><inertial><origin xyz="0 0 0"/>
            <mass value="5"/><inertia ixx="1" iyy="1" izz="1"/></inertial></link></robot>"""
            configured = {"name": "Compiled", "objects": [
                {"id": "static_box", "shape": "box", "position": [1, 0, 0.5], "rotation": [0, 0, 0],
                 "size": [1, 1, 1], "rgba": [0, 1, 0, 1], "motion": "static"},
                {"id": "dynamic_ball", "shape": "sphere", "position": [0, 0, 1], "rotation": [0, 0, 0],
                 "radius": 0.2, "rgba": [1, 0, 0, 1], "motion": "dynamic", "mass_kg": 0.4},
                {"id": "mesh_body", "shape": "mesh", "position": [0, 1, 0.5], "rotation": [0, 0, 0],
                 "mesh": "assets/test/tetra.obj", "scale": [0.2, 0.2, 0.2], "rgba": [0, 0, 1, 1],
                 "motion": "dynamic", "mass_kg": 0.6},
            ]}
            model = mujoco.MjModel.from_xml_string(make_scene(urdf, ".", scene_config=configured,
                                                               scene_directory=root))
            dynamic_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "dynamic_ball_body")
            mesh_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "mesh_body_body")
            self.assertAlmostEqual(float(model.body_mass[dynamic_id]), 0.4)
            self.assertAlmostEqual(float(model.body_mass[mesh_id]), 0.6)
            self.assertGreaterEqual(model.ngeom, 4)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is required")
    def test_urdf_fixed_axis_rpy_is_preserved(self):
        """Preserve a noncommuting URDF joint and collision rotation.

        Returns:
            None.
        """
        import mujoco
        import numpy as np

        urdf = """<robot name="test"><link name="base_link">
        <inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1"/></inertial></link>
        <link name="arm"><collision><origin rpy="1.5707963267948966 0 1.5707963267948966"/>
        <geometry><box size="1 1 1"/></geometry></collision></link>
        <joint name="arm_joint" type="fixed"><parent link="base_link"/><child link="arm"/>
        <origin rpy="-1.5707963267948966 -1.5707963267948966 0"/></joint></robot>"""
        model = mujoco.MjModel.from_xml_string(make_scene(urdf, "."))
        arm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm")
        joint_matrix = np.empty(9)
        mujoco.mju_quat2Mat(joint_matrix, model.body_quat[arm])
        np.testing.assert_allclose(joint_matrix.reshape(3, 3),
                                   [[0, 1, 0], [0, 0, 1], [1, 0, 0]], atol=1e-10)
        geometry_matrix = np.empty(9)
        mujoco.mju_quat2Mat(geometry_matrix, model.geom_quat[model.body_geomadr[arm]])
        np.testing.assert_allclose(geometry_matrix.reshape(3, 3),
                                   [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-10)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is required")
    def test_render_projection_matches_camera_info(self):
        """Match the rendered frustum to the camera's top-left pixel intrinsics.

        Returns:
            None.
        """
        import mujoco
        import numpy as np

        stream = {"width": 640, "height": 480, "fx": 372.082153, "fy": 372.082153,
                  "cx": 319.400024, "cy": 241.5}
        urdf = """<robot name="test"><link name="base_link">
        <inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1"/></inertial></link>
        <link name="camera_depth_frame"/><joint name="camera_joint" type="fixed">
        <parent link="base_link"/><child link="camera_depth_frame"/>
        <origin rpy="0.3 -0.8 0.5"/></joint>
        <link name="camera_depth_optical_frame"/><joint name="optical_joint" type="fixed">
        <parent link="camera_depth_frame"/><child link="camera_depth_optical_frame"/>
        <origin rpy="-1.5707963267948966 0 -1.5707963267948966"/></joint></robot>"""
        model = mujoco.MjModel.from_xml_string(make_scene(urdf, ".", {"streams": {"depth": stream}}))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        scene = mujoco.MjvScene(model, maxgeom=100)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        camera.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "camera_depth")
        optical_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "camera_depth_optical_frame")
        render_optical = data.cam_xmat[camera.fixedcamid].reshape(3, 3) @ np.diag([1, -1, -1])
        np.testing.assert_allclose(render_optical, data.xmat[optical_id].reshape(3, 3), atol=1e-10)
        mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera,
                              mujoco.mjtCatBit.mjCAT_ALL, scene)
        projection = scene.camera[0]
        cx = stream["width"] / 2 - projection.frustum_center * stream["fx"] / projection.frustum_near
        cy = stream["height"] / 2 + ((projection.frustum_top + projection.frustum_bottom) / 2
                                      * stream["fy"] / projection.frustum_near)
        self.assertAlmostEqual(cx, stream["cx"], places=3)
        self.assertAlmostEqual(cy, stream["cy"], places=3)

    @unittest.skipUnless(importlib.util.find_spec("mujoco") and shutil.which("xacro"),
                         "MuJoCo and a sourced ROS xacro are required")
    def test_shared_model_compiles_and_steps(self):
        """Compile the real shared model and advance its physics one second.

        Returns:
            None.
        """
        import mujoco

        description = Path(__file__).resolve().parents[3] / "common/third_party/embodied_nexus_sim_description"
        urdf = subprocess.run(["xacro", str(description / "urdf/arachne.urdf.xacro"),
                               "gazebo_drive_wheels:=true", "with_mimic_joints:=false"],
                              check=True, capture_output=True, text=True).stdout
        model = mujoco.MjModel.from_xml_string(make_scene(urdf, description))
        data = mujoco.MjData(model)
        self.assertEqual(model.njnt, 13)
        for _ in range(500):
            mujoco.mj_step(model, data)
        self.assertAlmostEqual(data.time, 1.0)
        self.assertTrue(all(warning.number == 0 for warning in data.warning))


if __name__ == "__main__":
    unittest.main()
