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
