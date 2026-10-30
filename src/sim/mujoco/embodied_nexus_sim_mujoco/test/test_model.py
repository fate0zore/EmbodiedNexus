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
