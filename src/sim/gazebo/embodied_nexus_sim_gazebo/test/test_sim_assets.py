from __future__ import annotations

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[5]
GAZEBO = REPO / "src/sim/gazebo/embodied_nexus_sim_gazebo"
DESCRIPTION = REPO / "src/sim/common/third_party/embodied_nexus_sim_description"


class SimulationAssetTests(unittest.TestCase):
    def test_xacro_and_world_are_well_formed_and_vendor_meshes_are_installed(self):
        for source in DESCRIPTION.rglob("*.xacro"):
            ET.parse(source)
        ET.parse(GAZEBO / "worlds/workcell.sdf")
        missing = []
        for source in DESCRIPTION.rglob("*.xacro"):
            content = source.read_text(encoding="utf-8")
            for uri in re.findall(r"package://embodied_nexus_sim_description/meshes/([^\"']+)", content):
                if "${" in uri:
                    continue
                if not (DESCRIPTION / "meshes" / uri).is_file():
                    missing.append(f"{source.relative_to(DESCRIPTION)} -> {uri}")
        self.assertEqual(missing, [])

    def test_platform_profile_and_control_joints_match_device_contracts(self):
        profile = yaml.safe_load((GAZEBO / "config/platform_sim.yaml").read_text(encoding="utf-8"))
        self.assertEqual({item["type"] for item in profile["devices"]},
                         {"base", "arm", "gripper", "camera", "lidar"})
        self.assertTrue(all(item["use_sim_time"] for item in profile["devices"]))
        controllers = yaml.safe_load((GAZEBO / "config/controllers.yaml").read_text(encoding="utf-8"))
        self.assertEqual(controllers["/sim/arachne/controller_manager"]["ros__parameters"]["update_rate"], 100)
        self.assertEqual(len(controllers["/sim/arachne/arm_controller"]["ros__parameters"]["joints"]), 6)
        self.assertEqual(len(controllers["/sim/arachne/gripper_controller"]["ros__parameters"]["joints"]), 2)

    def test_gazebo_launch_uses_humble_fortress_bridge_message_types(self):
        launch = (GAZEBO / "launch/sim.launch.py").read_text(encoding="utf-8")
        self.assertIn("gz_ros2_control/GazeboSimSystem", launch)
        self.assertIn("gz_ros2_control-system", launch)
        self.assertIn("--render-engine ogre", launch)
        self.assertIn("ros_gz_interfaces/srv/ControlWorld", launch)
        self.assertIn("executable=\"sim_clock_guard\"", launch)
        self.assertIn('"type": "gpu_lidar"', launch)
        self.assertIn('("samples", "180")', launch)
        self.assertIn('"lslidar_c16_front"', launch)
        self.assertIn('"lslidar_c16_rear"', launch)
        self.assertIn('("mu1", "1.2"), ("mu2", "0.25")', launch)
        relay = (GAZEBO / "embodied_nexus_sim_gazebo/sensor_relay.py").read_text(encoding="utf-8")
        self.assertIn("_merge_lidar_clouds", relay)
        wheels = (DESCRIPTION / "urdf/scout/scout_2_vendor.xacro").read_text(encoding="utf-8")
        self.assertEqual(wheels.count('axis="0 1 0"'), 4)
        self.assertIn("ignition.msgs.PointCloudPacked", launch)
        clock_guard = (GAZEBO / "embodied_nexus_sim_gazebo/clock_guard.py").read_text(encoding="utf-8")
        self.assertIn("world_control.pause = True", clock_guard)
        rewind_example = GAZEBO / "examples/validate_clock_rewind.py"
        self.assertTrue(rewind_example.is_file())
        controllers = (GAZEBO / "config/controllers.yaml").read_text(encoding="utf-8")
        self.assertIn("use_stamped_vel: true", controllers)

    def test_every_upstream_license_notice_is_recorded(self):
        licenses = DESCRIPTION / "third_party/licenses"
        self.assertTrue((licenses / "arachne-MIT-LICENSE.txt").is_file())
        self.assertTrue((licenses / "scout-ros2-Apache-2.0.txt").is_file())
        source_notes = (DESCRIPTION / "UPSTREAM_README.md").read_text(encoding="utf-8")
        self.assertIn("BSD", source_notes)
        self.assertIn("41caa564204eb5f6a39200ce2600ed052ab517b0", source_notes)


if __name__ == "__main__":
    unittest.main()
