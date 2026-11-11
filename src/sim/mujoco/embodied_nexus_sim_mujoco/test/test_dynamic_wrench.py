"""Verify the simulation-only dynamic-grasp test wrench boundary."""

from types import SimpleNamespace
import unittest

import mujoco
from geometry_msgs.msg import WrenchStamped

from embodied_nexus_sim_mujoco.simulator import MujocoSimulation


class TestDynamicWrench(unittest.TestCase):
    """Exercise scene, object and force limits without starting ROS nodes."""

    def setUp(self):
        """Build a minimal dynamic target and test request.

        Returns:
            None.
        """
        model = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><body name="target_body"><freejoint/>'
            '<geom type="sphere" size="0.02" mass="0.1"/>'
            '</body></worldbody></mujoco>')
        self.sim = SimpleNamespace(_scene_name="dynamic_grasp_cylinder", model=model,
                                   _scene_objects=[{"id": "target", "motion": "dynamic"}],
                                   _test_wrench=None)
        self.message = WrenchStamped()
        self.message.header.frame_id = "target"
        self.message.wrench.force.z = 1.0

    def test_restricts_scene_and_object(self):
        """Accept a known dynamic target only inside dedicated test scenes.

        Returns:
            None.
        """
        self.sim._scene_name = "visual_servo_convex"
        MujocoSimulation._on_test_object_wrench(self.sim, self.message)
        self.assertIsNone(self.sim._test_wrench)
        self.sim._scene_name = "dynamic_grasp_cylinder"
        self.message.header.frame_id = "not-target"
        MujocoSimulation._on_test_object_wrench(self.sim, self.message)
        self.assertIsNone(self.sim._test_wrench)
        self.message.header.frame_id = "target"
        MujocoSimulation._on_test_object_wrench(self.sim, self.message)
        self.assertIsNotNone(self.sim._test_wrench)

    def test_rejects_unsafe_wrenches(self):
        """Ignore nonfinite and excessive force commands.

        Returns:
            None.
        """
        self.message.wrench.force.x = float("nan")
        MujocoSimulation._on_test_object_wrench(self.sim, self.message)
        self.assertIsNone(self.sim._test_wrench)
        self.message.wrench.force.x = 11.0
        MujocoSimulation._on_test_object_wrench(self.sim, self.message)
        self.assertIsNone(self.sim._test_wrench)
