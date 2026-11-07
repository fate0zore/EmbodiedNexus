"""Checks for simulator arm trajectory diagnostic events."""

import unittest
from types import MethodType, SimpleNamespace

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from embodied_nexus_sim_mujoco.simulator import ARM_JOINTS, MujocoSimulation


class ArmTraceTests(unittest.TestCase):
    """Check receipt, replacement, and completion of simulated arm trajectories."""

    def test_received_replaced_and_finished_trajectory(self):
        """Record a replacement and distinguish finished targets from measured positions.

        Returns:
            None.
        """
        logs = []
        simulation = SimpleNamespace(
            data=SimpleNamespace(time=0.0, qpos=[0.0] * len(ARM_JOINTS),
                                 ctrl=[0.0] * len(ARM_JOINTS)),
            _actuators=dict(zip(ARM_JOINTS, range(len(ARM_JOINTS)))),
            _arm_trajectory=None, _arm_trajectory_sequence=0, _last_arm_trajectory_event=None,
            _joint_qpos=lambda name: ARM_JOINTS.index(name),
            get_logger=lambda: SimpleNamespace(info=logs.append, error=logs.append),
        )
        simulation._trajectory = MethodType(MujocoSimulation._trajectory, simulation)
        message = JointTrajectory()
        message.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [0.5] * len(ARM_JOINTS)
        point.time_from_start.sec = 1
        message.points = [point]
        MujocoSimulation._on_arm_trajectory(simulation, message)
        self.assertFalse(simulation._last_arm_trajectory_event["replaced_active_trajectory"])
        MujocoSimulation._on_arm_trajectory(simulation, message)
        self.assertTrue(simulation._last_arm_trajectory_event["replaced_active_trajectory"])
        self.assertEqual(simulation._last_arm_trajectory_event["previous_sequence"], 1)
        simulation.data.time = 1.0
        MujocoSimulation._apply_trajectory(simulation, simulation._arm_trajectory, ARM_JOINTS)
        event = simulation._last_arm_trajectory_event
        self.assertEqual(event["state"], "targets_finished")
        self.assertEqual(event["endpoint_rad"][ARM_JOINTS[0]], 0.5)
        self.assertEqual(event["finished_positions_rad"][ARM_JOINTS[0]], 0.0)
        self.assertIsNone(simulation._arm_trajectory)
        self.assertEqual(len(logs), 3)


if __name__ == "__main__":
    unittest.main()
