"""Exercise the force fixture against a minimal MuJoCo free body."""

from types import MethodType, SimpleNamespace
import unittest

try:
    import mujoco
    import numpy as np
    from embodied_nexus_sim_mujoco_interfaces.msg import SceneObjectStates
    from geometry_msgs.msg import Pose
    from std_msgs.msg import String
    from embodied_nexus_dynamic_grasp.motion_fixture import MotionFixture
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "MuJoCo and ROS 2 Humble interfaces are unavailable")
class MotionFixtureTest(unittest.TestCase):
    """Verify the fixture moves and releases an actual simulated free body."""

    def test_tabletop_motion_releases_free_body(self):
        """Slide each target on the ground and remove force at release.

        Returns:
            None.
        """
        for shape in ('box', 'cylinder'):
            with self.subTest(shape=shape):
                geom = ('type="box" size=".04 .02 .015"' if shape == 'box' else
                        'type="cylinder" size=".02 .05" euler="0 1.57079632679 0"')
                height = .015 if shape == 'box' else .02
                model = mujoco.MjModel.from_xml_string(
                    '<mujoco><compiler angle="radian"/><option timestep=".002" gravity="0 0 -9.8"/>'
                    '<worldbody><geom type="plane" size="1 1 .1" friction=".003 .0001 .00001"/>'
                    f'<body name="target_body" pos="0 0 {height}"><freejoint/>'
                    f'<geom {geom} mass=".1" friction=".003 .0001 .00001"/>'
                    '</body></worldbody></mujoco>')
                data = mujoco.MjData(model)
                commands, events = [], []
                fixture = SimpleNamespace(object_id='target', scenario='tabletop', phase='idle', previous=None,
                    hold_position=None, support_height=None, ready_since=None, start_stamp=None,
                    origin=None, origin_orientation=None, mass=.1, duration=175.0,
                    initial_velocity=np.array([-0.01, 0.0, 0.0]),
                    publisher=SimpleNamespace(publish=commands.append),
                    events=SimpleNamespace(publish=lambda event: events.append(event.data)))
                fixture._release = MethodType(MotionFixture._release, fixture)
                fixture._tabletop_state = MethodType(MotionFixture._tabletop_state, fixture)
                MotionFixture._prepare(fixture, String(data='target'))
                peak_speed = 0.0
                for _ in range(4000):
                    mujoco.mj_step(model, data, nstep=25)
                    message = SceneObjectStates()
                    message.header.stamp.sec = int(data.time)
                    message.header.stamp.nanosec = int((data.time - int(data.time)) * 1e9)
                    message.object_ids = ['target']
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = data.xpos[1].tolist()
                    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = data.xquat[1].tolist()
                    message.poses = [pose]
                    MotionFixture._state(fixture, message)
                    if commands:
                        wrench = commands[-1].wrench
                        data.xfrc_applied[1] = [wrench.force.x, wrench.force.y, wrench.force.z,
                                               wrench.torque.x, wrench.torque.y, wrench.torque.z]
                    if fixture.phase == 'hold':
                        MotionFixture._start(fixture, String(data='target'))
                    if fixture.phase == 'moving':
                        peak_speed = max(peak_speed, abs(float(data.qvel[0])))
                    if fixture.phase == 'released':
                        break
                self.assertIn('ready:target', events)
                self.assertIn('started:target', events)
                self.assertIn('released:target', events)
                self.assertAlmostEqual(float(data.xpos[1][0]), -.5, delta=.03)
                self.assertLess(peak_speed, .01)
                self.assertAlmostEqual(float(data.xpos[1][2]), height, delta=.003)
                self.assertEqual(commands[-1].wrench.force.x, 0.0)

    def test_cancel_releases_held_object(self):
        """Clear the wrench and emit a cancellation event when a trial stops.

        Returns:
            None.
        """
        commands, events = [], []
        fixture = SimpleNamespace(object_id="target", phase="hold",
                                  publisher=SimpleNamespace(publish=commands.append),
                                  events=SimpleNamespace(publish=lambda event: events.append(event.data)))
        fixture._release = MethodType(MotionFixture._release, fixture)
        MotionFixture._stop(fixture, String(data="target"))
        self.assertEqual(fixture.phase, "released")
        self.assertEqual(events, ["aborted_cancel:target"])
        self.assertEqual(commands[0].wrench.force.z, 0.0)

    def test_tabletop_clock_reversal_clears_force(self):
        """Release the free object if the simulation clock reverses.

        Returns:
            None.
        """
        commands, events = [], []
        fixture = SimpleNamespace(object_id='target', scenario='tabletop', phase='moving',
            previous=(2.0, np.zeros(3), np.array([0., 0., 0., 1.])),
            publisher=SimpleNamespace(publish=commands.append),
            events=SimpleNamespace(publish=lambda event: events.append(event.data)))
        fixture._release = MethodType(MotionFixture._release, fixture)
        message = SceneObjectStates()
        message.header.stamp.sec = 1
        message.object_ids = ['target']
        pose = Pose()
        pose.orientation.w = 1.0
        message.poses = [pose]
        MotionFixture._state(fixture, message)
        self.assertEqual(events, ['aborted_clock:target'])
        self.assertEqual(commands[-1].wrench.force.x, 0.0)

    def test_prepare_decelerate_and_release(self):
        """Raise, move in three dimensions, settle at support height and release.

        Returns:
            None.
        """
        model = mujoco.MjModel.from_xml_string(
            '<mujoco><option timestep="0.002" gravity="0 0 -9.8"/>'
            '<worldbody><geom type="plane" size="1 1 .1"/>'
            '<body name="target_body" pos="0 0 .02"><freejoint/>'
            '<geom type="sphere" size=".02" mass=".1"/>'
            '</body></worldbody></mujoco>')
        data = mujoco.MjData(model)
        commands, events = [], []
        fixture = SimpleNamespace(object_id="target", phase="idle", previous=None,
                                  hold_position=None, support_height=None, ready_since=None, start_stamp=None,
                                  origin=None, origin_orientation=None, mass=0.1, duration=4.0,
                                  last_report=-1.0,
                                  initial_velocity=np.array([0.007, 0.002, -0.005]), yaw_rate=0.0,
                                  get_logger=lambda: SimpleNamespace(info=lambda message: None),
                                  publisher=SimpleNamespace(publish=commands.append),
                                  events=SimpleNamespace(publish=lambda event: events.append(event.data)))
        fixture._release = MethodType(MotionFixture._release, fixture)
        MotionFixture._prepare(fixture, String(data="target"))
        max_speed = 0.0
        moved = False
        for _ in range(700):
            mujoco.mj_step(model, data, nstep=25)
            message = SceneObjectStates()
            message.header.stamp.sec = int(data.time)
            message.header.stamp.nanosec = int((data.time - int(data.time)) * 1e9)
            message.object_ids = ["target"]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = data.xpos[1].tolist()
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = data.xquat[1].tolist()
            message.poses = [pose]
            MotionFixture._state(fixture, message)
            if commands:
                wrench = commands[-1].wrench
                data.xfrc_applied[1] = [wrench.force.x, wrench.force.y, wrench.force.z,
                                       wrench.torque.x, wrench.torque.y, wrench.torque.z]
            if fixture.phase == "hold" and not moved:
                moved = True
                MotionFixture._start(fixture, String(data="target"))
            if fixture.phase == "moving":
                max_speed = max(max_speed, float(np.linalg.norm(data.qvel[:3])))
            if fixture.phase == "released":
                break
        self.assertIn("ready:target", events)
        self.assertIn("started:target", events)
        self.assertIn("released:target", events)
        self.assertLess(max_speed, 0.01)
        self.assertGreater(float(data.xpos[1][0]), 0.005)
        self.assertAlmostEqual(float(data.xpos[1][2]), 0.02, delta=0.002)
