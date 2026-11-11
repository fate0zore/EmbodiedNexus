"""MuJoCo-only force fixture for repeatable decelerating target motion."""

import math

import numpy as np
import rclpy
from embodied_nexus_sim_mujoco_interfaces.msg import SceneObjectStates
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import String

from .tracking import _multiply, _rotation_delta, _unit_quaternion


class MotionFixture(Node):
    """Drive an experimental object with a bounded force until release."""

    def __init__(self):
        """Initialize the isolated simulator truth and wrench connections.

        Returns:
            None.
        """
        super().__init__("dynamic_grasp_motion_fixture")
        self.declare_parameter("object_id", "cylinder-1")
        self.declare_parameter("mass_kg", 0.1)
        self.declare_parameter("duration_sec", 4.0)
        self.declare_parameter("initial_velocity_m_sec", [0.007, 0.002, -0.005])
        self.declare_parameter("initial_yaw_rate_rad_sec", 0.04)
        self.declare_parameter("scenario", "combined")
        self.object_id = self.get_parameter("object_id").value
        self.mass = float(self.get_parameter("mass_kg").value)
        self.duration = float(self.get_parameter("duration_sec").value)
        self.initial_velocity = np.asarray(self.get_parameter("initial_velocity_m_sec").value, dtype=float)
        self.yaw_rate = float(self.get_parameter("initial_yaw_rate_rad_sec").value)
        self.scenario = self.get_parameter("scenario").value
        if self.scenario == "tabletop":
            self.duration = 175.0
            # Move towards the robot rather than beyond its forward reach.
            self.initial_velocity = np.array([-0.01, 0.0, 0.0])
            self.yaw_rate = 0.0
        elif self.scenario == "translation":
            self.yaw_rate = 0.0
        elif self.scenario == "rotation":
            self.initial_velocity = np.array([0.0, 0.0, -0.005])
        elif self.scenario != "combined":
            raise ValueError("unknown fixture scenario")
        if (self.mass <= 0 or self.duration <= 0 or self.initial_velocity.shape != (3,) or
                not np.isfinite(self.initial_velocity).all() or
                np.linalg.norm(self.initial_velocity) > 0.01 or abs(self.yaw_rate) > math.radians(5)):
            raise ValueError("invalid fixture motion parameters")
        self.phase = "idle"
        self.start_stamp = None
        self.previous = None
        self.origin = None
        self.origin_orientation = None
        self.hold_position = None
        self.support_height = None
        self.ready_since = None
        self.last_report = -1.0
        self.publisher = self.create_publisher(WrenchStamped, "/sim/arachne/mujoco/test_object_wrench", 10)
        self.events = self.create_publisher(String, "/embodied_nexus/dynamic_grasp/fixture_event", 10)
        self.create_subscription(String, "/embodied_nexus/dynamic_grasp/fixture_prepare", self._prepare, 10)
        self.create_subscription(String, "/embodied_nexus/dynamic_grasp/fixture_start", self._start, 10)
        self.create_subscription(String, "/embodied_nexus/dynamic_grasp/fixture_release", self._stop, 10)
        self.create_subscription(SceneObjectStates, "/sim/arachne/ground_truth/scene_objects", self._state, 10)

    def _prepare(self, message):
        """Raise and hold the free object before starting the deceleration trial.

        Args:
            message: Object ID matching the configured experiment.

        Returns:
            None.
        """
        if message.data != self.object_id or self.phase not in ("idle", "released"):
            return
        self.phase = "prepare"
        self.previous = self.hold_position = self.support_height = self.ready_since = None
        self.events.publish(String(data="preparing:" + self.object_id))

    def _start(self, message):
        """Start a decelerating trajectory from the prepared hover pose.

        Args:
            message: Object ID matching the configured experiment.

        Returns:
            None.
        """
        if message.data != self.object_id or self.phase != "hold":
            return
        self.phase = "moving"
        self.start_stamp = self.origin = self.origin_orientation = None
        self.events.publish(String(data="armed:" + self.object_id))

    def _release(self, outcome):
        """Remove the external wrench and report a fixture outcome.

        Args:
            outcome: Event prefix describing release or a test abort.

        Returns:
            None.
        """
        self.phase = "released"
        command = WrenchStamped()
        command.header.frame_id = self.object_id
        self.publisher.publish(command)
        self.events.publish(String(data=outcome + ":" + self.object_id))

    def _stop(self, message):
        """Stop the test fixture and release any object it was holding.

        Args:
            message: Object ID matching the configured experiment.

        Returns:
            None.
        """
        if message.data == self.object_id and self.phase not in ("idle", "released"):
            self._release("aborted_cancel")

    def _state(self, message):
        """Track a slow 3-D reference using force, then release at support level.

        Args:
            message: Simulator ground-truth object state, used only by this fixture.

        Returns:
            None.
        """
        if self.phase in ("idle", "released") or self.object_id not in message.object_ids:
            return
        index = list(message.object_ids).index(self.object_id)
        pose = message.poses[index]
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1e9
        position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        orientation = _unit_quaternion([pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w])
        if self.previous is None:
            self.previous = (stamp, position, orientation)
            if self.phase == "prepare":
                self.support_height = float(position[2])
                self.hold_position = position + np.array([0.0, 0.0, 0.0 if getattr(self, "scenario", "combined") == "tabletop" else 0.01])
                self.origin_orientation = orientation
            return
        dt = stamp - self.previous[0]
        if dt <= 0 or dt > 0.5:
            self._release("aborted_clock")
            return
        measured_velocity = (position - self.previous[1]) / dt
        measured_angular = _rotation_delta(orientation, self.previous[2], "box") / dt
        if getattr(self, "scenario", "combined") == "tabletop":
            self._tabletop_state(stamp, position, orientation, measured_velocity, measured_angular)
            return
        if stamp - self.last_report >= 2.0:
            self.get_logger().info(
                f"fixture phase={self.phase} position_m={position.tolist()} "
                f"speed_m_sec={float(np.linalg.norm(measured_velocity)):.5f}")
            self.last_report = stamp
        if self.phase in ("prepare", "hold"):
            desired = self.hold_position
            desired_velocity = np.zeros(3)
            desired_orientation = self.origin_orientation
            desired_angular = np.zeros(3)
            if self.phase == "prepare":
                if np.linalg.norm(desired - position) < 0.0015 and np.linalg.norm(measured_velocity) < 0.003:
                    if self.ready_since is None:
                        self.ready_since = stamp
                    if stamp - self.ready_since >= 0.5:
                        self.phase = "hold"
                        self.events.publish(String(data="ready:" + self.object_id))
                else:
                    self.ready_since = None
        else:
            if self.start_stamp is None:
                self.start_stamp, self.origin, self.origin_orientation = stamp, position, orientation
                self.events.publish(String(data="started:" + self.object_id))
            elapsed = stamp - self.start_stamp
            if elapsed < 0 or elapsed > self.duration + 10.0:
                self._release("aborted_timeout")
                return
            trajectory_time = min(elapsed, self.duration)
            displacement_factor = trajectory_time - trajectory_time * trajectory_time / (2 * self.duration)
            desired = self.origin + self.initial_velocity * displacement_factor
            desired_velocity = self.initial_velocity * max(0.0, 1 - elapsed / self.duration)
            angle = self.yaw_rate * displacement_factor
            rotation = np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
            desired_orientation = _multiply(rotation, self.origin_orientation)
            desired_angular = np.array([0.0, 0.0, self.yaw_rate * max(0.0, 1 - elapsed / self.duration)])
            if elapsed >= self.duration:
                # A small downward reference maintains contact until release;
                # the free rigid body, not the fixture, determines support.
                desired[2] = self.support_height - 0.001
            if (elapsed >= self.duration and position[2] <= self.support_height + 0.0006 and
                    np.linalg.norm(measured_velocity) < 0.005):
                self._release("released")
                return
        position_error = desired - position
        speed_reference = desired_velocity + 1.5 * position_error
        speed_limit = 0.004 if self.phase == "prepare" else 0.009
        speed_reference *= min(1.0, speed_limit / max(np.linalg.norm(speed_reference), 1e-9))
        acceleration = 3.0 * (speed_reference - measured_velocity)
        acceleration *= min(1.0, 0.05 / max(np.linalg.norm(acceleration), 1e-9))
        # The shared MuJoCo Arachne model sets gravity to exactly 9.8 m/s².
        force = self.mass * (np.array([0.0, 0.0, 9.8]) + acceleration)
        orientation_error = _rotation_delta(desired_orientation, orientation, "box")
        angular_reference = desired_angular + 1.5 * orientation_error
        angular_reference *= min(1.0, 0.07 / max(np.linalg.norm(angular_reference), 1e-9))
        torque = 0.0001 * (angular_reference - measured_angular)
        torque *= min(1.0, 0.00003 / max(np.linalg.norm(torque), 1e-9))
        command = WrenchStamped()
        command.header.stamp = message.header.stamp
        command.header.frame_id = self.object_id
        command.wrench.force.x, command.wrench.force.y, command.wrench.force.z = force.tolist()
        command.wrench.torque.x, command.wrench.torque.y, command.wrench.torque.z = torque.tolist()
        self.publisher.publish(command)
        self.previous = (stamp, position, orientation)

    def _tabletop_state(self, stamp, position, orientation, velocity, angular):
        """Move a free body along its support with bounded horizontal force.

        Args:
            stamp: Simulation timestamp in seconds.
            position: Measured world position in meters.
            orientation: Measured orientation quaternion.
            velocity: Measured linear velocity in meters per second.
            angular: Measured angular velocity in radians per second.

        Returns:
            None.
        """
        if self.phase == "prepare":
            if np.linalg.norm(velocity) < 0.003 and np.linalg.norm(angular) < math.radians(3):
                if self.ready_since is None:
                    self.ready_since = stamp
                if stamp - self.ready_since >= 0.5:
                    self.phase = "hold"
                    self.events.publish(String(data="ready:" + self.object_id))
            else:
                self.ready_since = None
        elif self.phase == "moving":
            if self.start_stamp is None:
                self.start_stamp, self.origin = stamp, position.copy()
                self.events.publish(String(data="started:" + self.object_id))
            elapsed = stamp - self.start_stamp
            if elapsed >= self.duration:
                self._release("released")
                return
            # Counter contact friction while tracking the decelerating speed.
            reference = self.initial_velocity[0] * (1.0 - elapsed / self.duration)
            friction = math.copysign(self.mass * 9.8 * 0.001, reference)
            force_x = np.clip(friction + self.mass * 8.0 * (reference - velocity[0]),
                              -0.02, 0.02)
            command = WrenchStamped()
            command.header.stamp.sec = int(stamp)
            command.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
            command.header.frame_id = self.object_id
            command.wrench.force.x = float(force_x)
            self.publisher.publish(command)
        self.previous = (stamp, position, orientation)


def main(args=None):
    """Spin the MuJoCo-only test fixture until shutdown.

    Args:
        args: Optional ROS arguments.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = MotionFixture()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
