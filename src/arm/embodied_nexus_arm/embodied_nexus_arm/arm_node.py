from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from embodied_nexus_arm_interfaces.action import MoveArmJoints
from embodied_nexus_arm_interfaces.srv import (
    AcquireArmControl, ApplyArmConfiguration, GetArmConfiguration, GetArmInfo,
    GetArmState, ReleaseArmControl, RenewArmControl, SetArmJointVelocity,
    SetArmLifecycle, SetArmMotionEnabled, SetArmTeachMode, StopArm,
    ValidateArmConfiguration,
)

from .config import load_config
from .controller import ArmController


class ArmNode(Node):
    def __init__(self, config_path: str | None = None) -> None:
        super().__init__("embodied_nexus_arm")
        self.declare_parameter("config_file", config_path or str(Path(__file__).resolve().parents[1] / "config" / "aubo_i5_mock.yaml"))
        self.config = load_config(str(self.get_parameter("config_file").value))
        self.controller = ArmController(self.config)
        self.device_id = self.config["device_id"]
        self.prefix = f"/embodied_nexus/arms/{self.device_id}"
        self.group = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, f"{self.prefix}/_internal/state_json", 10)
        self.joints_pub = self.create_publisher(JointState, f"{self.prefix}/joint_states", 10)
        self._arm_services = [
            self.create_service(GetArmInfo, f"{self.prefix}/info", self._info, callback_group=self.group),
            self.create_service(GetArmState, f"{self.prefix}/get_state", self._state, callback_group=self.group),
            self.create_service(GetArmConfiguration, f"{self.prefix}/config/get", self._get_config, callback_group=self.group),
            self.create_service(ValidateArmConfiguration, f"{self.prefix}/config/validate", self._validate, callback_group=self.group),
            self.create_service(ApplyArmConfiguration, f"{self.prefix}/config/apply", self._apply, callback_group=self.group),
            self.create_service(AcquireArmControl, f"{self.prefix}/control/acquire", self._acquire, callback_group=self.group),
            self.create_service(RenewArmControl, f"{self.prefix}/control/renew", self._renew, callback_group=self.group),
            self.create_service(ReleaseArmControl, f"{self.prefix}/control/release", self._release, callback_group=self.group),
            self.create_service(SetArmMotionEnabled, f"{self.prefix}/motion/enable", self._enable, callback_group=self.group),
            self.create_service(SetArmJointVelocity, f"{self.prefix}/joint_velocity", self._velocity, callback_group=self.group),
            self.create_service(SetArmLifecycle, f"{self.prefix}/lifecycle", self._lifecycle, callback_group=self.group),
            self.create_service(SetArmTeachMode, f"{self.prefix}/teach_mode", self._teach, callback_group=self.group),
            self.create_service(StopArm, f"{self.prefix}/stop", self._stop, callback_group=self.group),
        ]
        self.move_action = ActionServer(self, MoveArmJoints, f"{self.prefix}/move_joints",
                                        execute_callback=self._move_joints,
                                        goal_callback=self._move_goal,
                                        cancel_callback=self._cancel_goal,
                                        callback_group=self.group)
        hz = float(self.config["feedback"]["rate_hz"])
        self.timer = self.create_timer(1.0 / hz, self._tick, callback_group=self.group)

    @staticmethod
    def _json(value) -> str:
        return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))

    @staticmethod
    def _ok(response, error=""):
        response.success = not bool(error)
        response.error = str(error)
        return response

    def _info(self, _request, response):
        try:
            response.descriptor_json = self._json(self.controller.info())
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _state(self, _request, response):
        try:
            response.state_json = self._json(self.controller.state())
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _get_config(self, _request, response):
        try:
            response.config_json = self._json(self.controller.configuration())
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _validate(self, request, response):
        try:
            response.config_json = self._json(self.controller.validate_configuration(json.loads(request.patch_json)))
            response.valid = True
            response.error = ""
        except Exception as exc:
            response.valid = False
            response.error = str(exc)
        return response

    def _apply(self, request, response):
        try:
            response.config_json = self._json(self.controller.apply_configuration(
                json.loads(request.patch_json), request.lease_id))
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _acquire(self, request, response):
        try:
            response.lease_id, response.expires_in_sec = self.controller.acquire_control(
                request.owner, request.ttl_sec or None)
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _renew(self, request, response):
        try:
            response.expires_in_sec = self.controller.renew_control(request.lease_id)
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _release(self, request, response):
        try:
            result = self.controller.release_control(request.lease_id)
            response.stop_requested = result["stop_requested"]
            response.controller_accepted = result["controller_accepted"]
            response.stationary_confirmed = result["stationary_confirmed"]
            response.success = True
            response.error = result["error"]
            return response
        except Exception as exc:
            return self._ok(response, exc)

    def _enable(self, request, response):
        try:
            response.enabled = self.controller.enable_motion(request.lease_id, request.enabled)["enabled"]
            return self._ok(response)
        except Exception as exc:
            response.enabled = False
            return self._ok(response, exc)

    def _velocity(self, request, response):
        try:
            result = self.controller.set_joint_velocity(request.lease_id, request.velocity_rad_sec)
            response.controller_accepted = result["controller_accepted"]
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _lifecycle(self, request, response):
        try:
            result = self.controller.set_lifecycle(request.lease_id, request.command)
            response.operation_state = result["operation_state"]
            return self._ok(response)
        except Exception as exc:
            response.operation_state = "failed"
            return self._ok(response, exc)

    def _teach(self, request, response):
        try:
            result = self.controller.set_teach_mode(request.lease_id, request.enabled)
            response.teach_state = result["teach_state"]
            return self._ok(response)
        except Exception as exc:
            response.teach_state = "unknown"
            return self._ok(response, exc)

    def _stop(self, request, response):
        try:
            result = self.controller.stop(request.reason or "ROS stop requested")
            response.stop_requested = result["stop_requested"]
            response.controller_accepted = result["controller_accepted"]
            response.stationary_confirmed = result["stationary_confirmed"]
            response.hardware_estop = result["hardware_estop"]
            response.success = True
            response.error = result["error"]
            return response
        except Exception as exc:
            return self._ok(response, exc)

    def _move_goal(self, request):
        try:
            ArmController._validate_vector(request.positions_rad, "positions_rad")
            ArmController._validate_vector([request.speed_rad_sec or self.config["motion"]["max_speed_rad_sec"]] * 6,
                                           "speed_rad_sec")
            self.controller._require_lease(request.lease_id)
            return GoalResponse.ACCEPT if self.controller.motion_enabled else GoalResponse.REJECT
        except Exception as exc:
            self.get_logger().warning(f"Rejected arm joint goal: {exc}")
            return GoalResponse.REJECT

    @staticmethod
    def _cancel_goal(_goal_handle):
        return CancelResponse.ACCEPT

    def _move_joints(self, goal_handle):
        request = goal_handle.request
        result = MoveArmJoints.Result()
        try:
            outcome = self.controller.move_joints(
                request.lease_id, request.positions_rad, request.speed_rad_sec,
                request.acceleration_rad_sec2, request.timeout_sec,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                feedback=lambda state, error: self._publish_action_feedback(goal_handle, state, error))
            result.success = outcome["success"]
            result.message = outcome["message"]
            result.max_error_rad = outcome["max_error_rad"]
            result.controller_accepted = outcome["controller_accepted"]
            result.stop_requested = outcome["stop_requested"]
            result.stationary_confirmed = outcome["stationary_confirmed"]
            result.hardware_estop = outcome["hardware_estop"]
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except Exception as exc:
            result.success = False
            result.message = str(exc)
            result.max_error_rad = 0.0
            result.controller_accepted = self.controller.controller_accepted
            result.stop_requested = self.controller.stop_requested
            result.stationary_confirmed = self.controller.stationary_confirmed
            result.hardware_estop = bool(self.controller.feedback.get("hardware_estop", False))
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
        return result

    @staticmethod
    def _publish_action_feedback(goal_handle, state, error):
        feedback = MoveArmJoints.Feedback()
        feedback.state = str(state)
        feedback.max_error_rad = float(error if math.isfinite(error) else 0.0)
        goal_handle.publish_feedback(feedback)

    def _tick(self):
        self.controller.tick()
        state = self.controller.state()
        self.state_pub.publish(String(data=self._json(state)))
        positions, velocities = state.get("positions_rad"), state.get("velocities_rad_sec")
        if state["feedback_valid"] and positions is not None and velocities is not None:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self.config["joints"]["names"])
            message.position = list(positions)
            message.velocity = list(velocities)
            self.joints_pub.publish(message)

    def destroy_node(self):
        try:
            self.controller.shutdown()
        finally:
            self.move_action.destroy()
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ArmNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=4.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
