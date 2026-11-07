"""Authenticated local HTTP trigger that forwards requests to the grasp action."""

from __future__ import annotations

import hmac
import json
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from embodied_nexus_operator_interfaces.action import PlanGrasp


class OperatorHttpServer:
    """Expose create, inspect, and cancel operations for grasp action tasks."""

    def __init__(self, node, action_client, host: str, port: int, token: str) -> None:
        """Create a local HTTP server with an independent bearer token.

        Args:
            node: ROS node used for logging.
            action_client: ROS ActionClient for PlanGrasp.
            host: HTTP bind address.
            port: HTTP listen port.
            token: Non-empty bearer token for HTTP access.

        Returns:
            None.
        """
        if not token:
            raise ValueError("HTTP trigger requires EMBODIED_NEXUS_OPERATOR_HTTP_TOKEN")
        self.node = node
        self.action_client = action_client
        self.host = host
        self.port = int(port)
        self.token = token
        self.tasks: dict[str, dict] = {}
        self.lock = threading.RLock()
        self.httpd = ThreadingHTTPServer((self.host, self.port), self._handler_type())
        self.httpd.owner = self
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       name="grasp-operator-http", daemon=True)
        self.thread.start()

    def _handler_type(self):
        """Create the request handler bound to this server instance.

        Args:
            None.

        Returns:
            A BaseHTTPRequestHandler subclass for task endpoints.
        """
        owner = self

        class Handler(BaseHTTPRequestHandler):
            """Handle authenticated JSON task requests."""

            def do_POST(self):
                """Create a grasp task from a JSON request body.

                Args:
                    None.

                Returns:
                    None.
                """
                if not self._authorized():
                    return self._write(401, {"error": "unauthorized"})
                path = urlparse(self.path).path.rstrip("/")
                if path not in {"/plan", "/tasks"}:
                    return self._write(404, {"error": "route not found"})
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if length <= 0 or length > 16384:
                        raise ValueError("request body must be between 1 and 16384 bytes")
                    payload = json.loads(self.rfile.read(length))
                    if not isinstance(payload, dict):
                        raise ValueError("request body must be a JSON object")
                    response = owner.create_task(payload)
                    return self._write(202, response)
                except (ValueError, json.JSONDecodeError) as exc:
                    return self._write(400, {"error": str(exc)})
                except Exception as exc:
                    return self._write(503, {"error": str(exc)})

            def do_GET(self):
                """Return the stored state and action feedback for a task.

                Args:
                    None.

                Returns:
                    None.
                """
                if not self._authorized():
                    return self._write(401, {"error": "unauthorized"})
                parts = urlparse(self.path).path.strip("/").split("/")
                if len(parts) != 2 or parts[0] != "tasks":
                    return self._write(404, {"error": "route not found"})
                task = owner.get_task(parts[1])
                return self._write(200, task) if task else self._write(404, {"error": "task not found"})

            def do_DELETE(self):
                """Cancel a running grasp task.

                Args:
                    None.

                Returns:
                    None.
                """
                if not self._authorized():
                    return self._write(401, {"error": "unauthorized"})
                parts = urlparse(self.path).path.strip("/").split("/")
                if len(parts) != 2 or parts[0] != "tasks":
                    return self._write(404, {"error": "route not found"})
                try:
                    return self._write(202, owner.cancel_task(parts[1]))
                except KeyError:
                    return self._write(404, {"error": "task not found"})
                except Exception as exc:
                    return self._write(409, {"error": str(exc)})

            def _authorized(self):
                """Check the configured HTTP bearer token in constant time.

                Args:
                    None.

                Returns:
                    True if the request carries the configured bearer token.
                """
                supplied = self.headers.get("Authorization", "")
                expected = f"Bearer {owner.token}"
                return hmac.compare_digest(supplied.encode(), expected.encode())

            def _write(self, status: int, payload: dict):
                """Write one compact JSON HTTP response.

                Args:
                    status: HTTP status code.
                    payload: JSON serializable response mapping.

                Returns:
                    None.
                """
                encoded = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(encoded)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(encoded)

            def log_message(self, _format, *_args):
                """Suppress default HTTP access logs that could expose request bodies.

                Args:
                    _format: Ignored base handler log format.
                    *_args: Ignored base handler log arguments.

                Returns:
                    None.
                """
                return None

        return Handler

    def create_task(self, payload: dict) -> dict:
        """Submit one grasp action goal and return its task ID.

        Args:
            payload: Goal fields accepted by the PlanGrasp action.

        Returns:
            Task identifier and initial state.
        """
        if not self.action_client.server_is_ready() and not self.action_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("grasp action server is unavailable")
        goal = PlanGrasp.Goal()
        default_class = str(getattr(self.node, "config", {}).get("class_prompt", "box"))
        goal.target_class = str(payload.get("target_class", default_class))
        goal.object_id = str(payload.get("object_id", getattr(self.node, "config", {}).get("object_id", "target_box")))
        goal.camera_id = str(payload.get("camera_id", "camera_front"))
        goal.arm_id = str(payload.get("arm_id", "arm_main"))
        goal.gripper_id = str(payload.get("gripper_id", "gripper_main"))
        task_id = uuid.uuid4().hex
        record = {"task_id": task_id, "state": "submitting", "feedback": {}, "result": None,
                  "goal_handle": None}
        with self.lock:
            self.tasks[task_id] = record
        future = self.action_client.send_goal_async(
            goal, feedback_callback=lambda message: self._feedback(task_id, message))
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(10.0):
            with self.lock:
                record["state"] = "rejected"
                record["result"] = {"message": "timed out waiting for goal acceptance"}
            raise TimeoutError("timed out waiting for grasp action acceptance")
        handle = future.result()
        if not handle.accepted:
            with self.lock:
                record["state"] = "rejected"
                record["result"] = {"message": "goal rejected"}
            raise RuntimeError("grasp goal was rejected")
        action_goal_id = bytes(handle.goal_id.uuid).hex()
        with self.lock:
            record["goal_handle"] = handle
            record["state"] = "accepted"
            record["action_goal_id"] = action_goal_id
        self.node.get_logger().info(
            f"HTTP grasp task accepted task_id={task_id} action_goal_id={action_goal_id} "
            f"target_class={goal.target_class} object_id={goal.object_id}")
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda completed: self._result(task_id, completed))
        return {"task_id": task_id, "state": "accepted"}

    def _feedback(self, task_id: str, message) -> None:
        """Store the most recent ROS action feedback for an HTTP task.

        Args:
            task_id: Local HTTP task identifier.
            message: ROS feedback wrapper containing the PlanGrasp feedback.

        Returns:
            None.
        """
        feedback = message.feedback
        phase_changed = False
        action_goal_id = ""
        with self.lock:
            record = self.tasks.get(task_id)
            if record:
                phase_changed = record.get("feedback", {}).get("phase") != str(feedback.phase)
                action_goal_id = record.get("action_goal_id", "")
                record["state"] = str(feedback.phase)
                record["feedback"] = {"phase": str(feedback.phase), "detail": str(feedback.detail),
                                      "detection_confidence": float(feedback.detection_confidence)}
        if phase_changed:
            self.node.get_logger().info(
                f"HTTP grasp task task_id={task_id} action_goal_id={action_goal_id} "
                f"phase={feedback.phase} detail={feedback.detail} "
                f"detection_confidence={float(feedback.detection_confidence):.3f}")

    def _result(self, task_id: str, future) -> None:
        """Store the terminal ROS action result for an HTTP task.

        Args:
            task_id: Local HTTP task identifier.
            future: Completed action result future.

        Returns:
            None.
        """
        try:
            wrapped = future.result()
            value = wrapped.result
            result = {"success": bool(value.success), "grasp_verified": bool(value.grasp_verified),
                      "message": str(value.message),
                      "object_pose": self._pose_stamped(value.object_pose),
                      "grasp_pose": self._pose_stamped(value.grasp_pose)}
            state = "succeeded" if value.success else "failed"
        except Exception as exc:
            result = {"success": False, "grasp_verified": False, "message": str(exc)}
            state = "failed"
        action_goal_id = ""
        with self.lock:
            record = self.tasks.get(task_id)
            if record:
                record["state"] = state
                record["result"] = result
                record["goal_handle"] = None
                action_goal_id = record.get("action_goal_id", "")
        self.node.get_logger().info(
            f"HTTP grasp task finished task_id={task_id} action_goal_id={action_goal_id} "
            f"state={state} success={result['success']} grasp_verified={result['grasp_verified']} "
            f"message={result['message']}")

    @staticmethod
    def _pose_stamped(value) -> dict:
        """Convert one PoseStamped ROS value into a JSON-safe mapping.

        Args:
            value: PoseStamped action result field.

        Returns:
            Frame, timestamp, position, and quaternion mapping.
        """
        stamp = value.header.stamp
        pose = value.pose
        return {
            "frame_id": str(value.header.frame_id),
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "position": {"x": float(pose.position.x), "y": float(pose.position.y),
                         "z": float(pose.position.z)},
            "orientation": {"x": float(pose.orientation.x), "y": float(pose.orientation.y),
                            "z": float(pose.orientation.z), "w": float(pose.orientation.w)},
        }

    def get_task(self, task_id: str) -> dict | None:
        """Return a JSON-safe snapshot of one task.

        Args:
            task_id: Local HTTP task identifier.

        Returns:
            Task snapshot or None when the identifier is unknown.
        """
        with self.lock:
            record = self.tasks.get(task_id)
            if record is None:
                return None
            return {key: value for key, value in record.items() if key != "goal_handle"}

    def cancel_task(self, task_id: str) -> dict:
        """Request cancellation for a running grasp action.

        Args:
            task_id: Local HTTP task identifier.

        Returns:
            Task identifier and cancellation request state.
        """
        with self.lock:
            record = self.tasks.get(task_id)
            if record is None:
                raise KeyError(task_id)
            handle = record.get("goal_handle")
            if handle is None:
                raise RuntimeError("task has no active goal to cancel")
        handle.cancel_goal_async()
        return {"task_id": task_id, "state": "cancel_requested"}

    def close(self) -> None:
        """Stop the HTTP listener and release its server socket.

        Args:
            None.

        Returns:
            None.
        """
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=2.0)
