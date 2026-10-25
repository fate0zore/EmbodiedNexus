from __future__ import annotations

import argparse
import inspect
import json
import os
import re
import signal
import threading
import time

import rclpy
import uvicorn
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from embodied_nexus_core_interfaces.srv import (
    GetCoreStatus, ManageAllCoreDevices, ManageCoreDevice,
    ValidatePlatformConfiguration,
)

from .config import default_config_path, load_platform, public_platform
from .manager import DeviceManager
from .visualization import RosbridgeRuntime
from .web_server import create_app


class CoreNode(Node):
    def __init__(self, config_file: str | None = None):
        super().__init__("embodied_nexus_core")
        self.declare_parameter("config_file", config_file or str(default_config_path()))
        self.config_file = str(self.get_parameter("config_file").value)
        self.manager = DeviceManager(self.config_file)
        core = self.manager.platform["core"]
        self.visualization = RosbridgeRuntime(
            self.manager.platform["visualization"], self.get_topic_names_and_types,
        )
        if self.visualization.enabled and not self.visualization.ready():
            self.get_logger().warning(self.visualization.info()["error"])
        token = os.environ.get(str(core["auth_token_env"]), "")
        if not token or not re.fullmatch(r"[A-Za-z0-9._~-]+", token):
            self.visualization.close()
            raise RuntimeError(f"set {core['auth_token_env']} before starting the core")
        self.app = create_app(self.manager, token, self.visualization)
        web_options = {
            "host": str(core["bind_host"]),
            "port": int(core["port"]),
            "log_level": "info",
            "access_log": True,
        }
        # Ubuntu 22.04's Uvicorn 0.15 predates ws_max_queue. Keep the bounded
        # queue where supported while allowing the system package on Humble.
        if "ws_max_queue" in inspect.signature(uvicorn.Config).parameters:
            web_options["ws_max_queue"] = 2
        self.web_server = uvicorn.Server(uvicorn.Config(self.app, **web_options))
        self.web_thread = threading.Thread(target=self.web_server.run, name="embodied-nexus-core-web", daemon=True)
        self.web_thread.start()
        deadline = time.monotonic() + 5.0
        while not self.web_server.started and self.web_thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not self.web_server.started:
            self.web_server.should_exit = True
            self.web_thread.join(timeout=1.0)
            self.manager.close_monitor()
            self.visualization.close()
            self.destroy_node()
            raise RuntimeError(f"core HTTP gateway failed to listen on {core['bind_host']}:{core['port']}")
        self.manager.start_monitor()
        self.status_pub = self.create_publisher(String, "/embodied_nexus/core/status", 10)
        self.create_service(GetCoreStatus, "/embodied_nexus/core/get_status", self._get_status)
        self.create_service(ManageCoreDevice, "/embodied_nexus/core/manage_device", self._manage_device)
        self.create_service(ManageAllCoreDevices, "/embodied_nexus/core/manage_all", self._manage_all)
        self.create_service(ValidatePlatformConfiguration, "/embodied_nexus/core/config/validate", self._validate_config)
        self._shutdown_requested = threading.Event()
        self._shutdown_guard = threading.Lock()
        self._shutdown_result: dict | None = None
        self._shutdown_in_progress = False
        self.timer = self.create_timer(float(core["poll_interval_sec"]), self._publish_status)
        self.get_logger().info(f"EmbodiedNexus core HTTP gateway listening on {core['bind_host']}:{core['port']}")

    def _get_status(self, _request, response):
        response.state_json = json.dumps(self.manager.status(), ensure_ascii=False, allow_nan=False)
        return response

    def _manage_device(self, request, response):
        try:
            action, device_id = str(request.action), str(request.device_id)
            if action == "start":
                result = self.manager.start(device_id)
            elif action == "shutdown":
                result = self.manager.shutdown(device_id)
            elif action == "restart":
                result = self.manager.restart(device_id)
            else:
                raise ValueError("action must be start, shutdown or restart")
            response.success = bool(result.get("success"))
            response.result_json = json.dumps(result, ensure_ascii=False, allow_nan=False)
            response.error = "" if response.success else str(result.get("error", "operation failed"))
        except Exception as exc:
            response.success = False
            response.result_json = "{}"
            response.error = str(exc)
        return response

    def _manage_all(self, request, response):
        try:
            action = str(request.action)
            if action == "start_all":
                results = self.manager.start_all()
            elif action == "shutdown_all":
                results = self.manager.shutdown_all()
            elif action == "stop_all":
                results = self.manager.stop_all()
            elif action == "shutdown_core":
                result = self.manager.request_core_shutdown()
                if result["success"]:
                    self.request_shutdown()
                response.success = bool(result["success"])
                response.results_json = json.dumps(result["results"], ensure_ascii=False, allow_nan=False)
                response.error = "" if response.success else "one or more device stops were not confirmed"
                return response
            else:
                raise ValueError("action must be start_all, shutdown_all, stop_all or shutdown_core")
            response.success = all(item.get("success", False) for item in results)
            response.results_json = json.dumps(results, ensure_ascii=False, allow_nan=False)
            response.error = "" if response.success else "one or more device operations failed"
        except Exception as exc:
            response.success = False
            response.results_json = "[]"
            response.error = str(exc)
        return response

    def _validate_config(self, request, response):
        path = str(request.config_file or self.config_file)
        try:
            checked = load_platform(path)
            response.valid = True
            response.diagnostics_json = json.dumps({"device_count": len(checked["devices"]),
                                                     "config_file": checked["config_file"]}, ensure_ascii=False)
            response.error = ""
        except Exception as exc:
            response.valid = False
            response.diagnostics_json = "{}"
            response.error = str(exc)
        return response

    def _publish_status(self):
        message = String()
        message.data = json.dumps(self.manager.status(), ensure_ascii=False, allow_nan=False,
                                  separators=(",", ":"))
        self.status_pub.publish(message)

    def request_shutdown(self):
        self._shutdown_requested.set()

    def shutdown_if_requested(self) -> bool:
        if not self._shutdown_requested.is_set():
            return False
        with self._shutdown_guard:
            if self._shutdown_in_progress:
                return False
            self._shutdown_in_progress = True
        try:
            result = self.manager.request_core_shutdown()
            self._shutdown_result = result
            if not result["success"]:
                self.get_logger().error("core shutdown paused because a device stop was not confirmed; management remains available")
                self._shutdown_requested.clear()
                return False
            return True
        finally:
            with self._shutdown_guard:
                self._shutdown_in_progress = False

    def close(self):
        self.web_server.should_exit = True
        if self.web_thread.is_alive():
            self.web_thread.join(timeout=3.0)
        self.manager.close_monitor()
        self.visualization.close()
        self.destroy_node()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run the EmbodiedNexus core device manager")
    parser.add_argument("--config", default=None, help="platform YAML; defaults to platform_mock.yaml")
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = CoreNode(args.config)
    except Exception as exc:
        print(f"nexus_core: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        return 2

    signal_event = threading.Event()
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.getsignal(sig)
        signal.signal(sig, lambda *_args: signal_event.set())
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.2)
            if signal_event.is_set():
                signal_event.clear()
                node.request_shutdown()
            if node.app.state.shutdown_requested.is_set():
                node.app.state.shutdown_requested.clear()
                node.request_shutdown()
            if node.shutdown_if_requested():
                break
        return 0
    except KeyboardInterrupt:
        node.request_shutdown()
        return 0 if node.shutdown_if_requested() else 1
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if node._shutdown_result and node._shutdown_result.get("success"):
            executor.shutdown(timeout_sec=3.0)
            node.close()
            if rclpy.ok():
                rclpy.shutdown()
        else:
            # A failed stop keeps the manager and gateway alive for recovery.
            if not rclpy.ok():
                node.get_logger().critical("core stopped without closing managed hardware processes")


if __name__ == "__main__":
    raise SystemExit(main())
