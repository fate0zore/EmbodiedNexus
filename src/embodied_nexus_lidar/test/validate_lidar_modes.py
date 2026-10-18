"""Humble acceptance smoke for mock/topics backends and ROS/Web modes.

Run after sourcing the workspace: python3 src/embodied_nexus_lidar/test/validate_lidar_modes.py
This script never launches the real C16 backend or sends hardware commands.
"""

from __future__ import annotations

import array
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(ROOT / "sdk" / "python"))

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from embodied_nexus_camera_client import LidarClient


PREFIX = "/embodied_nexus/lidars/lidar_main"
TOKEN = "lidar-smoke-token"


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def external_cloud(node: Node) -> PointCloud2:
    message = PointCloud2()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "lidar_link"
    message.height, message.width = 1, 1
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step, message.row_step = 12, 12
    message.data = array.array("B", struct.pack("<fff", 2.0, 0.0, 0.0))
    message.is_dense = True
    return message


def wait_for_cloud(process, node, received, publish=None, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("lidar launch exited before publishing a point cloud")
        if publish is not None:
            publish()
        rclpy.spin_once(node, timeout_sec=0.1)
        if received:
            return received[-1]
    raise TimeoutError("timed out waiting for the wrapped PointCloud2 output")


def wait_for_web(process, url, token, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("lidar launch exited before starting the Web API")
        try:
            request = urllib.request.Request(url, headers={"Authorization": f"Bearer {token}"})
            with urllib.request.urlopen(request, timeout=1.0) as response:
                if response.status == 200:
                    return
        except (OSError, urllib.error.URLError):
            time.sleep(0.2)
    raise TimeoutError(f"timed out waiting for the Web API at {url}")


def run_launch(harness: Node, *, config: Path, mode: str, port: int | None,
               verify_web: bool, publish_external: bool = False) -> dict:
    received = []
    output_base = PREFIX if mode != "web" else f"{PREFIX}/_internal"
    subscription = harness.create_subscription(
        PointCloud2, f"{output_base}/points", received.append, qos_profile_sensor_data
    )
    publisher = None
    if publish_external:
        publisher = harness.create_publisher(PointCloud2, "/lslidar_point_cloud", qos_profile_sensor_data)
    log_path = Path(f"/tmp/embodied-nexus-lidar-{mode}-{config.stem}.log")
    log_file = log_path.open("w")
    command = ["ros2", "launch", "embodied_nexus_lidar", "lidar.launch.py", f"config_file:={config}"]
    if mode != "both":
        command.append(f"interface_mode:={mode}")
    if port is not None:
        command.append(f"web_port:={port}")
    env = os.environ.copy()
    env["EMBODIED_NEXUS_LIDAR_TOKEN"] = TOKEN
    process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT,
                               env=env, start_new_session=True)
    summary = {"config": config.name, "mode": mode, "pid": process.pid, "log": str(log_path)}
    try:
        cloud = wait_for_cloud(
            process, harness, received,
            publish=(lambda: publisher.publish(external_cloud(harness))) if publisher else None,
        )
        summary["output_width"] = int(cloud.width)
        if publish_external:
            assert cloud.width == 1 and cloud.point_step == 12
        else:
            assert cloud.width == 360 and cloud.point_step == 32

        if verify_web:
            base_url = f"http://127.0.0.1:{port}"
            status_url = f"{base_url}/api/v1/lidars/lidar_main/status"
            wait_for_web(process, status_url, TOKEN)
            try:
                urllib.request.urlopen(status_url, timeout=2.0)
                raise AssertionError("unauthenticated HTTP request unexpectedly succeeded")
            except urllib.error.HTTPError as exc:
                assert exc.code == 401
            client = LidarClient(base_url, TOKEN)
            status = client.status()
            assert status["backend"] == ("ros2_topics" if publish_external else "mock")
            stream = client.subscribe(streams=("points",), points_fps=10, point_stride=4)
            try:
                frame = next(stream)
            finally:
                stream.close()
            assert frame.metadata["channel"] == "points"
            assert len(frame.data) == frame.metadata["point_count"] * frame.metadata["point_step"]
            assert len(list(frame.points(limit=1))) == 1
            summary["web_point_count"] = frame.metadata["point_count"]
            if not publish_external:
                assert frame.metadata["point_count"] == 90
                client.set_capture(False)
                client.set_capture(True)
                candidate = client.validate_config({"scan": {"range_max_m": 17.5}})
                assert candidate["scan"]["range_max_m"] == 17.5
                with client.acquire_control("humble-smoke") as lease:
                    assert lease.start_motor()["hardware_state_confirmed"]
                    assert lease.set_speed(10)["hardware_state_confirmed"]
                assert client.status()["motor_state"] == "stopped"
        return summary
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        log_file.close()
        harness.destroy_subscription(subscription)
        if publisher is not None:
            harness.destroy_publisher(publisher)
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        if "backend=mock" in log_text:
            assert "shutdown lidar stop command sent=True" in log_text
        summary["exit_code"] = process.returncode


def main() -> int:
    rclpy.init()
    harness = rclpy.create_node("embodied_nexus_lidar_validation")
    try:
        results = []
        mock_config = PACKAGE / "config" / "c16_mock.yaml"
        results.append(run_launch(harness, config=mock_config, mode="ros2", port=None, verify_web=False))
        results.append(run_launch(harness, config=mock_config, mode="web", port=free_port(), verify_web=True))
        # Do not pass interface_mode here: this verifies the YAML default (`both`).
        results.append(run_launch(harness, config=mock_config, mode="both", port=free_port(), verify_web=True))
        topics_config = PACKAGE / "config" / "c16_topics.yaml"
        results.append(run_launch(harness, config=topics_config, mode="ros2", port=None,
                                  verify_web=False, publish_external=True))
        print(json.dumps({"ok": True, "results": results}, indent=2))
        return 0
    finally:
        harness.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
