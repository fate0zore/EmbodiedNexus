"""Short C16 hardware acceptance; explicit flag required before any motor command.

Run on the Humble device after sourcing the workspace:
  python3 src/lidar/embodied_nexus_lidar/test/validate_c16_hardware.py --enable-motor-test

The script records the DIFOP speed before any start command. If stopped-state
DIFOP omits the speed, a previously observed value may be supplied explicitly;
the script re-applies and confirms that baseline before allowing motor start.
Without a reliable restore value or fresh, recognized C16 DIFOP, it does not
start or change speed and requests only a stop where possible.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from lslidar_msgs.msg import LslidarPacket

from embodied_nexus_lidar_interfaces.srv import (
    AcquireLidarControl, GetLidarState, ReleaseLidarControl, RenewLidarControl,
    SetLidarMotor, SetLidarMotorSpeed,
)
from embodied_nexus_lidar.protocol import inspect_difop


PACKAGE = Path(__file__).resolve().parents[1]
PREFIX = "/embodied_nexus/lidars/lidar_main"


class HardwareCheck:
    def __init__(self, node: Node):
        """初始化对象并保存构造参数或依赖。

        Args:
            node: 提供 ROS 参数、服务或话题接口的节点实例。 类型为 Node。

        Returns:
            无返回值。
        """
        self.node = node
        self.clients = {
            "state": node.create_client(GetLidarState, f"{PREFIX}/get_state"),
            "acquire": node.create_client(AcquireLidarControl, f"{PREFIX}/control/acquire"),
            "renew": node.create_client(RenewLidarControl, f"{PREFIX}/control/renew"),
            "release": node.create_client(ReleaseLidarControl, f"{PREFIX}/control/release"),
            "motor": node.create_client(SetLidarMotor, f"{PREFIX}/motor"),
            "speed": node.create_client(SetLidarMotorSpeed, f"{PREFIX}/motor/speed"),
        }
        self.cloud_samples: list[tuple[float, int]] = []
        self.difop_count = 0
        self.last_difop = None
        self.difop_subscription = node.create_subscription(
            LslidarPacket, f"{PREFIX}/_driver/difop_packet", self._on_difop, 10
        )
        self.subscription = node.create_subscription(
            PointCloud2, f"{PREFIX}/points", self._on_cloud, qos_profile_sensor_data
        )
        self.lease_id = ""
        self.last_renew = 0.0

    def _on_cloud(self, message: PointCloud2) -> None:
        """处理点云事件。

        Args:
            message: 待处理的消息对象。 类型为 PointCloud2。

        Returns:
            无返回值。
        """
        self.cloud_samples.append((time.monotonic(), len(message.data)))

    def _on_difop(self, message: LslidarPacket) -> None:
        """接收并记录雷达设备配置数据包。

        Args:
            message: 待处理的消息对象。 类型为 LslidarPacket。

        Returns:
            无返回值。
        """
        self.difop_count += 1
        packet = bytes(message.data)
        try:
            decoded = inspect_difop(packet)
            self.last_difop = {"length": len(packet), "header_hex": packet[:4].hex(),
                               "protocol_version": decoded.model_version,
                               "speed_hz": decoded.speed_hz}
        except ValueError as exc:
            self.last_difop = {"length": len(packet), "header_hex": packet[:4].hex(),
                               "decode_error": str(exc)}

    def call(self, key, request, timeout=5.0):
        """调用当前设备或操作。

        Args:
            key: 按键，作为调用当前设备或操作所需的输入。
            request: 封装调用输入字段的请求对象。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。

        Returns:
            本次处理产生的结果值。
        """
        client = self.clients[key]
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            raise TimeoutError(f"lidar {key} service is unavailable")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError(f"lidar {key} service timed out")
        response = future.result()
        if response is None:
            raise RuntimeError(f"lidar {key} service returned no response")
        return response

    def state(self) -> dict:
        """获取设备当前状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        response = self.call("state", GetLidarState.Request())
        if not response.success:
            raise RuntimeError(response.error)
        return json.loads(response.state_json)

    def motor(self, enabled: bool, *, lease=False) -> dict:
        """启停雷达电机并返回服务结果。

        Args:
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。
            lease: 设备控制租约对象。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = SetLidarMotor.Request()
        request.lease_id = self.lease_id if lease else ""
        request.enabled = enabled
        response = self.call("motor", request)
        result = {"success": response.success, "command_sent": response.command_sent,
                  "hardware_state_confirmed": response.hardware_state_confirmed,
                  "motor_state": response.motor_state, "error": response.error}
        return result

    def renew_if_due(self) -> None:
        """控制租约到期前续租。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if not self.lease_id or time.monotonic() - self.last_renew < 0.5:
            return
        request = RenewLidarControl.Request()
        request.lease_id = self.lease_id
        response = self.call("renew", request)
        if not response.success:
            raise RuntimeError(f"control lease renewal failed: {response.error}")
        self.last_renew = time.monotonic()

    def acquire(self) -> None:
        """申请设备控制权。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        request = AcquireLidarControl.Request()
        request.owner = "c16-short-hardware-acceptance"
        request.ttl_sec = 2.0
        response = self.call("acquire", request)
        if not response.success:
            raise RuntimeError(f"control lease acquisition failed: {response.error}")
        self.lease_id = response.lease_id
        self.last_renew = time.monotonic()

    def set_speed(self, speed_hz: int) -> dict:
        """设置速度。

        Args:
            speed_hz: 目标雷达转速，单位为 Hz。 类型为 int。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = SetLidarMotorSpeed.Request()
        request.lease_id = self.lease_id
        request.speed_hz = speed_hz
        response = self.call("speed", request)
        return {"success": response.success, "command_sent": response.command_sent,
                "hardware_state_confirmed": response.hardware_state_confirmed,
                "requested_speed_hz": response.requested_speed_hz,
                "actual_speed_known": response.actual_speed_known,
                "actual_speed_hz": response.actual_speed_hz if response.actual_speed_known else None,
                "error": response.error}

    def wait_for_speed(self, speed_hz: int, timeout=4.0) -> dict:
        """等待雷达达到目标转速或超时。

        Args:
            speed_hz: 目标雷达转速，单位为 Hz。 类型为 int。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        deadline = time.monotonic() + timeout
        last = {}
        while time.monotonic() < deadline:
            self.renew_if_due()
            last = self.state()
            if last["motor_speed_known"] and last["motor_speed_hz"] == speed_hz:
                return last
            rclpy.spin_once(self.node, timeout_sec=0.1)
        raise TimeoutError(f"DIFOP readback did not confirm {speed_hz} Hz; last state={last}")

    def sample(self, seconds: float) -> dict:
        """采集指定时长内的雷达状态样本。

        Args:
            seconds: 采集或等待的时长，单位为秒。 类型为 float。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        start_time = time.monotonic()
        start_count = len(self.cloud_samples)
        deadline = start_time + seconds
        while time.monotonic() < deadline:
            self.renew_if_due()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        duration = max(time.monotonic() - start_time, 1e-6)
        samples = self.cloud_samples[start_count:]
        byte_count = sum(size for _, size in samples)
        measured = 0.0
        if len(samples) > 1 and samples[-1][0] > samples[0][0]:
            measured = (len(samples) - 1) / (samples[-1][0] - samples[0][0])
        state = self.state()
        return {
            "duration_sec": round(duration, 2), "frames_received": len(samples),
            "measured_pointcloud_hz": round(measured, 2),
            "pointcloud_bandwidth_bytes_per_sec": round(byte_count / duration, 1),
            "state_pointcloud_fps": state["pointcloud_fps"],
            "pointcloud_age_sec": state["pointcloud_age_sec"],
            "dropped_frames": state["dropped_frames"],
        }

    def release(self) -> dict | None:
        """释放当前设备控制租约。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict | None。
        """
        if not self.lease_id:
            return None
        request = ReleaseLidarControl.Request()
        request.lease_id = self.lease_id
        response = self.call("release", request)
        self.lease_id = ""
        return {"success": response.success, "stop_sent": response.stop_sent,
                "error": response.error}


def main() -> int:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--enable-motor-test", action="store_true",
                        help="enable the explicitly authorized short motor test")
    parser.add_argument("--inspect-only", action="store_true",
                        help="read startup state and DIFOP without issuing motor service calls")
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--recorded-original-speed-hz", type=int, choices=(5, 10, 20),
                        help="previously observed live speed; use only when stopped-state DIFOP omits it")
    args = parser.parse_args()
    if not args.enable_motor_test and not args.inspect_only:
        parser.error("refusing to launch C16 hardware without --enable-motor-test")

    log_path = Path("/tmp/embodied-nexus-c16-hardware.log")
    log_file = log_path.open("w")
    command = ["ros2", "launch", "embodied_nexus_lidar", "lidar.launch.py",
               f"config_file:={PACKAGE / 'config' / 'c16.yaml'}", "interface_mode:=ros2"]
    process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT,
                               env=os.environ.copy(), start_new_session=True)
    result = {"launch_pid": process.pid, "log": str(log_path), "commands": [], "speed_tests": []}
    rclpy.init()
    node = rclpy.create_node("embodied_nexus_c16_hardware_acceptance")
    check = HardwareCheck(node)
    original_speed = None
    changed_speed = False
    exit_code = 0
    last_state_error = ""
    try:
        deadline = time.monotonic() + args.startup_timeout
        state = {}
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("C16 lidar launch exited during startup")
            try:
                state = check.state()
                if state.get("connected") and state.get("difop_age_sec") is not None:
                    break
            except Exception as exc:
                last_state_error = f"{type(exc).__name__}: {exc}"
            rclpy.spin_once(node, timeout_sec=0.1)
        result["initial_state"] = state
        result["state_read_error"] = last_state_error
        result["difop_messages_received"] = check.difop_count
        result["last_difop"] = check.last_difop
        original_speed = state.get("motor_speed_hz") if state.get("motor_speed_known") else None
        speed_source = "startup_state" if original_speed in (5, 10, 20) else ""
        if original_speed not in (5, 10, 20):
            original_speed = args.recorded_original_speed_hz
            if original_speed is not None:
                speed_source = "previous_live_readback"
        result["original_speed_hz"] = original_speed
        result["original_speed_source"] = speed_source or "unavailable"
        result["initial_speed_known"] = original_speed is not None

        if args.inspect_only:
            result["motor_test"] = "inspect-only: no test motor commands; normal service shutdown still attempts stop"
        elif not state.get("connected"):
            result["motor_test"] = "skipped: no fresh, model-validated C16 DIFOP"
        elif original_speed is None:
            stopped = check.motor(False)
            result["final_stop"] = stopped
            result["motor_test"] = "speed and start tests skipped: cannot safely record original speed"
            if not stopped["success"] or not stopped["command_sent"]:
                result["error"] = f"stop command failed: {stopped}"
                exit_code = 1
        else:
            # Store the readable speed before the first command. On protocol v3,
            # motor-start itself selects 10 Hz; restoration below is required.
            stopped = check.motor(False)
            result["commands"].append({"action": "stop_without_lease", **stopped})
            if not stopped["success"] or not stopped["command_sent"]:
                raise RuntimeError(f"initial motor stop failed: {stopped}")

            check.acquire()
            # DIFOP may report a zero/unknown speed while the motor is stopped.
            # If a live pre-test readback (or explicit prior readback) recorded
            # the baseline, restore and confirm it before sending motor-start.
            before_start = check.state()
            if (not before_start.get("motor_speed_known") or
                    before_start.get("motor_speed_hz") != original_speed):
                baseline = check.set_speed(original_speed)
                result["baseline_speed_restore"] = baseline
                if not baseline["success"] or not baseline["command_sent"]:
                    raise RuntimeError(f"restoring recorded baseline {original_speed} Hz failed: {baseline}")
                changed_speed = True
                baseline_state = check.wait_for_speed(original_speed)
                result["baseline_speed_readback_hz"] = baseline_state["motor_speed_hz"]
            started = check.motor(True, lease=True)
            result["commands"].append({"action": "start_with_lease", **started})
            if not started["success"] or not started["command_sent"]:
                raise RuntimeError(f"motor start command failed: {started}")
            result["startup_sample"] = check.sample(1.5)

            for speed_hz in (5, 10, 20):
                check.renew_if_due()
                command_result = check.set_speed(speed_hz)
                result["speed_tests"].append({"requested_hz": speed_hz, "command": command_result})
                if not command_result["success"] or not command_result["command_sent"]:
                    raise RuntimeError(f"setting {speed_hz} Hz failed: {command_result}")
                changed_speed = True
                state = check.wait_for_speed(speed_hz)
                result["speed_tests"][-1]["difop_readback_hz"] = state["motor_speed_hz"]
                result["speed_tests"][-1]["sample"] = check.sample(1.2)
            result["motor_test"] = "completed"
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        exit_code = 1
    finally:
        # Any speed path that may have changed the motor is restored before the
        # lease release requests stop. Restoration remains best-effort if DIFOP went stale.
        if original_speed is not None and (changed_speed or check.lease_id):
            try:
                if not check.lease_id:
                    check.acquire()
                restored = check.set_speed(original_speed)
                result["restore_speed"] = restored
                if restored["success"] and restored["command_sent"]:
                    state = check.wait_for_speed(original_speed, timeout=4.0)
                    result["restore_readback_hz"] = state["motor_speed_hz"]
                else:
                    exit_code = 1
            except Exception as exc:
                result["restore_error"] = f"{type(exc).__name__}: {exc}"
        try:
            if check.lease_id:
                release = check.release()
                result["release"] = release
                if not release["success"] or not release["stop_sent"]:
                    exit_code = 1
                    result["final_stop"] = check.motor(False)
            elif (not args.inspect_only and result.get("initial_state", {}).get("connected")
                  and "final_stop" not in result):
                result["final_stop"] = check.motor(False)
                if not result["final_stop"]["success"] or not result["final_stop"]["command_sent"]:
                    exit_code = 1
        except Exception as exc:
            result["release_error"] = f"{type(exc).__name__}: {exc}"
            exit_code = 1
            if check.lease_id:
                check.lease_id = ""
                try:
                    result["final_stop"] = check.motor(False)
                except Exception as stop_exc:
                    result["final_stop_error"] = f"{type(stop_exc).__name__}: {stop_exc}"
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        log_file.close()
        result["launch_exit_code"] = process.returncode
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        result["shutdown_stop_logged"] = "shutdown lidar stop command sent=True" in log_text
        if "restore_error" in result:
            exit_code = 1
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(result, indent=2, ensure_ascii=False))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
