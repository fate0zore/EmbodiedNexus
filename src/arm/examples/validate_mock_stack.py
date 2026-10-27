#!/usr/bin/env python3
"""Bounded HTTP/SDK integration check using only the default mock backend."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time

from embodied_nexus_client import ArmClient


def main() -> int:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="optional path to the Aubo mock YAML")
    parser.add_argument("--url", default="http://127.0.0.1:8779")
    parser.add_argument("--device-id", default="arm_main")
    args = parser.parse_args()
    token = "arm-mock-validation"
    env = os.environ.copy()
    env["EMBODIED_NEXUS_ARM_TOKEN"] = token
    launch = ["ros2", "launch", "embodied_nexus_arm", "arm.launch.py", "interface_mode:=both"]
    if args.config:
        launch.append(f"config_file:={args.config}")
    process = subprocess.Popen(launch, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, start_new_session=True)
    client = ArmClient(args.url, token, args.device_id, timeout=5.0)
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("arm launch exited early:\n" + (process.stdout.read() if process.stdout else ""))
            try:
                state = client.status()
                break
            except Exception:
                time.sleep(0.25)
        else:
            output = ""
            if process.poll() is not None and process.stdout:
                output = process.stdout.read()
            raise TimeoutError("arm HTTP gateway did not become ready within 30 seconds:\n" + output)
        if state.get("backend") != "mock" or not state.get("feedback_valid"):
            raise AssertionError(f"unexpected initial mock state: {state}")
        status_stream = client.subscribe_status()
        try:
            websocket_state = next(status_stream)
        finally:
            status_stream.close()
        if (websocket_state.get("type") != "status" or
                websocket_state.get("device_id") != args.device_id or
                websocket_state.get("backend") != "mock"):
            raise AssertionError(f"unexpected WebSocket status frame: {websocket_state}")
        operation_failure = None
        with client.acquire_control("arm-mock-validation") as lease:
            lease.enable_motion()
            operation = lease.move_joints([0.04, 0.0, 0.0, 0.0, 0.0, 0.0],
                                          speed_rad_sec=0.2, timeout_sec=8.0)
            result = operation.wait(timeout=12.0)
            if result.get("state") != "succeeded" or not result.get("controller_accepted"):
                operation_failure = AssertionError(f"mock joint operation failed: {result}")
            else:
                stop = client.stop("mock integration check complete")
        if operation_failure:
            raise operation_failure
        print(json.dumps({"initial_state": state, "websocket_state": websocket_state,
                          "operation": result, "stop": stop},
                         ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)


if __name__ == "__main__":
    raise SystemExit(main())
