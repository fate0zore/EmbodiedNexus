from __future__ import annotations

import json
import math
import os
import shlex
import subprocess
import time
from typing import Any

from embodied_nexus_client import CoreClient


WORLD = "embodied_nexus_workcell"
POSE_TOPIC = f"/world/{WORLD}/pose/info"
CLOCK_TOPIC = "/clock"
CONTROL_SERVICE = f"/world/{WORLD}/control"


def run_wsl(*command: str) -> str:
    """运行wsl。

    Args:
        *command: command，作为运行wsl所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    distro = os.environ.get("EMBODIED_NEXUS_WSL_DISTRO", "Ubuntu-22.04")
    ign_ip = os.environ.get("EMBODIED_NEXUS_IGN_IP", "192.168.31.185")
    inner = (
        "source /opt/ros/humble/setup.bash && "
        f"export ROS_LOCALHOST_ONLY=1 IGN_IP={shlex.quote(ign_ip)} && "
        + shlex.join(command)
    )
    result = subprocess.run(
        ["wsl", "-d", distro, "--", "bash", "-lc", inner],
        check=False, capture_output=True, text=True, timeout=20.0,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or
                           f"WSL command failed with exit code {result.returncode}")
    return result.stdout.strip()


def world_clock_sec() -> float:
    """读取 Gazebo 世界时钟并转换为秒。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 float。
    """
    output = run_wsl("ign", "topic", "-e", "-t", CLOCK_TOPIC, "-n", "1", "--json-output")
    for line in reversed(output.splitlines()):
        try:
            clock = json.loads(line).get("sim", {})
        except json.JSONDecodeError:
            continue
        return float(clock.get("sec", 0)) + float(clock.get("nsec", 0)) * 1e-9
    raise RuntimeError(f"Gazebo did not return a simulation clock sample: {output}")


def scene_pose() -> dict[str, float]:
    """查询并返回场景中机器人的位姿。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, float]。
    """
    output = run_wsl("ign", "topic", "-e", "-t", POSE_TOPIC, "-n", "1", "--json-output")
    for line in reversed(output.splitlines()):
        try:
            poses = json.loads(line).get("pose", [])
        except json.JSONDecodeError:
            continue
        robot = next((pose for pose in poses if pose.get("name") == "arachne"), None)
        if robot is None:
            continue
        position = robot["position"]
        orientation = robot["orientation"]
        yaw = math.atan2(
            2.0 * (orientation.get("w", 1.0) * orientation.get("z", 0.0) +
                   orientation.get("x", 0.0) * orientation.get("y", 0.0)),
            1.0 - 2.0 * (orientation.get("y", 0.0) ** 2 + orientation.get("z", 0.0) ** 2),
        )
        return {"x_m": float(position["x"]), "y_m": float(position["y"]), "yaw_rad": yaw}
    raise RuntimeError(f"Gazebo scene did not return the Arachne pose: {output}")


def set_world_time_only_reset() -> str:
    """设置world时间only重置。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    request: dict[str, Any] = {"world_control": {"reset": {"time_only": True}}}
    result = run_wsl(
        "ros2", "service", "call", CONTROL_SERVICE,
        "ros_gz_interfaces/srv/ControlWorld", json.dumps(request),
    )
    if "success=True" not in result:
        raise RuntimeError(f"Gazebo rejected the simulation-time reset: {result}")
    return result


def set_world_paused(paused: bool) -> str:
    """设置worldpaused。

    Args:
        paused: paused，作为设置worldpaused所需的输入。 类型为 bool。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    request = {"world_control": {"pause": bool(paused)}}
    result = run_wsl(
        "ros2", "service", "call", CONTROL_SERVICE,
        "ros_gz_interfaces/srv/ControlWorld", json.dumps(request),
    )
    if "success=True" not in result:
        raise RuntimeError(f"Gazebo rejected the world pause request: {result}")
    return result


def validate(core: CoreClient) -> dict[str, Any]:
    """验证当前设备或操作。

    Args:
        core: core，作为验证当前设备或操作所需的输入。 类型为 CoreClient。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    started = core.start("base_main")
    if not started.get("success"):
        raise RuntimeError(f"could not start base_main: {started}")
    base = core.base("base_main")
    lease = base.acquire_control("sim-clock-rewind-acceptance", ttl_sec=10.0)
    try:
        lease.enable_motion()
        operation = lease.turn_relative(1.2, 0.1)
        time.sleep(0.3)
        before = {"clock_sec": world_clock_sec(), "scene_pose": scene_pose()}

        reset_result = set_world_time_only_reset()
        terminal = operation.wait(timeout=8.0)
        time.sleep(0.3)
        paused = {"clock_sec": world_clock_sec(), "scene_pose": scene_pose(), "state": base.state()}
        time.sleep(0.5)
        paused_again = {"clock_sec": world_clock_sec(), "scene_pose": scene_pose()}

        if terminal.get("state") != "failed" or not terminal.get("stop_requested"):
            raise RuntimeError(f"active base motion did not fail on time rewind: {terminal}")
        if not terminal.get("zero_velocity_written"):
            raise RuntimeError(f"base did not request zero velocity after time rewind: {terminal}")
        if abs(paused_again["clock_sec"] - paused["clock_sec"]) > 0.05:
            raise RuntimeError("Gazebo clock guard did not keep the world paused")

        manual_resume = set_world_paused(False)
        time.sleep(0.2)
        resumed_once = {"clock_sec": world_clock_sec(), "scene_pose": scene_pose()}
        time.sleep(0.8)
        guard_repaused = {"clock_sec": world_clock_sec(), "scene_pose": scene_pose()}
        delta_m = math.hypot(
            guard_repaused["scene_pose"]["x_m"] - resumed_once["scene_pose"]["x_m"],
            guard_repaused["scene_pose"]["y_m"] - resumed_once["scene_pose"]["y_m"],
        )
        delta_yaw = math.atan2(
            math.sin(guard_repaused["scene_pose"]["yaw_rad"] - resumed_once["scene_pose"]["yaw_rad"]),
            math.cos(guard_repaused["scene_pose"]["yaw_rad"] - resumed_once["scene_pose"]["yaw_rad"]),
        )
        if abs(guard_repaused["clock_sec"] - resumed_once["clock_sec"]) > 0.05:
            raise RuntimeError("clock-rewind guard did not re-pause a manual unpause")
        if delta_m > 0.05 or abs(delta_yaw) > math.radians(2.0):
            raise RuntimeError(
                f"robot moved after manual resume from a rewind: {delta_m:.4f} m, "
                f"{math.degrees(delta_yaw):.3f} deg"
            )
        return {
            "before_rewind": before,
            "reset_service": reset_result,
            "operation_after_rewind": terminal,
            "paused_after_rewind": paused,
            "manual_resume_service": manual_resume,
            "guard_repaused": guard_repaused,
            "post_resume_scene_delta_m": delta_m,
            "post_resume_scene_delta_yaw_rad": delta_yaw,
            "cleanup": "restart sim.launch.py to clear the latched time-rewind fault",
        }
    finally:
        lease._stop_event.set()
        lease._thread.join(timeout=1.0)
        # The guard intentionally leaves Gazebo paused with stale feedback.
        # Core may retain the device process until the enclosing launch restarts.
        try:
            core.shutdown("base_main")
        except Exception:
            pass


def main() -> int:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    core = CoreClient(
        os.environ.get("EMBODIED_NEXUS_CORE_URL", "http://127.0.0.1:8780"),
        os.environ["EMBODIED_NEXUS_CORE_TOKEN"], timeout=8.0, management_timeout=60.0,
    )
    print(json.dumps(validate(core), ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
