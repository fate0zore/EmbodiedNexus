from __future__ import annotations

import json
import hashlib
import math
import os
import threading
import time

from embodied_nexus_client import CoreClient


def wait_for(predicate, label: str, timeout: float = 20.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = predicate()
        if last:
            return last
        time.sleep(0.2)
    raise TimeoutError(f"timed out waiting for {label}; last state={last}")


def operation_succeeded(result: dict) -> bool:
    return result.get("success") is True or result.get("state") == "succeeded"


def read_camera_color_sample(camera, *, after_sequence: int | None = None,
                             timeout_sec: float = 15.0) -> dict:
    deadline = time.monotonic() + timeout_sec
    for frame in camera.subscribe(streams=("color",), color_fps=10.0):
        if frame.metadata.get("channel") != "color":
            continue
        sequence = int(frame.metadata["sequence"])
        if after_sequence is not None and sequence <= after_sequence:
            if time.monotonic() >= deadline:
                break
            continue
        return {"sequence": sequence, "stamp_sec": float(frame.metadata["stamp_sec"]),
                "frame_id": frame.metadata.get("frame_id"),
                "payload_sha256": hashlib.sha256(frame.data).hexdigest()}
    raise TimeoutError("timed out waiting for a new simulated wrist-camera color frame")


def validate(core: CoreClient) -> int:
    lifecycle = []
    for device_id in ("base_main", "arm_main", "gripper_main", "camera_front", "lidar_main"):
        print(f"[sim validation] start/query/shutdown {device_id}", flush=True)
        started = core.start(device_id)
        if not started.get("success"):
            raise RuntimeError(f"could not start {device_id}: {started}")
        state = core.device(device_id)
        if state.get("process_state") != "running" or not state.get("gateway_reachable"):
            raise RuntimeError(f"{device_id} did not become queryable: {state}")
        config_read = None
        if device_id == "gripper_main":
            config_read = bool(core.gripper(device_id).config())
        stopped = core.shutdown(device_id)
        if not stopped.get("success"):
            raise RuntimeError(f"could not stop {device_id}: {stopped}")
        lifecycle.append({"device_id": device_id, "start": started.get("success"),
                          "query": state.get("gateway_reachable"), "shutdown": stopped.get("success"),
                          "config_read": config_read})

    startup = core.start_all()
    if not startup.get("success"):
        raise RuntimeError(f"could not start all simulation devices: {startup}")
    print("[sim validation] all devices are running; exercising the platform", flush=True)
    base, arm, gripper = core.base("base_main"), core.arm("arm_main"), core.gripper("gripper_main")
    camera, lidar = core.camera("camera_front"), core.lidar("lidar_main")
    wait_for(lambda: base.state().get("motion_feedback_valid"), "base odometry")
    wait_for(lambda: arm.state().get("feedback_valid"), "arm joint state")
    wait_for(lambda: gripper.status().get("feedback_valid"), "gripper joint state")
    wait_for(lambda: camera.status().get("online"), "RGB-D camera")
    wait_for(lambda: lidar.status().get("connected"), "lidar point cloud")

    scan_stop = threading.Event()
    nearest_forward = {"range_m": None}
    scan_error = {"message": ""}
    lidar_frames = {"points": None, "scan": None}

    def read_scans() -> None:
        try:
            for frame in lidar.subscribe(streams=("points", "scan"), points_fps=5.0,
                                         scan_fps=10.0, point_stride=2):
                if scan_stop.is_set():
                    break
                channel = frame.metadata.get("channel")
                if channel == "points":
                    sample = list(frame.points(limit=1))
                    if sample and all(name in sample[0] for name in ("x", "y", "z")):
                        lidar_frames["points"] = {
                            "point_count": frame.metadata.get("point_count"),
                            "frame_id": frame.metadata.get("frame_id"),
                        }
                    continue
                if channel != "scan":
                    continue
                scan = frame.scan()
                lidar_frames["scan"] = {"count": len(scan["ranges"]),
                                        "frame_id": frame.metadata.get("frame_id")}
                angle_min = float(frame.metadata["angle_min_rad"])
                angle_step = float(frame.metadata["angle_increment_rad"])
                ranges = []
                for index, distance in enumerate(scan["ranges"]):
                    angle = angle_min + index * angle_step
                    front_delta = abs(math.atan2(math.sin(angle), math.cos(angle)))
                    if front_delta <= math.radians(50) and math.isfinite(distance) and distance > 0.15:
                        ranges.append(distance)
                if ranges:
                    nearest_forward["range_m"] = min(ranges)
        except Exception as exc:
            if not scan_stop.is_set():
                scan_error["message"] = str(exc)

    scan_thread = threading.Thread(target=read_scans, name="sim-validation-lidar", daemon=True)
    scan_thread.start()
    def forward_scan_ready():
        if scan_error["message"]:
            raise RuntimeError(f"lidar scan stream failed: {scan_error['message']}")
        return nearest_forward["range_m"]

    wait_for(forward_scan_ready, "forward lidar scan")

    with base.acquire_control("sim-validation") as base_lease:
        base_lease.enable_motion()
        base_move = base_lease.move_relative(0.5, 0.12).wait(timeout=15.0)
        base_after_move = base.state()
        print("[sim validation] base move", json.dumps(base_move),
              "pose", base_after_move.get("x_m"), base_after_move.get("y_m"), flush=True)
        base_turn = base_lease.turn_relative(0.5235987756, 0.3).wait(timeout=15.0)
        base_after_turn = base.state()
        print("[sim validation] base turn", json.dumps(base_turn),
              "pose", base_after_turn.get("x_m"), base_after_turn.get("y_m"),
              "yaw", base_after_turn.get("yaw_rad"), flush=True)
        obstacle_approach = base_lease.move_relative(1.4, 0.15)
        obstacle_detected = False
        deadline = time.monotonic() + 12.0
        next_report = time.monotonic()
        while time.monotonic() < deadline:
            operation = obstacle_approach.status()
            if operation.get("state") in {"succeeded", "failed", "cancelled"}:
                break
            if scan_error["message"]:
                obstacle_approach.cancel()
                raise RuntimeError(f"lidar stream failed during obstacle approach: {scan_error['message']}")
            if nearest_forward["range_m"] is not None and nearest_forward["range_m"] < 1.2:
                obstacle_detected = True
                obstacle_approach.cancel()
                break
            if time.monotonic() >= next_report:
                print("[sim validation] obstacle approach",
                      "nearest_forward_m=", nearest_forward["range_m"],
                      "operation=", operation.get("state"), flush=True)
                next_report = time.monotonic() + 1.0
            time.sleep(0.05)
        else:
            obstacle_approach.cancel()
        obstacle_stop = obstacle_approach.wait(timeout=5.0)
        base_lease.enable_motion(False)
        base_after_obstacle = base.state()
    scan_stop.set()
    if not obstacle_detected or obstacle_stop.get("state") != "cancelled":
        raise RuntimeError(
            "forward lidar did not cancel the base before the obstacle: "
            f"nearest={nearest_forward['range_m']}, stop={obstacle_stop}, "
            f"pose=({base_after_obstacle.get('x_m')}, {base_after_obstacle.get('y_m')})"
        )

    wrist_camera_before_arm = read_camera_color_sample(camera)
    with arm.acquire_control("sim-validation") as arm_lease:
        arm_lease.power_on()
        arm_lease.startup()
        arm_lease.enable_motion()
        home = list(arm.state()["positions_rad"])
        approach = list(home)
        approach[1] += 0.10
        arm_move = arm_lease.move_joints(approach, speed_rad_sec=0.15, timeout_sec=12.0).wait(timeout=14.0)
        wrist_camera_after_arm = read_camera_color_sample(
            camera, after_sequence=wrist_camera_before_arm["sequence"])
        if wrist_camera_after_arm["payload_sha256"] == wrist_camera_before_arm["payload_sha256"]:
            raise RuntimeError("wrist-camera RGB image did not change after the arm moved")

        velocity_command = [0.0, 0.05, 0.0, 0.0, 0.0, 0.0]
        velocity_start = arm_lease.set_joint_velocity(velocity_command)
        time.sleep(0.4)
        velocity_stop = arm_lease.set_joint_velocity([0.0] * 6)
        velocity_state = arm.state()

        cancel_target = list(velocity_state["positions_rad"])
        cancel_target[1] += 0.25
        arm_cancel_handle = arm_lease.move_joints(
            cancel_target, speed_rad_sec=0.10, timeout_sec=12.0)
        time.sleep(0.35)
        arm_cancel_request = arm_cancel_handle.cancel()
        arm_cancel = arm_cancel_handle.wait(timeout=6.0)
        arm_cancel_feedback = arm.state()

        with gripper.acquire_control("sim-validation") as gripper_lease:
            gripper_lease.enable_motion()
            gripper_close = gripper_lease.move_close().wait(timeout=8.0)
        arm_return = arm_lease.move_joints(home, speed_rad_sec=0.15, timeout_sec=12.0).wait(timeout=14.0)

    with gripper.acquire_control("sim-validation") as gripper_lease:
        gripper_lease.enable_motion()
        gripper_open = gripper_lease.move_open().wait(timeout=8.0)
        gripper_relative_handle = gripper_lease.move_relative(30.0)
        time.sleep(0.12)
        gripper_cancel_request = gripper_relative_handle.cancel()
        gripper_cancel = gripper_relative_handle.wait(timeout=6.0)

    camera_frames = set()
    camera_frame_samples = {"color": [], "depth": []}
    camera_pair = None
    camera_deadline = time.monotonic() + 15.0
    for frame in camera.subscribe(streams=("color", "depth"), color_fps=10.0, depth_fps=10.0):
        channel = frame.metadata.get("channel")
        camera_frames.add(channel)
        if channel in camera_frame_samples:
            samples = camera_frame_samples[channel]
            samples.append(frame.metadata)
            del samples[:-30]
        if camera_frame_samples["color"] and camera_frame_samples["depth"]:
            candidates = [
                (abs(float(color["stamp_sec"]) - float(depth["stamp_sec"])), color, depth)
                for color in camera_frame_samples["color"]
                for depth in camera_frame_samples["depth"]
            ]
            camera_pair = min(candidates, key=lambda value: value[0])
            if camera_pair[0] <= 0.100001:
                break
        if time.monotonic() >= camera_deadline:
            break
    camera_status, camera_info = camera.status(), camera.info()
    camera_calibration = camera.calibration()
    color_frame = camera_pair[1] if camera_pair else {}
    depth_frame = camera_pair[2] if camera_pair else {}
    pair_delta_sec = camera_pair[0] if camera_pair else float("inf")
    color_model, depth_model = camera_calibration.get("color"), camera_calibration.get("depth")
    if not color_model or not depth_model:
        raise RuntimeError(f"RGB-D camera intrinsics were not received: {camera_calibration}")
    if color_frame.get("width") != depth_frame.get("width") or color_frame.get("height") != depth_frame.get("height"):
        raise RuntimeError("simulated RGB and depth image dimensions do not align")
    if color_model.get("image_size") != depth_model.get("image_size"):
        raise RuntimeError("simulated RGB and depth intrinsic dimensions do not align")
    if float(depth_frame.get("depth_scale_m", 0.0)) <= 0.0:
        raise RuntimeError("simulated depth image did not declare a positive meter scale")
    if pair_delta_sec > 0.1:
        raise RuntimeError(f"simulated RGB and depth timestamps differ by {pair_delta_sec:.3f}s")
    lidar_status, lidar_info = lidar.status(), lidar.info()
    results = {
        "device_lifecycle": lifecycle,
        "base_move_0_5m": base_move, "base_turn_30deg": base_turn,
        "base_after_obstacle": base_after_obstacle,
        "obstacle_detected": obstacle_detected, "obstacle_stop": obstacle_stop,
        "minimum_forward_range_m": nearest_forward["range_m"],
        "arm_approach": arm_move, "arm_velocity_start": velocity_start,
        "wrist_camera_before_arm": wrist_camera_before_arm,
        "wrist_camera_after_arm": wrist_camera_after_arm,
        "arm_velocity_stop": velocity_stop, "arm_velocity_feedback": velocity_state,
        "arm_cancel_request": arm_cancel_request, "arm_cancel": arm_cancel,
        "arm_cancel_feedback": arm_cancel_feedback,
        "gripper_close": gripper_close,
        "arm_return": arm_return, "gripper_open": gripper_open,
        "gripper_cancel_request": gripper_cancel_request, "gripper_cancel": gripper_cancel,
        "camera": {"online": camera_status.get("online"), "frames_received": sorted(camera_frames),
                   "streams": camera_status.get("streams"),
                   "capabilities": camera_info.get("capabilities"),
                   "color_depth_stamp_delta_sec": pair_delta_sec,
                   "depth_scale_m": depth_frame.get("depth_scale_m"),
                   "color_intrinsic": color_model.get("intrinsic"),
                   "depth_intrinsic": depth_model.get("intrinsic")},
        "lidar": {"connected": lidar_status.get("connected"), "capabilities": lidar_info.get("capabilities")},
        "lidar_frames": lidar_frames,
        "arm_capabilities": arm.info().get("capabilities"),
        "gripper_feedback": gripper.info().get("feedback"),
        "base_final_state": base.state(),
    }
    print(json.dumps(results, ensure_ascii=False, indent=2))
    successful = all(operation_succeeded(result) for result in
                     (base_move, base_turn, arm_move, gripper_close, arm_return, gripper_open))
    if not successful:
        raise RuntimeError("simulated actions, RGB-D streaming or lidar obstacle stop failed")
    if not {"color", "depth"}.issubset(camera_frames):
        raise RuntimeError("simulated RGB-D stream did not return both color and depth")
    if lidar_frames["points"] is None or lidar_frames["scan"] is None:
        raise RuntimeError(f"simulated lidar did not return both PointCloud2 and LaserScan: {lidar_frames}")
    if arm_cancel.get("state") != "cancelled":
        raise RuntimeError("arm trajectory cancellation was not confirmed")
    if gripper_cancel.get("state") != "cancelled":
        raise RuntimeError("gripper relative-motion cancellation was not confirmed")
    return 0


def main() -> int:
    core = CoreClient(os.environ.get("EMBODIED_NEXUS_CORE_URL", "http://127.0.0.1:8780"),
                      os.environ["EMBODIED_NEXUS_CORE_TOKEN"], timeout=8.0,
                      management_timeout=90.0)
    try:
        return validate(core)
    finally:
        print("device_shutdown:", json.dumps(core.shutdown_all(), ensure_ascii=False))


if __name__ == "__main__":
    raise SystemExit(main())
