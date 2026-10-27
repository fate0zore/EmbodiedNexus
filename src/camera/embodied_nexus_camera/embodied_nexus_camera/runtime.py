from __future__ import annotations

import json
import re
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import cv2

from .calibration import RgbdCalibration, apply_rgb_candidate, capture_intrinsic_sample, inspect_apriltag_sample, load_calibration, solve_rgb_intrinsics
from .config import apply_runtime_patch, public_config
from .web_server import CameraHub


class CameraRuntime:
    def __init__(self, config: dict[str, Any], log) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。
            log: log，作为初始化对象并保存构造参数或依赖所需的输入。

        Returns:
            无返回值。
        """
        self.config = config
        self._log = log
        self.hub = CameraHub(config["camera_id"], float(config["streams"]["frame_stale_sec"]))
        calibration_cfg = config["calibration"]
        capture = config["capture"]
        self.calibration = load_calibration(
            factory_file=calibration_cfg.get("factory_file", ""),
            custom_file=calibration_cfg.get("custom_file", ""),
            color_size=(int(capture["color_width"]), int(capture["color_height"])),
            depth_size=(int(capture["depth_width"]), int(capture["depth_height"])),
            depth_scale_m=float(calibration_cfg["depth_scale_m"]),
            extrinsic_unit=str(calibration_cfg["depth_to_color_translation_unit"]),
            allow_resolution_scaling=bool(calibration_cfg.get("allow_resolution_scaling", False)),
        )
        if self.calibration.issue:
            self._log("warning", self.calibration.issue)
        self._lock = threading.RLock()
        self._color = None
        self._sessions: dict[str, Path] = {}
        self._results: dict[str, dict[str, Any]] = {}
        self._solving: set[str] = set()
        self._solve_events: dict[str, threading.Event] = {}
        self._solve_status: dict[str, dict[str, Any]] = {}
        self._last_result_id = ""
        session_root = Path(config["calibration_session"]["root"]).expanduser()
        if session_root.is_dir():
            for session_dir in session_root.iterdir():
                if session_dir.is_dir() and (session_dir / "session.json").is_file():
                    self._sessions[session_dir.name] = session_dir

    def log(self, level: str, message: str) -> None:
        """构造并输出夹爪运行日志。

        Args:
            level: level，作为执行 log 对应的处理流程所需的输入。 类型为 str。
            message: 待处理的消息对象。 类型为 str。

        Returns:
            无返回值。
        """
        self._log(level, message)

    def update_color(self, image) -> None:
        """更新color。

        Args:
            image: 图像，作为更新color所需的输入。

        Returns:
            无返回值。
        """
        with self._lock:
            self._color = image

    def public_config(self) -> dict[str, Any]:
        """公开配置。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return public_config(self.config)

    def apply_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        """应用配置。

        Args:
            patch: patch，作为应用配置所需的输入。 类型为 dict[str, Any]。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self._lock:
            self.config = apply_runtime_patch(self.config, patch)
            self.config["config_version"] = int(self.config.get("config_version", 1)) + 1
            self.hub.stale_sec = float(self.config["streams"]["frame_stale_sec"])
            return self.public_config()

    def validate_config_patch(self, patch: dict[str, Any]) -> dict[str, Any]:
        """验证配置patch。

        Args:
            patch: patch，作为验证配置patch所需的输入。 类型为 dict[str, Any]。

        Returns:
            布尔值，表示验证配置patch是否成功或成立。 返回类型为 dict[str, Any]。
        """
        with self._lock:
            return public_config(apply_runtime_patch(self.config, patch))

    def calibration_summary(self) -> dict[str, Any]:
        """生成相机标定结果的摘要信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        cal = self.calibration
        return {
            "version": cal.version,
            "source": cal.source,
            "depth_scale_m": cal.depth_scale_m,
            "depth_to_color_translation_m": cal.depth_to_color_translation_m.tolist(),
            "depth_to_color_rotation": cal.depth_to_color_rotation.tolist(),
            "color": self._model_summary(cal.color),
            "depth": self._model_summary(cal.depth),
            "usable_for_color_cloud": cal.usable_for_color_cloud,
            "issue": cal.issue,
            "sessions": {key: str(path) for key, path in self._sessions.items()},
            "solving": sorted(self._solving),
            "solve_status": {key: dict(value) for key, value in self._solve_status.items()},
            "history": self.calibration_history(),
        }

    @staticmethod
    def _model_summary(model) -> dict[str, Any] | None:
        """生成相机模型参数的摘要信息。

        Args:
            model: model，作为执行 _model_summary 对应的处理流程所需的输入。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any] | None。
        """
        if model is None:
            return None
        return {
            "image_size": [model.width, model.height],
            "intrinsic": {"fx": model.fx, "fy": model.fy, "cx": model.cx, "cy": model.cy},
            "distortion": model.distortion,
            "distortion_model": model.distortion_model,
            "calibrated": model.calibrated,
        }

    def start_calibration(self, request: dict[str, Any]) -> dict[str, Any]:
        """启动calibration。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 dict[str, Any]。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        profile = self.config["calibration_session"]
        board_path = str(request.get("board_image_path") or profile["board_image_path"]).strip()
        if not board_path or not Path(board_path).expanduser().is_file():
            raise ValueError("calibration board_image_path must refer to an existing board reference image")
        session_id = str(uuid.uuid4())
        session_dir = Path(profile["root"]).expanduser() / session_id
        session_dir.mkdir(parents=True, exist_ok=False)
        metadata = {
            "session_id": session_id,
            "board_image_path": board_path,
            "board_width_m": float(request.get("board_width_m", profile["board_width_m"])),
            "board_height_m": float(request.get("board_height_m", profile["board_height_m"])),
            "image_size": [int(self.config["capture"]["color_width"]), int(self.config["capture"]["color_height"])],
            "config_version": self.config.get("config_version", 1),
            "calibration_version": self.calibration.version,
        }
        (session_dir / "session.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        with self._lock:
            self._sessions[session_id] = session_dir
        return metadata

    def capture_calibration(self, session_id: str) -> dict[str, Any]:
        """采集calibration。

        Args:
            session_id: 会话标识符，作为采集calibration所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self._lock:
            session_dir = self._sessions.get(session_id)
            color = self._color.copy() if self._color is not None else None
        if session_dir is None:
            raise ValueError("unknown calibration session")
        if color is None:
            raise RuntimeError("no current color frame is available")
        metadata = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
        detection = inspect_apriltag_sample(color, metadata["board_image_path"])
        return capture_intrinsic_sample(
            color,
            session_dir,
            {"board": {key: metadata[key] for key in ("board_image_path", "board_width_m", "board_height_m")},
             "stamp_sec": time.time(), "config_version": self.config.get("config_version", 1),
             "calibration_version": self.calibration.version, "detection": detection},
        )

    def delete_calibration_sample(self, session_id: str, filename: str) -> dict[str, Any]:
        """删除calibration采集样本。

        Args:
            session_id: 会话标识符，作为删除calibration采集样本所需的输入。 类型为 str。
            filename: filename，作为删除calibration采集样本所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        if not re.fullmatch(r"frame_\d{4,}\.png", filename):
            raise ValueError("sample filename must be a camera calibration PNG basename")
        with self._lock:
            session_dir = self._sessions.get(session_id)
        if session_dir is None:
            raise ValueError("unknown calibration session")
        manifest_path = session_dir / "capture_manifest.json"
        if not manifest_path.is_file():
            raise ValueError("calibration session contains no samples")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        kept = [row for row in manifest if row.get("file") != filename]
        if len(kept) == len(manifest):
            raise ValueError("calibration sample not found")
        sample_path = session_dir / filename
        if sample_path.exists():
            sample_path.unlink()
        manifest_path.write_text(json.dumps(kept, indent=2), encoding="utf-8")
        return {"deleted": filename, "remaining_samples": len(kept)}

    def solve_calibration(self, session_id: str) -> dict[str, Any]:
        """启动相机标定求解并返回任务结果。

        Args:
            session_id: 会话标识符，作为执行 solve_calibration 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self._lock:
            session_dir = self._sessions.get(session_id)
            if session_dir is None:
                raise ValueError("unknown calibration session")
            if session_id in self._solving:
                raise RuntimeError("calibration session is already solving")
            self._solving.add(session_id)
            self._solve_events[session_id] = threading.Event()
            manifest_path = session_dir / "capture_manifest.json"
            total = len(json.loads(manifest_path.read_text(encoding="utf-8"))) if manifest_path.exists() else 0
            self._solve_status[session_id] = {"state": "solving", "processed_frames": 0, "total_frames": total, "progress": 0.0}
        thread = threading.Thread(target=self._solve_worker, args=(session_id, session_dir), daemon=True)
        thread.start()
        return {"session_id": session_id, "state": "solving"}

    def _solve_worker(self, session_id: str, session_dir: Path) -> None:
        """在后台线程中执行相机标定求解。

        Args:
            session_id: 会话标识符，作为执行 _solve_worker 对应的处理流程所需的输入。 类型为 str。
            session_dir: 会话dir，作为执行 _solve_worker 对应的处理流程所需的输入。 类型为 Path。

        Returns:
            无返回值。
        """
        try:
            metadata = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
            profile = self.config["calibration_session"]
            candidate = solve_rgb_intrinsics(
                session_dir,
                board_image_path=metadata["board_image_path"],
                board_width_m=float(metadata["board_width_m"]),
                board_height_m=float(metadata["board_height_m"]),
                min_frames=int(profile["min_usable_frames"]),
                max_rms_px=float(profile["max_rms_px"]),
                max_outlier_px=float(profile["max_outlier_px"]),
                progress_callback=lambda done, total: self._update_solve_progress(session_id, done, total),
                cancel_event=self._solve_events[session_id],
            )
            with self._lock:
                self._results[candidate["calibration_id"]] = candidate
                self._last_result_id = candidate["calibration_id"]
                self._solve_status[session_id] = {"state": "succeeded", "processed_frames": len(candidate["per_frame"]), "total_frames": len(candidate["per_frame"]), "progress": 1.0, "result_id": candidate["calibration_id"]}
            self.log("info", f"RGB calibration finished: id={candidate['calibration_id']} accepted={candidate['accepted']} RMS={candidate['rms_px']:.4f}px")
        except InterruptedError as exc:
            with self._lock:
                status = self._solve_status.get(session_id, {})
                self._solve_status[session_id] = {**status, "state": "cancelled", "error": str(exc)}
        except Exception as exc:
            with self._lock:
                status = self._solve_status.get(session_id, {})
                self._solve_status[session_id] = {**status, "state": "failed", "error": str(exc)}
            self.log("error", f"RGB calibration failed for session {session_id}: {exc}")
        finally:
            with self._lock:
                self._solving.discard(session_id)

    def _update_solve_progress(self, session_id: str, done: int, total: int) -> None:
        """更新solveprogress。

        Args:
            session_id: 会话标识符，作为更新solveprogress所需的输入。 类型为 str。
            done: 操作完成后调用的回调函数。 类型为 int。
            total: total，作为更新solveprogress所需的输入。 类型为 int。

        Returns:
            无返回值。
        """
        with self._lock:
            status = self._solve_status.setdefault(session_id, {})
            status.update({"state": "solving", "processed_frames": int(done), "total_frames": int(total), "progress": float(done / total) if total else 0.0})

    def calibration_solve_status(self, session_id: str) -> dict[str, Any]:
        """查询标定求解状态。

        Args:
            session_id: 会话标识符，作为执行 calibration_solve_status 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self._lock:
            return dict(self._solve_status.get(session_id, {"state": "unknown", "processed_frames": 0, "total_frames": 0, "progress": 0.0}))

    def cancel_calibration_solve(self, session_id: str) -> bool:
        """取消calibrationsolve。

        Args:
            session_id: 会话标识符，作为取消calibrationsolve所需的输入。 类型为 str。

        Returns:
            布尔值，表示取消calibrationsolve是否成功或成立。
        """
        with self._lock:
            event = self._solve_events.get(session_id)
            if event is None or session_id not in self._solving:
                return False
            event.set()
            return True

    def result_for_session(self, session_id: str) -> dict[str, Any] | None:
        """结果的会话。

        Args:
            session_id: 会话标识符，作为结果的会话所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any] | None。
        """
        with self._lock:
            session_dir = self._sessions.get(session_id)
        if session_dir is None:
            return None
        candidates = sorted(session_dir.glob("candidate_*.json"), key=lambda path: path.stat().st_mtime)
        return json.loads(candidates[-1].read_text(encoding="utf-8")) if candidates else None

    def _active_profile_path(self) -> Path:
        """返回当前选中仿真性能配置文件的路径。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 Path。
        """
        config = self.config["calibration"]
        return Path(config.get("custom_file") or (Path(config["history_file"]).parent / "active_calibration.json")).expanduser()

    def calibration_history(self) -> list[dict[str, Any]]:
        """列出已保存的相机标定历史记录。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 list[dict[str, Any]]。
        """
        path = self._active_profile_path()
        if not path.is_file():
            return []
        try:
            profile = json.loads(path.read_text(encoding="utf-8"))
            return list(profile.get("history", []))
        except (OSError, json.JSONDecodeError, TypeError):
            return []

    def get_calibration_result(self, result_id: str) -> dict[str, Any] | None:
        """获取calibration结果。

        Args:
            result_id: 结果标识符，作为获取calibration结果所需的输入。 类型为 str。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 dict[str, Any] | None。
        """
        with self._lock:
            result = self._results.get(result_id)
        if result is not None:
            return result
        for candidate in self.calibration_history():
            if candidate.get("calibration_id") == result_id:
                return candidate
        for path in Path(self.config["calibration_session"]["root"]).expanduser().glob(f"*/candidate_{result_id}.json"):
            return json.loads(path.read_text(encoding="utf-8"))
        return None

    def apply_calibration(self, result_id: str) -> dict[str, Any]:
        """应用calibration。

        Args:
            result_id: 结果标识符，作为应用calibration所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        candidate = self.get_calibration_result(result_id)
        if candidate is None:
            raise ValueError("unknown calibration result")
        config = self.config["calibration"]
        profile_path = self._active_profile_path()
        # Preserve the imported depth stream and RGB-D extrinsics when applying a new RGB model.
        base_path = config.get("custom_file") if config.get("custom_file") and Path(config["custom_file"]).is_file() else config.get("factory_file")
        if not profile_path.exists() and base_path and Path(base_path).is_file():
            profile_path.parent.mkdir(parents=True, exist_ok=True)
            profile_path.write_text(Path(base_path).read_text(encoding="utf-8"), encoding="utf-8")
        apply_rgb_candidate(candidate, profile_path)
        capture = self.config["capture"]
        updated = load_calibration(
            custom_file=str(profile_path),
            color_size=(int(capture["color_width"]), int(capture["color_height"])),
            depth_size=(int(capture["depth_width"]), int(capture["depth_height"])),
            depth_scale_m=float(config["depth_scale_m"]),
            extrinsic_unit=str(config["depth_to_color_translation_unit"]),
            allow_resolution_scaling=bool(config.get("allow_resolution_scaling", False)),
        )
        with self._lock:
            self.calibration = updated
            self._results[result_id] = candidate
            self.hub._frames.pop("points", None)
        return {"applied": True, "calibration_version": updated.version, "active_file": str(profile_path)}
