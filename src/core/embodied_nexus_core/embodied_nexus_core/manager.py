from __future__ import annotations

from hashlib import sha256
import json
import os
import re
import secrets
import signal
import socket
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .config import DEVICE_TYPES, load_platform


class OperationInProgress(RuntimeError):
    """A conflicting operation is already managing this device instance."""


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _unused_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _group_alive(process: Any) -> bool:
    if process.poll() is None:
        return True
    if os.name == "nt":
        return False
    try:
        os.killpg(process.pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


class DeviceManager:
    """Owns only the device launch processes started by this core instance."""

    def __init__(self, config_file: str | None = None, *, popen=subprocess.Popen,
                 check_packages: bool = True):
        self.platform = load_platform(config_file, check_packages=check_packages)
        self._popen = popen
        self._items = {item["id"]: {
            **item, "process": None, "process_state": "stopped", "gateway_reachable": False,
            "device_status": None, "last_seen_at": None, "last_probe_monotonic": 0.0,
            "last_error": "", "exit_code": None, "port": 0, "token": "",
        } for item in self.platform["devices"]}
        self._locks = {key: threading.RLock() for key in self._items}
        self._operation_locks = {key: threading.Lock() for key in self._items}
        self._port_lock = threading.Lock()
        self._refresh_pool = ThreadPoolExecutor(max_workers=max(1, len(self._items)),
                                                thread_name_prefix="core-device-probe")
        self._stop_monitor = threading.Event()
        self._monitor: threading.Thread | None = None

    def start_monitor(self) -> None:
        if self._monitor is None:
            self._monitor = threading.Thread(target=self._monitor_loop, name="core-device-monitor", daemon=True)
            self._monitor.start()

    def close_monitor(self) -> None:
        self._stop_monitor.set()
        if self._monitor and self._monitor.is_alive():
            self._monitor.join(timeout=2.0)
        self._refresh_pool.shutdown(wait=False, cancel_futures=True)

    def _monitor_loop(self) -> None:
        interval = float(self.platform["core"]["poll_interval_sec"])
        while not self._stop_monitor.is_set():
            futures = [self._refresh_pool.submit(self._probe, key) for key in self._items]
            for future in as_completed(futures):
                try:
                    future.result()
                except Exception:
                    pass
            self._stop_monitor.wait(interval)

    @staticmethod
    def _http(item: dict[str, Any], suffix: str, *, method: str = "GET", body: dict | None = None,
              timeout: float = 0.5) -> tuple[int, bytes, str]:
        data = json.dumps(body or {}, separators=(",", ":")).encode() if method != "GET" else None
        request = Request(f"http://127.0.0.1:{item['port']}{suffix}", data=data, method=method,
                          headers={"Authorization": f"Bearer {item['token']}",
                                   "Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=timeout) as response:
                return response.status, response.read(), response.headers.get("Content-Type", "application/json")
        except HTTPError as exc:
            return exc.code, exc.read(), exc.headers.get("Content-Type", "application/json")

    def _probe(self, device_id: str) -> None:
        item = self._items[device_id]
        with self._locks[device_id]:
            process = item["process"]
            if process is None:
                return
            exit_code = process.poll()
            process_error = ""
            if exit_code is not None:
                process_error = item["last_error"] or f"device launch exited with code {exit_code}"
                item.update(process_state="failed", gateway_reachable=False, exit_code=exit_code,
                            last_error=process_error)
                if not _group_alive(process):
                    return
            try:
                code, payload, _ = self._http(item, f"/api/v1/{DEVICE_TYPES[item['type']][3]}/{device_id}/status")
                if code >= 400:
                    raise RuntimeError(f"device gateway returned HTTP {code}: {payload[:256].decode(errors='replace')}")
                state = json.loads(payload.decode("utf-8"))
                item.update(gateway_reachable=True, device_status=state, last_seen_at=_now(),
                            last_probe_monotonic=time.monotonic(), last_error=process_error)
                if item["process_state"] == "starting":
                    item["process_state"] = "running"
            except Exception as exc:
                if time.monotonic() - item["last_probe_monotonic"] >= float(self.platform["core"]["gateway_unavailable_sec"]):
                    item["gateway_reachable"] = False
                    item["last_error"] = process_error or f"device gateway unavailable: {exc}"

    def _start_one(self, device_id: str) -> dict[str, Any]:
        item = self._require(device_id)
        operation_lock = self._operation_locks[device_id]
        if not operation_lock.acquire(blocking=False):
            raise OperationInProgress(f"another operation is in progress for device {device_id}")
        try:
            return self._start_one_impl(item)
        finally:
            operation_lock.release()

    def _start_one_impl(self, item: dict[str, Any]) -> dict[str, Any]:
        device_id = item["id"]
        with self._locks[device_id]:
            if item["process"] is not None:
                if item["process"].poll() is not None and not _group_alive(item["process"]):
                    item.update(process=None, process_state="failed", gateway_reachable=False,
                                device_status=None, token="", port=0)
                else:
                    was_alive = item["process"].poll() is None
                    self._probe(device_id)
                    success = was_alive and item["gateway_reachable"] and item["process_state"] == "running"
                    return {"success": success, "device": self._snapshot(item), "already_running": True,
                            "error": "managed process group already exists; inspect its state before starting again" if not success else ""}
            if not item["enabled"]:
                raise RuntimeError(f"device {device_id} is disabled in the platform configuration")
            package, launch_file = DEVICE_TYPES[item["type"]][:2]
            selected_port = 0
            for _ in range(20):
                candidate = _unused_port()
                with self._port_lock:
                    used = {other["port"] for other in self._items.values()
                            if other["id"] != device_id and other["port"]}
                    if candidate not in used:
                        selected_port = candidate
                        break
            if not selected_port:
                raise RuntimeError("could not allocate a unique loopback port for device gateway")
            item["port"] = selected_port
            item["token"] = secrets.token_urlsafe(32)
            env = dict(os.environ)
            env.pop(str(self.platform["core"]["auth_token_env"]), None)
            env["EMBODIED_NEXUS_DEVICE_TOKEN"] = item["token"]
            node_slug = re.sub(r"[^A-Za-z0-9_]", "_", device_id)[:48]
            node_suffix = f"{node_slug}_{sha256(device_id.encode('utf-8')).hexdigest()[:10]}"
            arguments = [
                "ros2", "launch", package, launch_file,
                f"config_file:={item['config']}", "interface_mode:=both",
                "web_bind_host:=127.0.0.1", f"web_port:={item['port']}",
                f"use_sim_time:={'true' if item['use_sim_time'] else 'false'}",
                "web_auth_token_env:=EMBODIED_NEXUS_DEVICE_TOKEN",
                f"node_name:=en_{item['type']}_{node_suffix}",
            ]
            if item["type"] != "camera":
                arguments.append(f"node_name_web:=en_{item['type']}_{node_suffix}_web")
            try:
                options = {"env": env, "stdout": None, "stderr": None}
                if os.name == "nt":
                    options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
                else:
                    options["start_new_session"] = True
                item["process"] = self._popen(arguments, **options)
            except Exception as exc:
                item.update(process_state="failed", last_error=str(exc), gateway_reachable=False,
                            port=0, token="")
                raise
            item.update(process_state="starting", gateway_reachable=False, device_status=None,
                        last_seen_at=None, last_probe_monotonic=0.0, last_error="", exit_code=None)
            deadline = time.monotonic() + float(self.platform["core"]["start_timeout_sec"])
            while time.monotonic() < deadline:
                if item["process"].poll() is not None:
                    item.update(process_state="failed", exit_code=item["process"].returncode,
                                last_error=f"device launch exited with code {item['process'].returncode}")
                    break
                self._probe(device_id)
                if item["gateway_reachable"]:
                    return {"success": True, "device": self._snapshot(item)}
                time.sleep(0.15)
            item["process_state"] = "failed"
            item["last_error"] = item["last_error"] or "timed out waiting for the device gateway; process retained"
            raise RuntimeError(item["last_error"])

    def start(self, device_id: str) -> dict[str, Any]:
        return self._start_one(device_id)

    def start_all(self) -> list[dict[str, Any]]:
        ids = [key for key, item in self._items.items() if item["enabled"]]
        return self._run_many(ids, self._start_one)

    def _stop_confirmed(self, item: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        response: dict[str, Any] = {}
        deadline = time.monotonic() + float(self.platform["core"]["shutdown_timeout_sec"])
        try:
            code, payload, _ = self._http(
                item, f"/api/v1/{DEVICE_TYPES[item['type']][3]}/{item['id']}/stop",
                method="POST", body={"reason": "EmbodiedNexus core device shutdown"},
                timeout=max(0.01, deadline - time.monotonic()),
            )
            if code >= 400:
                return False, {"error": f"stop request returned HTTP {code}: {payload[:256].decode(errors='replace')}"}
            response = json.loads(payload.decode("utf-8"))
        except Exception as exc:
            return False, {"error": f"stop request failed: {exc}"}
        if time.monotonic() > deadline:
            return False, response
        confirmed_key = "hardware_stop_confirmed" if item["type"] == "gripper" else "stationary_confirmed"
        sent_key = "stop_sent" if item["type"] == "gripper" else "controller_accepted" if item["type"] == "arm" else "zero_velocity_written"
        if response.get(confirmed_key) is True:
            return True, response
        if response.get(sent_key) is not True:
            return False, response
        feedback_error = ""
        while time.monotonic() < deadline:
            try:
                code, payload, _ = self._http(
                    item, f"/api/v1/{DEVICE_TYPES[item['type']][3]}/{item['id']}/status",
                    timeout=min(0.5, max(0.01, deadline - time.monotonic())),
                )
                if code >= 400:
                    feedback_error = f"status request returned HTTP {code}"
                    break
                state = json.loads(payload.decode("utf-8"))
                state_confirmed_key = "stop_feedback_confirmed" if item["type"] == "gripper" else "stationary_confirmed"
                if (time.monotonic() <= deadline and
                        (state.get(state_confirmed_key) is True or
                         (item["type"] == "gripper" and state.get("hardware_stop_confirmed") is True))):
                    return True, {**response, "feedback": state}
            except Exception as exc:
                feedback_error = f"status feedback unavailable: {exc}"
                break
            time.sleep(0.1)
        if feedback_error:
            response["feedback_error"] = feedback_error
        return False, response

    @staticmethod
    def _signal_process(process: Any, sig: int) -> None:
        try:
            if os.name == "nt":
                if process.poll() is not None:
                    return
                process.send_signal(getattr(signal, "CTRL_BREAK_EVENT", sig))
            else:
                os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def _close_process(self, item: dict[str, Any]) -> None:
        process = item["process"]
        if process is None:
            item.update(process=None, process_state="stopped", gateway_reachable=False,
                        device_status=None, token="", port=0)
            return
        item["process_state"] = "closing"
        self._signal_process(process, signal.SIGINT)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        deadline = time.monotonic() + 3.0
        while _group_alive(process) and time.monotonic() < deadline:
            time.sleep(0.1)
        if _group_alive(process):
            self._signal_process(process, signal.SIGTERM)
            deadline = time.monotonic() + 2.0
            while _group_alive(process) and time.monotonic() < deadline:
                time.sleep(0.1)
        if _group_alive(process):
            self._signal_process(process, signal.SIGKILL)
            deadline = time.monotonic() + 1.0
            while _group_alive(process) and time.monotonic() < deadline:
                time.sleep(0.05)
        item.update(process=None, process_state="stopped", gateway_reachable=False,
                    device_status=None, token="", port=0, last_error="")

    def shutdown(self, device_id: str) -> dict[str, Any]:
        item = self._require(device_id)
        operation_lock = self._operation_locks[device_id]
        if not operation_lock.acquire(blocking=False):
            raise OperationInProgress(f"another operation is in progress for device {device_id}")
        try:
            return self._shutdown_impl(item)
        finally:
            operation_lock.release()

    def _shutdown_impl(self, item: dict[str, Any]) -> dict[str, Any]:
        device_id = item["id"]
        with self._locks[device_id]:
            process = item["process"]
            if process is None:
                self._close_process(item)
                return {"success": True, "device": self._snapshot(item), "already_stopped": True}
            if process.poll() is not None and not _group_alive(process):
                item.update(process_state="failed", gateway_reachable=False,
                            exit_code=process.returncode,
                            last_error="device launch exited before stop could be confirmed")
                return {"success": False, "device": self._snapshot(item),
                        "error": item["last_error"]}
            if item["type"] in {"base", "arm", "gripper"}:
                confirmed, result = self._stop_confirmed(item)
                if not confirmed:
                    item.update(process_state="stop_blocked", last_error="device stop was not confirmed; process retained")
                    return {"success": False, "device": self._snapshot(item), "stop": result,
                            "error": item["last_error"]}
            self._close_process(item)
            return {"success": True, "device": self._snapshot(item), "stop": result if item["type"] in {"base", "arm", "gripper"} else None}

    def restart(self, device_id: str) -> dict[str, Any]:
        item = self._require(device_id)
        operation_lock = self._operation_locks[device_id]
        if not operation_lock.acquire(blocking=False):
            raise OperationInProgress(f"another operation is in progress for device {device_id}")
        try:
            stopped = self._shutdown_impl(item)
            if not stopped["success"]:
                return {"success": False, "shutdown": stopped, "error": stopped["error"]}
            return {"success": True, "shutdown": stopped, "start": self._start_one_impl(item)}
        finally:
            operation_lock.release()

    def shutdown_all(self) -> list[dict[str, Any]]:
        ids = [key for key, item in self._items.items() if item["process"] is not None]
        return self._run_many(ids, self.shutdown)

    def stop_all(self) -> list[dict[str, Any]]:
        ids = [key for key, item in self._items.items()
               if item["type"] in {"base", "arm", "gripper"} and item["process"] is not None]
        def stop_one(device_id: str) -> dict[str, Any]:
            item = self._require(device_id)
            operation_lock = self._operation_locks[device_id]
            if not operation_lock.acquire(blocking=False):
                return {"success": False, "device_id": device_id, "error": "another operation is in progress"}
            try:
                with self._locks[device_id]:
                    confirmed, result = self._stop_confirmed(item)
                    if not confirmed:
                        item["last_error"] = "stop was not confirmed"
                    return {"success": confirmed, "device_id": device_id, "stop": result,
                            "error": "" if confirmed else item["last_error"]}
            finally:
                operation_lock.release()
        return self._run_many(ids, stop_one)

    def _run_many(self, ids: list[str], operation) -> list[dict[str, Any]]:
        results = []
        with ThreadPoolExecutor(max_workers=max(1, len(ids))) as pool:
            futures = {pool.submit(operation, key): key for key in ids}
            for future in as_completed(futures):
                key = futures[future]
                try:
                    results.append({"device_id": key, **future.result()})
                except Exception as exc:
                    results.append({"device_id": key, "success": False, "error": str(exc)})
        order = {key: index for index, key in enumerate(ids)}
        return sorted(results, key=lambda result: order[result["device_id"]])

    def refresh(self) -> None:
        futures = [self._refresh_pool.submit(self._probe, key) for key in self._items]
        for future in futures:
            future.result()

    def get_device(self, device_id: str) -> dict[str, Any]:
        item = self._require(device_id)
        return self._snapshot(item)

    def list_devices(self) -> list[dict[str, Any]]:
        return [self._snapshot(item) for item in self._items.values()]

    def status(self) -> dict[str, Any]:
        items = self.list_devices()
        return {"state": "running", "config_file": self.platform["config_file"],
                "device_count": len(items), "running_count": sum(i["process_state"] in {"running", "starting", "stop_blocked"} for i in items),
                "devices": items, "timestamp": _now()}

    def request_core_shutdown(self) -> dict[str, Any]:
        results = self.shutdown_all()
        return {"success": all(item.get("success", False) for item in results), "results": results}

    @staticmethod
    def _snapshot(item: dict[str, Any]) -> dict[str, Any]:
        state = item["device_status"] or {}
        return {"id": item["id"], "type": item["type"], "enabled": item["enabled"],
                "process_state": item["process_state"], "pid": item["process"].pid if item["process"] else None,
                "gateway_reachable": item["gateway_reachable"],
                "device_connected": state.get("connected", state.get("online")),
                "motion_enabled": state.get("motion_enabled"), "device_state": state,
                "last_seen_at": item["last_seen_at"], "last_error": item["last_error"],
                "exit_code": item["exit_code"]}

    def _require(self, device_id: str) -> dict[str, Any]:
        try:
            return self._items[device_id]
        except KeyError as exc:
            raise KeyError(f"unknown core device: {device_id}") from exc
