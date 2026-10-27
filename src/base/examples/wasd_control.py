#!/usr/bin/env python3
"""Independent WASD teleoperation window; all device access uses the installed SDK."""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

from embodied_nexus_client import BaseClient


KEYS = {"w", "a", "s", "d"}
SEND_PERIOD_SEC = 0.05


def velocity_for_keys(keys: set[str], linear_speed: float, angular_speed: float) -> tuple[float, float]:
    """Map held keys to Scout differential-drive velocity; opposing keys cancel.

    Args:
        keys: 当前处于按下状态的键集合，用于合成底盘运动指令。 类型为 set[str]。
        linear_speed: 底盘前进或后退时使用的线速度上限。 类型为 float。
        angular_speed: 底盘转向时使用的角速度上限。 类型为 float。

    Returns:
        本次处理产生的结果值。 返回类型为 tuple[float, float]。
    """
    forward = int("w" in keys) - int("s" in keys)
    left = int("a" in keys) - int("d" in keys)
    return forward * linear_speed, left * angular_speed


class WasdWindow:
    def __init__(self, root: tk.Tk) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            root: root，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 tk.Tk。

        Returns:
            无返回值。
        """
        self.root = root
        self.root.title("EmbodiedNexus · Scout 底盘 WASD 控制")
        self.root.resizable(False, False)
        self.commands: queue.Queue[tuple[str, tuple]] = queue.Queue(maxsize=16)
        self.ui_events: queue.Queue[tuple[tk.Variable, str]] = queue.Queue()
        self.lock = threading.RLock()
        self.keys: set[str] = set()
        self.client: BaseClient | None = None
        self.lease = None
        self.motion_enabled = False
        self.connected = False
        self.closing = False
        self.worker_done = threading.Event()
        self.stop_requested = threading.Event()
        self.speed_linear = tk.DoubleVar(value=0.16)
        self.speed_angular = tk.DoubleVar(value=0.60)
        self._linear_speed = 0.16
        self._angular_speed = 0.60
        self.url = tk.StringVar(value="http://127.0.0.1:8778")
        self.device_id = tk.StringVar(value="base_main")
        self.token = tk.StringVar(value="")
        self.status_text = tk.StringVar(value="未连接；连接不会使能或移动底盘")
        self.state_text = tk.StringVar(value="连接状态：未连接")
        self.backend_text = tk.StringVar(value="后端：未知")
        self.velocity_text = tk.StringVar(value="反馈速度：未知")
        self._build()
        self.root.after(50, self._drain_ui_events)
        self.root.bind_all("<KeyPress>", self._key_press, add=True)
        self.root.bind_all("<KeyRelease>", self._key_release, add=True)
        self.root.bind("<FocusOut>", self._focus_lost, add=True)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.worker = threading.Thread(target=self._worker_loop, name="base-wasd-network", daemon=True)
        self.worker.start()

    def _build(self) -> None:
        """构建当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(sticky="nsew")
        ttk.Label(frame, text="服务地址").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.url, width=36).grid(row=0, column=1, columnspan=3, sticky="ew")
        ttk.Label(frame, text="设备 ID").grid(row=1, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.device_id, width=16).grid(row=1, column=1, sticky="ew")
        ttk.Label(frame, text="Bearer Token").grid(row=1, column=2, sticky="w", padx=(10, 0))
        ttk.Entry(frame, textvariable=self.token, show="•", width=20).grid(row=1, column=3, sticky="ew")
        ttk.Button(frame, text="连接", command=self._connect).grid(row=2, column=0, pady=8, sticky="ew")
        ttk.Button(frame, text="申请控制权", command=lambda: self._enqueue("acquire")).grid(row=2, column=1, pady=8, sticky="ew")
        ttk.Button(frame, text="使能运动", command=self._enable).grid(row=2, column=2, pady=8, sticky="ew")
        ttk.Button(frame, text="停止并解除使能  Space", command=self._stop).grid(row=2, column=3, pady=8, sticky="ew")
        ttk.Label(frame, text="线速度 (m/s)").grid(row=3, column=0, sticky="w")
        ttk.Scale(frame, from_=0.03, to=0.50, variable=self.speed_linear,
                  command=self._set_linear_speed).grid(row=3, column=1, sticky="ew")
        ttk.Label(frame, text="角速度 (rad/s)").grid(row=3, column=2, sticky="w", padx=(10, 0))
        ttk.Scale(frame, from_=0.10, to=1.00, variable=self.speed_angular,
                  command=self._set_angular_speed).grid(row=3, column=3, sticky="ew")
        ttk.Label(frame, text="按住 W/S 前进/后退，A/D 左转/右转；组合键可同时平移与旋转。",
                  wraplength=520).grid(row=4, column=0, columnspan=4, sticky="w", pady=(8, 2))
        ttk.Label(frame, textvariable=self.state_text).grid(row=5, column=0, columnspan=4, sticky="w")
        ttk.Label(frame, textvariable=self.backend_text).grid(row=6, column=0, columnspan=4, sticky="w")
        ttk.Label(frame, textvariable=self.velocity_text).grid(row=7, column=0, columnspan=4, sticky="w")
        ttk.Label(frame, textvariable=self.status_text, wraplength=520).grid(
            row=8, column=0, columnspan=4, sticky="w", pady=(4, 0))
        for column in range(4):
            frame.columnconfigure(column, weight=1)

    def _connect(self) -> None:
        """连接当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._enqueue("connect", self.url.get().strip(), self.device_id.get().strip(), self.token.get())

    def _enable(self) -> None:
        """更新设备运动使能状态。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._clear_keys()
        self._enqueue("enable")

    def _stop(self) -> None:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._clear_keys()
        self.stop_requested.set()

    def _enqueue(self, name: str, *args) -> None:
        """将设备命令放入后台发送队列。

        Args:
            name: 名称，作为将设备命令放入后台发送队列所需的输入。 类型为 str。
            *args: 额外的位置参数集合。

        Returns:
            无返回值。
        """
        if self.closing:
            return
        try:
            self.commands.put_nowait((name, args))
        except queue.Full:
            self._ui_status("操作队列已满，请稍后重试")

    def _key_press(self, event) -> None:
        """记录按键按下并更新运动状态。

        Args:
            event: 浏览器或 ROS 派发的事件对象。

        Returns:
            无返回值。
        """
        if isinstance(event.widget, (tk.Entry, ttk.Entry)):
            return
        key = event.keysym.lower()
        if key == "space":
            self._stop()
            return "break"
        if key in KEYS:
            with self.lock:
                if self.motion_enabled:
                    self.keys.add(key)
            self._show_keys()

    def _key_release(self, event) -> None:
        """记录按键释放并更新运动状态。

        Args:
            event: 浏览器或 ROS 派发的事件对象。

        Returns:
            无返回值。
        """
        key = event.keysym.lower()
        if key in KEYS:
            with self.lock:
                self.keys.discard(key)
            self._show_keys()

    def _focus_lost(self, _event) -> None:
        """窗口失焦时清除按键状态并停止运动。

        Args:
            _event: 事件，作为窗口失焦时清除按键状态并停止运动所需的输入。

        Returns:
            无返回值。
        """
        if self.root.focus_displayof() is None:
            self._clear_keys()
            self.stop_requested.set()

    def _clear_keys(self) -> None:
        """清除所有按键状态。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self.lock:
            self.keys.clear()

    def _show_keys(self) -> None:
        """更新界面中的按键状态显示。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self.lock:
            keys = set(self.keys)
            linear_speed, angular_speed = self._linear_speed, self._angular_speed
        linear, angular = velocity_for_keys(keys, linear_speed, angular_speed)
        self._ui_status(f"按键: {' '.join(sorted(keys)) or '无'}；目标速度 {linear:.2f} m/s, {angular:.2f} rad/s")

    def _set_linear_speed(self, value: str) -> None:
        """设置底盘线速度上限。

        Args:
            value: 要应用、比较或转换的输入值。 类型为 str。

        Returns:
            无返回值。
        """
        with self.lock:
            self._linear_speed = float(value)
        self._show_keys()

    def _set_angular_speed(self, value: str) -> None:
        """设置底盘角速度上限。

        Args:
            value: 要应用、比较或转换的输入值。 类型为 str。

        Returns:
            无返回值。
        """
        with self.lock:
            self._angular_speed = float(value)
        self._show_keys()

    def _ui_status(self, value: str) -> None:
        """界面状态。

        Args:
            value: 要应用、比较或转换的输入值。 类型为 str。

        Returns:
            无返回值。
        """
        if not self.closing:
            self.ui_events.put((self.status_text, value))

    def _drain_ui_events(self) -> None:
        """处理后台线程排入的界面事件。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        while True:
            try:
                variable, value = self.ui_events.get_nowait()
            except queue.Empty:
                break
            variable.set(value)
        if not self.closing or not self.worker_done.is_set():
            self.root.after(50, self._drain_ui_events)

    def _worker_loop(self) -> None:
        """在后台线程中持续处理设备命令。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        last_velocity = 0.0
        last_state = 0.0
        try:
            while True:
                if self.stop_requested.is_set():
                    self.stop_requested.clear()
                    self._clear_keys()
                    self._emergency_stop("停止请求")
                if self.closing:
                    self._emergency_stop("窗口关闭")
                    if self.lease is not None:
                        try:
                            self.lease.release()
                        except Exception:
                            pass
                        self.lease = None
                    return
                try:
                    command, args = self.commands.get(timeout=0.01)
                except queue.Empty:
                    command, args = "", ()
                if command:
                    self._execute(command, args)
                now = time.monotonic()
                if self.lease is not None and self.motion_enabled:
                    if self.lease.renew_error:
                        self._fault(f"控制租约续约失败：{self.lease.renew_error}")
                        continue
                    if now - last_velocity >= SEND_PERIOD_SEC:
                        with self.lock:
                            keys = set(self.keys)
                        with self.lock:
                            linear_speed, angular_speed = self._linear_speed, self._angular_speed
                        linear, angular = velocity_for_keys(keys, linear_speed, angular_speed)
                        try:
                            self.lease.set_velocity(linear, angular)
                            last_velocity = now
                        except Exception as exc:
                            self._fault(f"速度请求失败：{exc}")
                            continue
                    if now - last_state >= 0.5:
                        try:
                            state = self.client.status()
                            self._show_state(state)
                            last_state = now
                            if not state.get("connected") or not state.get("motion_enabled"):
                                self._fault("服务报告底盘断连或运动已解除使能")
                        except Exception as exc:
                            self._fault(f"状态读取失败：{exc}")
        except Exception as exc:
            self._ui_status(f"控制线程错误：{exc}")
        finally:
            self.worker_done.set()

    def _execute(self, command: str, args: tuple) -> None:
        """执行当前设备或操作。

        Args:
            command: command，作为执行当前设备或操作所需的输入。 类型为 str。
            args: 额外的位置参数集合。 类型为 tuple。

        Returns:
            无返回值。
        """
        try:
            if command == "connect":
                url, device_id, token = args
                self._prepare_reconnect()
                client = BaseClient(url, token, device_id, timeout=1.0)
                info = client.info()
                state = client.status()
                self.client = client
                self.connected = True
                self.ui_events.put((self.state_text, "连接状态：已连接"))
                self.ui_events.put((self.backend_text, f"后端：{info.get('backend', '未知')} · {info.get('model', 'base')}"))
                self._show_state(state)
                self._ui_status("已连接；底盘保持未使能")
            elif command == "acquire":
                if self.client is None:
                    raise RuntimeError("请先连接底盘服务")
                self.lease = self.client.acquire_control("wasd-window")
                self._ui_status("已申请控制权；请显式使能后移动")
            elif command == "enable":
                if self.lease is None:
                    raise RuntimeError("请先连接并申请控制权")
                result = self.lease.enable_motion(True)
                self.motion_enabled = bool(result.get("enabled"))
                self._clear_keys()
                self._ui_status("底盘已使能；按住 WASD 开始控制" if self.motion_enabled else "底盘未使能")
            elif command == "release":
                if self.lease is not None:
                    self.lease.release()
                self.lease = None
                self.motion_enabled = False
                self._ui_status("控制权已释放")
        except Exception as exc:
            self._fault(f"{command or '操作'}失败：{exc}")

    def _prepare_reconnect(self) -> None:
        """Stop and release the previous session before replacing its client.

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._clear_keys()
        self.motion_enabled = False
        self.connected = False
        if self.client is not None:
            self._emergency_stop("重新连接")
        if self.lease is not None:
            try:
                self.lease.release()
            except Exception as exc:
                self._ui_status(f"旧控制租约释放失败：{exc}")
            finally:
                self.lease = None
        self.client = None

    def _show_state(self, state: dict) -> None:
        """显示状态。

        Args:
            state: 状态，作为显示状态所需的输入。 类型为 dict。

        Returns:
            无返回值。
        """
        connected = bool(state.get("connected"))
        motion_enabled = bool(state.get("motion_enabled"))
        self.ui_events.put((self.state_text,
                            f"连接状态：{'已连接' if connected else '断开'} · 运动：{'已使能' if motion_enabled else '未使能'}"))
        linear = state.get("measured_linear_mps")
        angular = state.get("measured_angular_rps")
        text = "反馈速度：未知" if linear is None or angular is None else (
            f"反馈速度：{linear:.3f} m/s, {angular:.3f} rad/s")
        self.ui_events.put((self.velocity_text, text))

    def _emergency_stop(self, reason: str) -> None:
        """发送紧急停止指令。

        Args:
            reason: 原因，作为发送紧急停止指令所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        self.motion_enabled = False
        self._clear_keys()
        client = self.client
        if client is not None:
            try:
                result = client.stop(reason)
                if not result.get("zero_velocity_written"):
                    self._ui_status(f"{reason}；停止请求已接收，但未确认写出零速度：{result.get('error', '')}")
            except Exception as exc:
                self._ui_status(f"{reason}；停止请求未确认：{exc}")
        self.ui_events.put((self.state_text, "连接状态：停止请求已发送 · 运动未使能"))

    def _fault(self, message: str) -> None:
        """记录故障并更新界面状态。

        Args:
            message: 待处理的消息对象。 类型为 str。

        Returns:
            无返回值。
        """
        self._emergency_stop(message)
        self._ui_status(message)

    def _close(self) -> None:
        """关闭当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.closing = True
        self._clear_keys()
        self._poll_close()

    def _poll_close(self) -> None:
        """检查关闭请求并停止后台轮询。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if self.worker_done.wait(0):
            self.root.destroy()
        else:
            self.root.after(50, self._poll_close)


def main() -> None:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        无返回值。
    """
    root = tk.Tk()
    WasdWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
