from __future__ import annotations

import argparse
import time

import rclpy
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.srv import ControlWorld
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from embodied_nexus_sim.ros_support import SimTimestampGuard


class ClockRewindLatch:
    """Stay faulted after the first backward jump until this process restarts."""

    def __init__(self) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._guard = SimTimestampGuard()
        self._last_stamp_sec: float | None = None
        self.faulted = False
        self.just_faulted = False

    def observe(self, stamp) -> bool:
        """记录时钟样本并检测时钟跳变。

        Args:
            stamp: 时间戳，作为执行 observe 对应的处理流程所需的输入。

        Returns:
            布尔值，表示执行 observe 对应的处理流程是否成功或成立。
        """
        current = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        rewound = self._guard.observe(stamp)
        advanced = (self._last_stamp_sec is None or
                    current > self._last_stamp_sec + 1e-9)
        self._last_stamp_sec = current
        self.just_faulted = rewound and not self.faulted
        if rewound:
            self.faulted = True
        # Fortress may keep publishing /clock at a fixed timestamp while
        # paused. Reassert the pause only after a new tick if someone manually
        # unpauses the world after a rewind.
        return rewound or (self.faulted and advanced)


class SimClockGuard(Node):
    """Pause Fortress after a backward clock jump so stale commands cannot run."""

    def __init__(self, world_name: str) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            world_name: world名称，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        super().__init__("sim_clock_guard", parameter_overrides=[
            Parameter("use_sim_time", value=False),
        ])
        self.world_name = world_name.strip()
        if not self.world_name:
            raise ValueError("world_name must not be empty")
        self._clock_guard = ClockRewindLatch()
        self._pause_request_pending = False
        self._last_pause_request = 0.0
        self._control = self.create_client(
            ControlWorld, f"/world/{self.world_name}/control")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._clock_subscription = self.create_subscription(
            Clock, "/clock", self._on_clock, qos)

    def _on_clock(self, message: Clock) -> None:
        """处理clock事件。

        Args:
            message: 待处理的消息对象。 类型为 Clock。

        Returns:
            无返回值。
        """
        if not self._clock_guard.observe(message.clock):
            return
        now = time.monotonic()
        if self._pause_request_pending or now - self._last_pause_request < 0.5:
            return
        self._pause_request_pending = True
        self._last_pause_request = now
        request = ControlWorld.Request()
        request.world_control.pause = True
        if self._clock_guard.just_faulted:
            self.get_logger().error(
                "simulator clock moved backwards; latching the Gazebo world paused until restart")
        future = self._control.call_async(request)
        future.add_done_callback(self._pause_finished)

    def _pause_finished(self, future) -> None:
        """处理 Gazebo 暂停服务的异步完成结果。

        Args:
            future: 异步服务调用返回的 Future 对象。

        Returns:
            无返回值。
        """
        self._pause_request_pending = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f"Gazebo rejected the safety pause for world {self.world_name}")
        except Exception as exc:
            self.get_logger().error(f"could not pause Gazebo after a clock rewind: {exc}")


def main(argv=None) -> int:
    """解析启动参数并运行程序入口。

    Args:
        argv: argv，作为解析启动参数并运行程序入口所需的输入。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-name", default="embodied_nexus_workcell")
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = SimClockGuard(args.world_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
