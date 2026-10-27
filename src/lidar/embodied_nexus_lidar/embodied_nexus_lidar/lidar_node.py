from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from embodied_nexus_lidar_interfaces.msg import LidarState
from embodied_nexus_lidar_interfaces.srv import (
    AcquireLidarControl, ApplyLidarConfiguration, GetLidarConfiguration,
    GetLidarInfo, GetLidarState, ReleaseLidarControl, RenewLidarControl,
    SetLidarCapture, SetLidarMotor, SetLidarMotorSpeed,
    ValidateLidarConfiguration,
)

from .config import load_config, validate_config
from .geometry import CloudFrame, cloud_from_ros, cloud_to_ros, transform_cloud
from .runtime import LidarRuntime, mock_cloud


def _default_config_path() -> str:
    """获取默认配置文件路径。

    Args:
        无参数。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 str。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("embodied_nexus_lidar")) / "config" / "c16.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "c16.yaml")


def _topic(path: str, namespace: str) -> str:
    """话题当前设备或操作。

    Args:
        path: 目标文件、资源或接口的路径。 类型为 str。
        namespace: namespace，作为话题当前设备或操作所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    return path if path.startswith("/") else f"{namespace}/{path.strip('/')}"


class LidarNode(Node):
    """Owns the driver lifecycle and exposes the single public lidar contract."""

    def __init__(self) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        super().__init__("embodied_nexus_lidar")
        self.declare_parameter("config_file", _default_config_path())
        self.declare_parameter("interface_mode", "")
        self.declare_parameter("web_port", 0)
        self.declare_parameter("web_bind_host", "")
        self.declare_parameter("web_auth_token_env", "")
        self.config = load_config(str(self.get_parameter("config_file").value))
        mode = str(self.get_parameter("interface_mode").value).strip()
        if mode:
            self.config["interface_mode"] = mode
        port = int(self.get_parameter("web_port").value)
        if port > 0:
            self.config["web"]["port"] = port
        bind_host = str(self.get_parameter("web_bind_host").value).strip()
        if bind_host:
            self.config["web"]["bind_host"] = bind_host
        token_env = str(self.get_parameter("web_auth_token_env").value).strip()
        if token_env:
            self.config["web"]["auth_token_env"] = token_env
        validate_config(self.config)
        self.runtime = LidarRuntime(self.config)
        self.device_id = self.config["device_id"]
        self.namespace = f"/embodied_nexus/lidars/{self.device_id}"
        self.ros_enabled = self.config["interface_mode"] in {"ros2", "both"}
        self.stream_base = self.namespace if self.ros_enabled else f"{self.namespace}/_internal"
        source = self.config["source"]
        self.points_topic = _topic(source["output_topic"], self.stream_base)
        self.scan_topic = _topic(source["scan_output_topic"], self.stream_base)
        input_topic = _topic(source["pointcloud_topic"], self.namespace)
        if input_topic in {self.points_topic, self.scan_topic}:
            raise ValueError("resolved PointCloud2 input and output topics must be distinct")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Keep sensor callbacks ordered, while allowing control/state services
        # and the status timer to run during CPU-heavy cloud projection.
        self._sensor_group = MutuallyExclusiveCallbackGroup()
        self._difop_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self.points_pub = self.create_publisher(PointCloud2, self.points_topic, qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, qos_profile_sensor_data)
        self.state_pub = self.create_publisher(LidarState, f"{self.namespace}/status", 10)
        self.state_json_pub = self.create_publisher(String, f"{self.namespace}/_internal/state_json", 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data,
            callback_group=self._sensor_group,
        )
        if self.config["backend"] == "c16":
            self.difop_sub = self.create_subscription(
                self._packet_type(), f"{self.namespace}/_driver/difop_packet", self._on_difop, 10,
                callback_group=self._difop_group,
            )
        else:
            self.difop_sub = None

        self._install_services()
        self._mock_timer = (self.create_timer(0.1, self._mock_tick, callback_group=self._sensor_group)
                            if self.config["backend"] == "mock" else None)
        self._status_timer = self.create_timer(0.2, self._publish_state,
                                               callback_group=self._timer_group,
                                               clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._last_published_seq = 0
        self.get_logger().info(
            f"lidar {self.device_id} backend={self.config['backend']} mode={self.config['interface_mode']} "
            f"points={self.points_topic} scan={self.scan_topic}"
        )

    @staticmethod
    def _packet_type():
        """数据包类型。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        from lslidar_msgs.msg import LslidarPacket
        return LslidarPacket

    def _install_services(self) -> None:
        """创建雷达节点的 ROS 服务接口。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        group = self._service_group
        self.create_service(GetLidarInfo, f"{self.namespace}/info", self._get_info, callback_group=group)
        self.create_service(GetLidarState, f"{self.namespace}/get_state", self._get_state, callback_group=group)
        self.create_service(GetLidarConfiguration, f"{self.namespace}/config/get", self._get_config, callback_group=group)
        self.create_service(ValidateLidarConfiguration, f"{self.namespace}/config/validate", self._validate_config, callback_group=group)
        self.create_service(ApplyLidarConfiguration, f"{self.namespace}/config/apply", self._apply_config, callback_group=group)
        self.create_service(SetLidarCapture, f"{self.namespace}/capture", self._set_capture, callback_group=group)
        self.create_service(AcquireLidarControl, f"{self.namespace}/control/acquire", self._acquire, callback_group=group)
        self.create_service(RenewLidarControl, f"{self.namespace}/control/renew", self._renew, callback_group=group)
        self.create_service(ReleaseLidarControl, f"{self.namespace}/control/release", self._release, callback_group=group)
        self.create_service(SetLidarMotor, f"{self.namespace}/motor", self._set_motor, callback_group=group)
        self.create_service(SetLidarMotorSpeed, f"{self.namespace}/motor/speed", self._set_speed, callback_group=group)

    def _on_cloud(self, message: PointCloud2) -> None:
        """处理点云事件。

        Args:
            message: 待处理的消息对象。 类型为 PointCloud2。

        Returns:
            无返回值。
        """
        try:
            frame = cloud_from_ros(message)
            target_frame = str(self.config["filter"].get("target_frame", "")).strip()
            if target_frame and frame.frame_id != target_frame:
                stamp = rclpy.time.Time.from_msg(message.header.stamp)
                tf = self.tf_buffer.lookup_transform(target_frame, frame.frame_id, stamp, timeout=rclpy.duration.Duration(seconds=0.1))
                t = tf.transform.translation
                q = tf.transform.rotation
                frame = transform_cloud(frame, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), target_frame)
            if self.runtime.ingest_cloud(frame):
                self.points_pub.publish(cloud_to_ros(self.runtime.latest_cloud(), PointCloud2()))
                self._publish_latest_scan()
        except Exception as exc:
            self.runtime.set_error(f"point cloud rejected: {exc}")

    def _on_difop(self, message: Any) -> None:
        """接收并记录雷达设备配置数据包。

        Args:
            message: 待处理的消息对象。 类型为 Any。

        Returns:
            无返回值。
        """
        try:
            self.runtime.ingest_difop(bytes(message.data))
        except Exception as exc:
            self.runtime.set_error(f"invalid DIFOP packet: {exc}")

    def _publish_latest_scan(self) -> None:
        """发布latest扫描。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        scan = self.runtime.latest_scan()
        if scan is None:
            return
        msg = LaserScan()
        msg.header.stamp.sec = int(scan.stamp_sec)
        msg.header.stamp.nanosec = int((scan.stamp_sec - int(scan.stamp_sec)) * 1e9)
        msg.header.frame_id = scan.frame_id
        msg.angle_min, msg.angle_max = scan.angle_min, scan.angle_max
        msg.angle_increment, msg.time_increment = scan.angle_increment, scan.time_increment
        msg.scan_time, msg.range_min, msg.range_max = scan.scan_time, scan.range_min, scan.range_max
        msg.ranges, msg.intensities = scan.ranges, scan.intensities
        self.scan_pub.publish(msg)

    def _mock_tick(self) -> None:
        """更新一次雷达 mock 状态和传感器数据。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if not self.runtime.config["mock"]["disconnect"]:
            self._on_mock_cloud(mock_cloud(self.get_clock().now().nanoseconds * 1e-9,
                                           str(self.config["driver"]["frame_id"])))

    def _on_mock_cloud(self, frame: CloudFrame) -> None:
        """处理mock点云事件。

        Args:
            frame: 坐标系，作为处理mock点云事件所需的输入。 类型为 CloudFrame。

        Returns:
            无返回值。
        """
        if self.runtime.ingest_cloud(frame):
            self.points_pub.publish(cloud_to_ros(self.runtime.latest_cloud(), PointCloud2()))
            self._publish_latest_scan()

    def _publish_state(self) -> None:
        """发布状态。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.runtime.tick()
        state = self.runtime.state()
        message = LidarState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.device_id, message.model, message.backend = state["device_id"], state["model"], state["backend"]
        message.lifecycle_state = state["lifecycle_state"]
        message.connected, message.capturing = state["connected"], state["capturing"]
        message.data_fresh, message.point_sequence = state["data_fresh"], state["point_sequence"]
        message.pointcloud_fps = state["pointcloud_fps"]
        message.pointcloud_age_sec = state["pointcloud_age_sec"] if state["pointcloud_age_sec"] is not None else -1.0
        message.frame_id = state["frame_id"]
        message.motor_state = state["motor_state"]
        message.motor_command_sent = state["motor_command_sent"]
        message.hardware_state_confirmed = state["hardware_state_confirmed"]
        message.motor_speed_known = state["motor_speed_known"]
        message.motor_speed_hz = int(state["motor_speed_hz"] or 0)
        message.lease_owner, message.config_version = state["lease_owner"], state["config_version"]
        message.last_error = state["last_error"]
        self.state_pub.publish(message)
        json_message = String()
        json_message.data = json.dumps(state, ensure_ascii=False)
        self.state_json_pub.publish(json_message)

    @staticmethod
    def _response(response, success: bool, error: str = ""):
        """响应当前设备或操作。

        Args:
            response: 服务端或被调用操作返回的响应对象。
            success: success，作为响应当前设备或操作所需的输入。 类型为 bool。
            error: error，作为响应当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。
        """
        response.success = success
        response.error = error
        return response

    def _get_info(self, _request, response):
        """获取信息。

        Args:
            _request: 请求，作为获取信息所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        try:
            response.descriptor_json = json.dumps(self.runtime.descriptor(), ensure_ascii=False)
            return self._response(response, True)
        except Exception as exc:
            response.descriptor_json = "{}"
            return self._response(response, False, str(exc))

    def _get_state(self, _request, response):
        """获取状态。

        Args:
            _request: 请求，作为获取状态所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        response.state_json = json.dumps(self.runtime.state(), ensure_ascii=False)
        return self._response(response, True)

    def _get_config(self, _request, response):
        """读取设备配置并构造服务响应。

        Args:
            _request: 请求，作为读取设备配置并构造服务响应所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        response.config_json = json.dumps(self.runtime.configuration(), ensure_ascii=False)
        return self._response(response, True)

    def _validate_config(self, request, response):
        """验证配置。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            布尔值，表示验证配置是否成功或成立。
        """
        try:
            patch = json.loads(request.patch_json)
            response.config_json = json.dumps(self.runtime.validate_config(patch), ensure_ascii=False)
            response.valid, response.error = True, ""
        except Exception as exc:
            response.valid, response.config_json, response.error = False, "{}", str(exc)
        return response

    def _apply_config(self, request, response):
        """应用配置。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            patch = json.loads(request.patch_json)
            response.config_json = json.dumps(self.runtime.apply_config(patch), ensure_ascii=False)
            response.success, response.error = True, ""
        except Exception as exc:
            response.success, response.config_json, response.error = False, "{}", str(exc)
        return response

    def _set_capture(self, request, response):
        """设置采集。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        response.capturing = self.runtime.set_capture(request.capturing)
        response.success, response.error = True, ""
        return response

    def _acquire(self, request, response):
        """申请设备控制权。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            lease_id, ttl = self.runtime.acquire(request.owner, request.ttl_sec)
            response.success, response.lease_id, response.expires_in_sec, response.error = True, lease_id, ttl, ""
        except Exception as exc:
            response.success, response.error = False, str(exc)
        return response

    def _renew(self, request, response):
        """续租当前设备控制权。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.expires_in_sec = self.runtime.renew(request.lease_id)
            response.success, response.error = True, ""
        except Exception as exc:
            response.success, response.error = False, str(exc)
        return response

    def _release(self, request, response):
        """释放当前设备控制租约。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.stop_sent = self.runtime.release(request.lease_id)
            response.success = True
            response.error = "" if response.stop_sent else self.runtime.state()["last_error"] or "stop command was not sent"
        except Exception as exc:
            response.success, response.error = False, str(exc)
        return response

    def _set_motor(self, request, response):
        """设置控制电机。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            result = self.runtime.set_motor(request.lease_id, request.enabled)
            for key in ("success", "command_sent", "hardware_state_confirmed", "motor_state", "error"):
                setattr(response, key, result[key])
        except Exception as exc:
            response.success, response.error = False, str(exc)
        return response

    def _set_speed(self, request, response):
        """设置速度。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        response.requested_speed_hz = request.speed_hz
        try:
            result = self.runtime.set_motor_speed(request.lease_id, request.speed_hz)
            response.success = result["success"]
            response.command_sent = result["command_sent"]
            response.hardware_state_confirmed = result["hardware_state_confirmed"]
            response.actual_speed_known = result["actual_speed_known"]
            response.actual_speed_hz = result["actual_speed_hz"]
            response.error = result["error"]
        except Exception as exc:
            response.success, response.error = False, str(exc)
        return response

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        state = self.runtime.state()
        if state["backend"] in {"c16", "mock"}:
            sent = self.runtime.shutdown()
            self.get_logger().info(f"shutdown lidar stop command sent={sent}")
        super().destroy_node()


def main(argv=None) -> int:
    """解析启动参数并运行程序入口。

    Args:
        argv: argv，作为解析启动参数并运行程序入口所需的输入。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = LidarNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
