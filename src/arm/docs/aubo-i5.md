# Aubo i5 机械臂接入

机械臂模块提供独立 ROS 2 节点、受认证的 HTTP/WebSocket 网关和 Python SDK。默认 launch 使用 mock，不连接机器人。机械臂控制柜的 JSON-RPC 服务默认为 `192.168.127.128:30004`；真实配置需显式指定。

在 Ubuntu 22.04 / ROS 2 Humble 构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_arm
source install/setup.bash
ros2 launch embodied_nexus_arm arm.launch.py
```

默认设备 ID 是 `arm_main`，ROS 命名空间为 `/embodied_nexus/arms/arm_main`。通过 HTTP/WebSocket 提供远程接口时，设置 `EMBODIED_NEXUS_ARM_TOKEN` 并启动：

```bash
export EMBODIED_NEXUS_ARM_TOKEN='replace-with-a-local-token'
ros2 launch embodied_nexus_arm arm.launch.py interface_mode:=both
```

SDK 安装方式与其他设备相同：

```bash
python3 -m pip install -e sdk/python
```

客户端先读状态，再显式申请控制权并使能运动。应用正常退出时释放租约；释放会请求机械臂停止。机械臂运动始终使用弧度、弧度每秒和弧度每秒平方。

完成构建后可运行 `python3 src/arm/examples/validate_mock_stack.py`，通过 HTTP/SDK 验证启动、反馈、租约和 mock 关节操作。

真实配置位于 `src/arm/embodied_nexus_arm/config/aubo_i5_real.yaml`，启动时不会自动上电、启动、使能或进入示教模式。运动使能要求控制器反馈新鲜、运行模式为 `Running`、安全模式为 `Normal` 或 `ReducedMode`，且控制器关节范围可读取。关节限制取本地和控制器范围交集。示教模式和关节运动互斥。

基础能力包括关节状态、关节目标、关节速度、上电、启动、下电、freedrive 示教、取消和停止。负载与 TCP 偏移只读；未提供笛卡尔规划、抓取任务或自动下电。

在控制器就绪且现场确认后，实机只读检查可调用 `ArmClient.status()`，检查反馈、模式、安全状态及控制器可返回的负载/TCP 信息。本检查不会发送运动、停止、使能、上下电或示教命令。动作验收另行进行。

## 驱动来源

控制策略依据 Jetson `/home/jetson/zhaoyang/Arachne` 的基线提交 `41caa564204eb5f6a39200ce2600ed052ab517b0`：`src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`、`src/arachne_hardware/arachne_hardware/aubo_sdk/`、`src/arachne_hardware/arachne_hardware/aubo_move_joint_action_server.py` 和 `src/arachne_hardware/launch/real_bringup.launch.py`。本模块重新组织 RPC、状态、生命周期、关节运动和速度控制，不依赖 Arachne 运行目录、面板或 ros2_control。Arachne MIT 许可副本见 [arachne-MIT-LICENSE.txt](../third_party/licenses/arachne-MIT-LICENSE.txt)。

JSON-RPC 的方法与返回值以现有 Arachne 驱动和 Aubo 控制器为准；关节速度查询方法见 [Aubo RobotState API](https://docs.aubo-robotics.cn/arcs_api/zh/group__RobotState.html)。机器人固件响应、真实负载/TCP 字段和实机运动效果尚需在设备现场确认。

实机关节动作及验收记录见[机械臂验证记录](validation-2026-10-19.md)。
