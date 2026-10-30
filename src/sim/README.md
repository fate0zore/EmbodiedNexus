# 仿真实施状态

仿真器无关的后端契约与设备适配器位于 `common/embodied_nexus_sim/`，固定复用 Arachne 提交 `41caa564204eb5f6a39200ce2600ed052ab517b0` 的模型包位于 `common/third_party/embodied_nexus_sim_description/`。MuJoCo ROS bridge 位于 `mujoco/embodied_nexus_sim_mujoco/`；Fortress 场景、控制器、传感器桥接、启动配置和 SDK 验收程序位于 `gazebo/embodied_nexus_sim_gazebo/`。两个模拟器都通过共享模型和标准 ROS 契约连接设备与 Core。

ROS 仿真设备适配器按型号保存在 `common/embodied_nexus_sim/embodied_nexus_sim/adapters/`：分别为 `aubo_i5`、`scout_2` 和 `ms42dc`。它们的后端 entry point 名称包含型号（例如 `aubo_i5.ros2_sim`），与设备配置中的 `model`、`backend: ros2_sim` 对应。变更后需重新构建并 source 工作区，以更新已安装的 entry point；原来的单一 `ros2_sim` entry point 不再使用。

## 本轮型号适配重构验收（2026-10-28）

WSL2 Ubuntu 22.04 / ROS 2 Humble 发现并全量构建 17 个 ROS 包。最终测试通过：机械臂 20 项、底盘 25 项、夹爪 18 项、相机 22 项、雷达 22 项、Core 29 项、仿真公共后端 5 项、Gazebo 11 项、Python SDK 9 项，共 161 项。各设备模拟租约过期、速度超时和传感器数据过期检查在设备单元测试中通过；Gazebo 时钟回退也完成集成验证。

Gazebo Fortress headless 验收通过五类设备独立启停与查询、底盘运动/显式取消、机械臂运动/速度/取消、夹爪开合/取消、相机 RGB-D 图像与标定、雷达点云与 LaserScan，以及机械臂和夹爪顺序协作。仿真时钟回退后，活动底盘操作失败并写入零速，守卫保持世界暂停。具体结果及边界见[本轮验收记录](gazebo/embodied_nexus_sim_gazebo/docs/model-adapter-validation-2026-10-28.md)。

场景中的前向雷达最小距离在底盘移动时保持约 4.124 m，未展示与运动同步变化；因此本轮将底盘验收聚焦于计划要求的显式取消，**没有验证自动障碍物停止**。时钟故障锁存后，暂停世界无法提供新反馈，Core 未确认该底盘已停稳并保留设备进程；停止 Gazebo 时清理过程升级终止，最终未遗留仿真进程。所有结果均为仿真/mock，不构成实机验收。2026-10-24 的首期仿真记录仍保留为历史结果，见[历史验收记录](gazebo/embodied_nexus_sim_gazebo/docs/acceptance-20261024.md)。

在已有 Humble 的 WSL shell 中单独运行通用后端契约测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s src/sim/common/embodied_nexus_sim/test -v
```

首期仿真功能验收已完成：Gazebo 暂停、仿真时钟回跳、租约过期、速度超时和传感器桥接中断的故障语义均已验证；时钟回跳会锁存暂停，重启 launch 后恢复。仿真场景、关节反馈与传感器流已通过本机 Humble 构建、129 项自动测试和 Windows SDK 联调验证。运行性能及不覆盖的真实设备特性见验收记录；仿真结果不替代真实设备验收。
