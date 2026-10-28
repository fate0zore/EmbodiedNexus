# 按型号重构与仿真验收记录（2026-10-28）

## 范围

本轮把底盘、机械臂、夹爪、相机和雷达改为“设备能力公共层 + 型号适配器”。本记录覆盖源代码重构、ROS 2 Humble 构建、设备与 SDK 测试、Gazebo Fortress 运行验收。没有连接或操作真实硬件；没有暂存或提交变更。

## 结构与接口变更

- 五类设备配置分别表达 `device_id`、`model` 和 `backend`；相机来源使用 `source.backend`。实例 ID 不参与型号选择，同型号设备可使用不同 ID、配置和命名空间。
- Aubo i5、Scout 2.0、MS42DC、Gemini335、LSLIDAR C16 的协议、状态解释、型号限制和动作转换分别移入所属设备包的 `adapters/`。型号注册、入口点、适配器职责和新增型号方式见[总体架构说明](../../../../../docs/architecture.md#型号适配和注册)。不提供旧根级协议模块的兼容入口。
- Core 和设备服务仍通过公开 ROS 接口或 HTTP 管理设备。仿真专用后端放在 `src/sim/common/embodied_nexus_sim/`，按型号注册 `*.ros2_sim`，不依赖其他设备模块内部代码。
- 机械臂关节动作、速度接口改用变长数组，并以关节名称、数量和限位校验输入/反馈。当前 Aubo i5 和 Gazebo 仿真仍为六关节；四关节契约由临时测试提供方验证，不代表新增了真实四轴型号。接口迁移见[变长关节接口说明](../../../../arm/docs/variable-joint-interface.md)。
- `BaseState` 不再暴露 Scout 状态码，改用 `hardware_estop`、`fault_active`、`control_mode_confirmed`。夹爪状态改用 `feedback_moving`、`feedback_reached`、`feedback_stationary`。雷达状态反馈新鲜度改名为 `device_feedback_age_sec`。更新 ROS 客户端并重新生成接口。

## 构建和自动化测试

在 WSL2 Ubuntu 22.04 / ROS 2 Humble 中使用 `colcon list --base-paths src` 发现 17 个 ROS 包，随后全量执行 `colcon build --base-paths src --symlink-install`，17 个包全部构建通过。

最终设备、网关、仿真、Gazebo 和 SDK 测试共 **161 项通过，0 项失败，0 项跳过**：

| 测试范围 | 结果 |
| --- | ---: |
| 机械臂 | 20 |
| 底盘 | 25 |
| 夹爪 | 18 |
| 相机 | 22 |
| 雷达 | 22 |
| Core | 29 |
| 仿真公共后端 | 5 |
| Gazebo 资源、传感器转接和时钟守卫 | 11 |
| Python SDK | 9 |

额外覆盖了未知型号/后端拒绝、同类设备实例隔离、四关节契约的数量/限位/非有限值检查、租约过期、运动取消与停止确认、底盘和机械臂速度看门狗、夹爪反馈超时，以及相机/雷达数据中断后的过期状态。ROS 话题雷达来源不支持电机控制，已有显式拒绝测试。

## Gazebo Fortress 运行验收

以 `gui:=false performance:=balanced` 启动 Gazebo，并通过 Windows Python SDK 连接 WSL Core。

- `base_main`、`arm_main`、`gripper_main`、`camera_front`、`lidar_main` 分别完成启动、状态/配置读取和关闭；随后五者同时运行并可查询。
- 底盘直行、转向成功；运行中的相对运动被显式取消，操作返回 `cancelled`，零速命令已写入，随后确认静止。取消期间雷达扫描序号前进且扫描新鲜。
- 机械臂关节运动、速度命令和运动取消成功；取消结果得到确认。腕部相机在机械臂移动后返回不同 RGB 帧。
- 夹爪闭合、张开和相对运动取消成功；取消有停止反馈确认。
- 相机返回 RGB 与深度图及标定，时间戳差为 0 ms，深度比例为正。雷达同时返回 PointCloud2（593 点）和 360 个 LaserScan 量程。
- 机械臂与夹爪顺序协作完成；整轮验证结束后 Core 关闭五个设备。

### 仿真时钟回退

通过 Gazebo WorldControl 把仿真时钟从约 459.673 s 重置到 0.454 s。活动底盘转向操作以 `failed` 结束，`stop_requested=true` 且 `zero_velocity_written=true`。时钟守卫暂停世界；手动解除暂停后守卫再次暂停，期间机器人位姿变化为 0 m / 0°。这验证了时钟回退联锁。

时钟故障锁存后，暂停仿真无法提供新的底盘反馈。Core 因而不能确认该设备已静止并保留了设备进程；停止整个仿真时外层清理升级终止进程，最终检查没有遗留 Gazebo/Core 仿真进程。此行为符合“未确认停止时保留设备进程”的停止语义，清除锁存需要重新启动 Gazebo。

### 未验证项和观察

初次尝试按雷达距离触发障碍取消时，底盘移动约 1.39 m，雷达扫描持续更新，但前向最小距离保持约 4.124 m，动作正常完成。因此没有把障碍物联动记为通过；本轮按验收计划改用显式取消验证底盘运动取消。**Gazebo 自动障碍物检测与制动仍未验证**，仿真场景中的该量程现象需要另行排查。

租约超时、速度超时和传感器中断通过设备单元/mock 测试验证，未逐项在 Gazebo 中注入。仿真结果不代表真实控制器、线缆、急停链路或真实传感器行为的验收。

## 复现命令

从仓库根目录执行：

```bash
# WSL Ubuntu 22.04 / ROS 2 Humble
source /opt/ros/humble/setup.bash
colcon list --base-paths src
colcon build --base-paths src --symlink-install
source install/setup.bash
```

设备测试按 `src/<设备>/embodied_nexus_<设备>/test` 目录分别运行；Core、仿真与 SDK 测试目录见上表对应包。Gazebo 启动、Core 地址和验证脚本使用方式见[仿真实施说明](gazebo-fortress.md)。
