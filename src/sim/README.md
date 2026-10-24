# 仿真实施状态

仿真器无关的后端契约与设备适配器位于 `common/embodied_nexus_sim/`，固定复用 Arachne 提交 `41caa564204eb5f6a39200ce2600ed052ab517b0` 的模型包位于 `common/third_party/embodied_nexus_sim_description/`。Fortress 场景、控制器、传感器桥接、启动配置和 SDK 验收程序位于 `gazebo/embodied_nexus_sim_gazebo/`。后续模拟器使用 `src/sim/<仿真器名>/` 独立接入共享模型和标准 ROS 契约。

本机 WSL2 Ubuntu 22.04 / ROS 2 Humble 已完成 Fortress 构建与首期功能验收。底盘、机械臂、夹爪、RGB-D 相机和 16 线雷达都能通过 Core 与 Windows SDK 分别启动、查询和关闭；底盘避障、机械臂和夹爪顺序协作、RGB-D 与雷达数据读取、腕部相机随机械臂运动改变观测均已通过。Gazebo GUI 的运动前后截图也已检查。详细结果、误差和运行边界见 [验收记录](gazebo/embodied_nexus_sim_gazebo/docs/acceptance-20261024.md)。

最终 Humble 工作区发现 17 个 ROS 包，`colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_sim_gazebo` 全部通过。WSL 测试通过：底盘 22 项、机械臂 17 项、夹爪 16 项、相机 18 项、雷达 18 项、Core 18 项、通用仿真后端及时间戳回退 5 项、Gazebo 资源/传感器转接/时钟守卫 6 项。Windows Python 3.12 SDK 测试 9 项通过，共 129 项。运行组件依赖的 Starlette/Uvicorn/WebSocket 更新在 WSL 用户级 Python 环境中，不改变 ROS 包清单。

在已有 Humble 的 WSL shell 中单独运行通用后端契约测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s src/sim/common/embodied_nexus_sim/test -v
```

首期仿真功能验收已完成：Gazebo 暂停、仿真时钟回跳、租约过期、速度超时和传感器桥接中断的故障语义均已验证；时钟回跳会锁存暂停，重启 launch 后恢复。仿真场景、关节反馈与传感器流已通过本机 Humble 构建、129 项自动测试和 Windows SDK 联调验证。运行性能及不覆盖的真实设备特性见验收记录；仿真结果不替代真实设备验收。
