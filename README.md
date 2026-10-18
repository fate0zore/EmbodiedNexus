# EmbodiedNexus

连接多样化机器人设备与具身智能算法的统一接口平台。

## 项目目标

构建一个高可用、易扩展、易适配的具身算法接口平台：统一读取不同机器人设备的状态与信息，并向设备输出标准化动作指令，降低算法与硬件平台之间的集成成本。

## 设计方向

- **设备适配**：面向不同机器人平台与设备形态，提供清晰、可扩展的接入方式。
- **统一接口**：规范设备信息读取与动作输出，减少上层算法对具体硬件的耦合。
- **可靠运行**：围绕可用性、故障处理与运行可观测性进行设计。

## 当前状态

Gemini335 相机、MS42DC 夹爪和 C16 雷达已完成各自适配。Scout 2.0 底盘现提供独立 ROS 2、HTTP/WebSocket、Python SDK 和 mock 实现；WASD 图形示例位于底盘模块 `examples/`。此次底盘实现仅完成离线开发与 mock 验证，没有进行实机移动验收。

## 工作基线

[设备中间层总体规划](todo/设备中间层总体规划.md)是已确认的后续工作基线，包含总体架构、设备接口、配置体系、实施阶段和验收标准。

## 目录与开发约定

项目采用单仓库、统一 ROS 2 工作区，按设备能力聚合模块：

```text
src/camera/    相机实现、ROS 接口、文档和示例
src/base/      底盘实现、ROS 接口、文档和示例
src/gripper/   夹爪实现、ROS 接口、来源许可、文档和示例
src/lidar/     雷达实现、ROS 接口、厂商驱动、文档和示例
sdk/python/    不依赖 ROS 的统一 Python SDK
docs/          总体架构与跨模块说明
todo/          规划与工作基线
```

目录归属、迁移说明和验证命令见[架构说明](docs/architecture.md)，后续开发遵循根目录 [AGENTS.md](AGENTS.md)。

SDK 在仓库根目录执行 `pip install -e sdk/python` 安装，使用 `from embodied_nexus_client import BaseClient, CameraClient, GripperClient, LidarClient`。ROS 2 构建使用 `colcon build --base-paths src --packages-up-to <包名>`；现有 ROS 包名和 launch 入口保持不变。

## 第一期：相机适配

[RGB-D 相机适配与 Yahboom 实机验证指南](src/camera/docs/camera.md)记录 ROS 2 Humble 节点、WebSocket/Python 接口、点云、标定和机器人部署方式。

[Yahboom 实机验证记录（2026-10-18）](src/camera/docs/yahboom-camera-validation-2026-10-18.md)记录设备枚举、三种接口模式、ROS 2 话题输入、实测吞吐和未完成的现场验证项。

## MS42DC 夹爪适配

[MS42DC 夹爪接入与验证指南](src/gripper/docs/ms42dc-gripper.md)记录 ROS 2、HTTP、Python SDK、配置和 Yahboom 实机验收结果。真实后端启动时仍默认禁用运动；本次确认的反馈语义仅适用于已验证的 Yahboom/MS42DC 组合。

## C16 雷达适配

[C16 雷达接入指南](src/lidar/docs/lidar-c16.md)记录真实/mock/已有 ROS 2 话题后端、点云与 LaserScan、租约控制、HTTP/WebSocket、Python SDK 和浏览器预览。驱动来源及许可证见 [lidar-source.md](src/lidar/docs/lidar-source.md)。

## Scout 2.0 底盘适配

[Scout 底盘使用指南](src/base/docs/scout-base.md)记录 mock 与 Waveshare USB-CAN-A 配置、控制租约、速度/相对运动接口和远程 WASD 示例。[驱动来源记录](src/base/docs/scout-source.md)说明 Arachne 与厂商参考及 payload 字节序差异。默认 launch 使用 mock；真实配置需显式指定 `scout_waveshare.yaml`，Web 模式还需设置配置指定的 Bearer token 环境变量。此次没有启动真实底盘驱动或进行实机运动。
