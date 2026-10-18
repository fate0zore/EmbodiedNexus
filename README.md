# EmbodiedNexus

连接多样化机器人设备与具身智能算法的统一接口平台。

## 项目目标

构建一个高可用、易扩展、易适配的具身算法接口平台：统一读取不同机器人设备的状态与信息，并向设备输出标准化动作指令，降低算法与硬件平台之间的集成成本。

## 设计方向

- **设备适配**：面向不同机器人平台与设备形态，提供清晰、可扩展的接入方式。
- **统一接口**：规范设备信息读取与动作输出，减少上层算法对具体硬件的耦合。
- **可靠运行**：围绕可用性、故障处理与运行可观测性进行设计。

## 当前状态

第一期 Gemini335 相机适配已实现，并已在 Yahboom 机器人上验证 ROS 2、Web、点云、配置接口和 ROS 2 话题接入；RGB 标定求解尚待标定板条件具备后实测。MS42DC 夹爪已增加独立驱动、mock、ROS 2、HTTP 和 Python SDK 适配，并于 2026-10-18 在 Yahboom 上完成闭合/打开实机验证。

## 工作基线

[设备中间层总体规划](todo/设备中间层总体规划.md)是已确认的后续工作基线，包含总体架构、设备接口、配置体系、实施阶段和验收标准。

## 第一期：相机适配

[RGB-D 相机适配与 Yahboom 实机验证指南](docs/camera.md)记录 ROS 2 Humble 节点、WebSocket/Python 接口、点云、标定和机器人部署方式。

[Yahboom 实机验证记录（2026-10-18）](docs/yahboom-camera-validation-2026-10-18.md)记录设备枚举、三种接口模式、ROS 2 话题输入、实测吞吐和未完成的现场验证项。

## MS42DC 夹爪适配

[MS42DC 夹爪接入与验证指南](docs/ms42dc-gripper.md)记录 ROS 2、HTTP、Python SDK、配置和 Yahboom 实机验收结果。真实后端启动时仍默认禁用运动；本次确认的反馈语义仅适用于已验证的 Yahboom/MS42DC 组合。

## C16 雷达适配

[C16 雷达接入指南](docs/lidar-c16.md)记录真实/mock/已有 ROS 2 话题后端、点云与 LaserScan、租约控制、HTTP/WebSocket、Python SDK 和浏览器预览。驱动来源及许可证见 [lidar-source.md](docs/lidar-source.md)。
