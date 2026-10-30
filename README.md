# EmbodiedNexus

连接多样化机器人设备与具身智能算法的统一接口平台。

## 项目目标

构建一个高可用、易扩展、易适配的具身算法接口平台：统一读取不同机器人设备的状态与信息，并向设备输出标准化动作指令，降低算法与硬件平台之间的集成成本。

## 设计方向

- **设备适配**：面向不同机器人平台与设备形态，提供清晰、可扩展的接入方式。
- **统一接口**：规范设备信息读取与动作输出，减少上层算法对具体硬件的耦合。
- **可靠运行**：围绕可用性、故障处理与运行可观测性进行设计。

## 当前状态

Gemini335 相机、MS42DC 夹爪和 C16 雷达已完成各自适配。Scout 2.0 底盘与 Aubo i5 机械臂提供独立 ROS 2、HTTP/WebSocket、Python SDK 和 mock 实现；WASD 图形示例位于底盘模块 `examples/`。机械臂默认启动 mock，实机只读与动作验收状态见[机械臂指南](src/arm/docs/aubo-i5.md)。

`src/sim/` 提供 MuJoCo 与 Gazebo 仿真入口、共享 Arachne 移动机械臂模型，以及通用 `ros2_sim` 底盘、机械臂和夹爪后端；RGB-D 相机和 16 线雷达通过标准 ROS 话题接入。五类设备已完成 Gazebo 本机 SDK 联调；MuJoCo 支持同一设备平台和 Core 生命周期。安装、构建与启动步骤见 [MuJoCo 仿真指南](src/sim/mujoco/README.md) 和 [Gazebo 仿真指南](src/sim/gazebo/embodied_nexus_sim_gazebo/docs/gazebo-fortress.md)，仿真记录见 [仿真实施记录](src/sim/README.md)。

## 工作基线

[设备中间层总体规划](todo/设备中间层总体规划.md)是已确认的后续工作基线，包含总体架构、设备接口、配置体系、实施阶段和验收标准。

## 目录与开发约定

项目采用单仓库、统一 ROS 2 工作区，按设备能力聚合模块：

```text
src/camera/    相机实现、ROS 接口、文档和示例
src/base/      底盘实现、ROS 接口、文档和示例
src/arm/       机械臂实现、ROS 接口、来源许可、文档和示例
src/gripper/   夹爪实现、ROS 接口、来源许可、文档和示例
src/lidar/     雷达实现、ROS 接口、厂商驱动、文档和示例
sdk/python/    不依赖 ROS 的统一 Python SDK
src/core/      五类设备的统一 ROS 管理节点与 HTTP/WebSocket 入口
docs/          总体架构与跨模块说明
todo/          规划与工作基线
scripts/       全仓 ROS 构建与跨设备服务的统一管理入口
```

目录归属、迁移说明和验证命令见[架构说明](docs/architecture.md)，后续开发遵循根目录 [AGENTS.md](AGENTS.md)。

## 本地依赖与全量构建

ROS 2 工作区在 Ubuntu 22.04 / WSL2 Ubuntu-22.04 和 ROS 2 Humble 下构建。以下 Bash 命令均从仓库根目录执行。首次配置时安装构建工具和清单中声明的 ROS 依赖；`rosdep init` 只需在该发行版首次配置时执行一次：

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
python3 -m pip install --user --upgrade "starlette>=1.3.1" "uvicorn>=0.15" "websockets>=12"
```

若 `rosdep init` 提示已初始化，跳过该命令即可。根目录 `scripts/build.sh` 是全仓 ROS 构建入口，会递归发现设备接口、设备节点、Core、共享仿真后端和 Gazebo 仿真包。默认目标是全工作区：

```bash
bash scripts/build.sh all
```

增量构建、串行构建和自定义输出目录示例：

```bash
bash scripts/build.sh embodied_nexus_base
bash scripts/build.sh sim --serial
bash scripts/build.sh all --build-base /root/embodied-nexus-run/build \
  --install-base /root/embodied-nexus-run/install \
  --log-base /root/embodied-nexus-run/log
```

目标可选 `all`、`core`、`sim` 或任一 ROS 包名。Windows PowerShell 可从仓库根目录运行 `./scripts/build.ps1 all`，脚本通过 WSL 调用同一 Bash 构建入口；默认使用 `Ubuntu-22.04`，可通过 `EMBODIED_NEXUS_WSL_DISTRO` 指定发行版。更多节点生命周期和 WSL 工作区用法见[节点命令参考](docs/node-command-reference.md)。

仓库位于 WSL 的 `/mnt/<盘符>/...` 时，默认 build/install/log 目录位于 `${XDG_CACHE_HOME:-~/.cache}/embodied-nexus`，避免 Windows 挂载盘不支持安装源码符号链接。构建结束会打印应 source 的工作区路径；`scripts/services.sh` 会自动找到这个默认安装目录。自定义输出目录时，可将打印路径设置到 `EMBODIED_NEXUS_SETUP` 供服务脚本使用。

Web RViz 前端需要 Node.js 20+ 和 npm；运行 `bash src/core/scripts/core.sh build core` 可构建前端及 Core ROS 包。SDK 安装命令见下方。

Python SDK 不依赖 ROS，Python 3.10+ 环境在仓库根目录安装：

```bash
python -m pip install -e sdk/python
```

安装后可通过 `from embodied_nexus_client import ArmClient, BaseClient, CameraClient, GripperClient, LidarClient` 导入。Gazebo 仿真启动与验收步骤见 [Gazebo 仿真指南](src/sim/gazebo/embodied_nexus_sim_gazebo/docs/gazebo-fortress.md)；构建和测试记录见[仿真实施记录](src/sim/README.md)。

## 第一期：相机适配

[RGB-D 相机适配与 Yahboom 实机验证指南](src/camera/docs/camera.md)记录 ROS 2 Humble 节点、WebSocket/Python 接口、点云、标定和机器人部署方式。

[Yahboom 实机验证记录（2026-10-18）](src/camera/docs/yahboom-camera-validation-2026-10-18.md)记录设备枚举、三种接口模式、ROS 2 话题输入、实测吞吐和未完成的现场验证项。

## MS42DC 夹爪适配

[MS42DC 夹爪接入与验证指南](src/gripper/docs/ms42dc-gripper.md)记录 ROS 2、HTTP、Python SDK、配置和 Yahboom 实机验收结果。真实后端启动时仍默认禁用运动；本次确认的反馈语义仅适用于已验证的 Yahboom/MS42DC 组合。

## C16 雷达适配

[C16 雷达接入指南](src/lidar/docs/lidar-c16.md)记录真实/mock/已有 ROS 2 话题后端、点云与 LaserScan、租约控制、HTTP/WebSocket、Python SDK 和浏览器预览。驱动来源及许可证见 [lidar-source.md](src/lidar/docs/lidar-source.md)。

## Scout 2.0 底盘适配

[Scout 底盘使用指南](src/base/docs/scout-base.md)记录 mock 与 Waveshare USB-CAN-A 配置、控制租约、速度/相对运动接口和远程 WASD 示例。[驱动来源记录](src/base/docs/scout-source.md)说明 Arachne 与厂商参考及 payload 字节序差异。默认 launch 使用 mock；真实配置需显式指定 `scout_waveshare.yaml`，Web 模式还需设置配置指定的 Bearer token 环境变量。此次没有启动真实底盘驱动或进行实机运动。

## Aubo i5 机械臂

[机械臂接入指南](src/arm/docs/aubo-i5.md)记录 mock/真实配置、关节状态和控制接口、控制租约、运动与生命周期检查、HTTP/WebSocket 和 SDK 用法。真实配置需显式指定 `aubo_i5_real.yaml`；真实后端启动不自动上电或使能。

[机械臂验证记录](src/arm/docs/validation-2026-10-19.md)列出 Windows 本地测试、Jetson Humble mock 联调和 Aubo 实机只读检查结果；实机动作验收尚未执行。

## 统一 Core 管理节点

`nexus_core` 从平台 YAML 加载底盘、机械臂、夹爪、相机和雷达清单，默认使用 `platform_mock.yaml`。core 启动后设备保持关闭，可通过 ROS 管理服务或 Python SDK 按 ID 启动、关闭和查询，也可使用批量管理；单设备故障不影响其他设备。执行设备关闭前，core 会检查停止反馈，无法确认停稳时保留设备进程和管理入口。

在 Ubuntu 22.04 / ROS 2 Humble 上构建；Web RViz 前端构建需要 Node.js 20+ 和 npm，桥接使用 rosbridge 2.0.8+：

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-rosbridge-server
python3 -m pip install --user "starlette>=1.3.1" "uvicorn>=0.15" "websockets>=10.4"
bash src/core/scripts/core.sh build core
source install/setup.bash
python3 -m pip install -e sdk/python
export EMBODIED_NEXUS_CORE_TOKEN='replace-with-a-private-url-safe-token'
ros2 run embodied_nexus_core nexus_core --config src/core/embodied_nexus_core/config/platform_mock.yaml
```

远程 Python 客户端连接 core 的同一地址，例如 `CoreClient("http://robot-host:8780", token)`；可运行 [Core SDK 示例](sdk/python/examples/core_client.py) 查询状态、管理实例或读取相机信息。平台实机模板 `platform_real.yaml` 明确引用现有真实设备配置，启动真实设备前按设备指南确认硬件就绪。平台字段、管理接口和更多 SDK 用法见[架构说明](docs/architecture.md#统一-core-管理节点)。

浏览器访问 Core 同源根路径可打开运维面板；首次访问需输入 Core token，面板可查看设备状态、管理设备生命周期及读取平台配置。token 仅保存在当前浏览器本地，不写入平台配置或服务端页面资源。

面板中的“Web RViz”入口可在同源 `/rviz/` 只读查看已发布的机器人模型、TF、点云、LaserScan、RGB 和深度图像。完整构建、话题预设和模型资源配置见 [Core Web RViz 指南](src/core/docs/web-rviz.md)。

Core 和设备节点的 Mock/仿真启动、停止命令，以及 PowerShell/WSL 与 Bash 用法见[跨平台节点命令参考](docs/node-command-reference.md)。
