# 目录结构与模块边界

## 组织方式

项目采用单仓库、统一 ROS 2 工作区，以设备能力作为源码归属边界。一个设备模块聚合多个构建包和设备专用资料；模块目录本身不是 ROS 包。配置、launch 和包测试留在所属构建包内。

```text
EmbodiedNexus/
├── AGENTS.md
├── README.md
├── docs/
│   └── architecture.md
├── todo/
├── sdk/python/
│   ├── pyproject.toml
│   ├── embodied_nexus_client/
│   │   ├── __init__.py
│   │   ├── camera.py
│   │   ├── gripper.py
│   │   └── lidar.py
│   └── tests/
└── src/
    ├── camera/
    │   ├── embodied_nexus_camera/
    │   ├── embodied_nexus_camera_interfaces/
    │   ├── docs/
    │   └── examples/
    ├── gripper/
    │   ├── embodied_nexus_gripper/
    │   ├── embodied_nexus_gripper_interfaces/
    │   ├── third_party/licenses/
    │   ├── docs/
    │   └── examples/
    └── lidar/
        ├── embodied_nexus_lidar/
        ├── embodied_nexus_lidar_interfaces/
        ├── third_party/lslidar_c16_ros2/
        │   ├── LICENSE
        │   ├── UPSTREAM_README.md
        │   ├── lslidar_driver/
        │   └── lslidar_msgs/
        ├── docs/
        └── examples/
```

功能包使用 `ament_python`，自定义接口包使用 `ament_cmake` / `rosidl`，因此继续保留独立构建边界。厂商雷达包保持原包名和来源材料。colcon 的递归发现允许设备目录作为中间分组层，构建时用 `--base-paths src` 限定范围，详见 [colcon 官方发现参数](https://colcon.readthedocs.io/en/released/reference/discovery-arguments.html)。

## 资料与依赖归属

| 内容 | 归属 |
| --- | --- |
| 设备实现、配置、launch、包测试 | `src/<设备能力>/embodied_nexus_<设备能力>/` |
| 自定义 ROS 消息、服务、action | 同模块的 `embodied_nexus_<设备能力>_interfaces/` |
| 设备指南、来源、实机记录 | `src/<设备能力>/docs/` |
| 设备专用示例 | `src/<设备能力>/examples/` |
| 随仓库保存的专用第三方代码与许可 | `src/<设备能力>/third_party/` |
| 通用远程 Python 客户端与测试 | `sdk/python/` |
| 总体架构与跨模块说明 | 根目录 `docs/` |
| 已确认规划 | 根目录 `todo/` |

设备模块通过公开接口协作，不互相导入内部实现。SDK 依赖远程协议，不依赖 ROS 或机器人端源码。普通 Python、系统和 ROS 依赖继续通过包清单声明；只有需要随项目分发的专用依赖才存入模块的 `third_party/`。

设备入口：[相机](../src/camera/docs/camera.md)、[夹爪](../src/gripper/docs/ms42dc-gripper.md)、[雷达](../src/lidar/docs/lidar-c16.md)。开发约定统一维护在根目录 [AGENTS.md](../AGENTS.md)。

## 迁移说明

| 原位置或名称 | 新位置或名称 |
| --- | --- |
| `src/embodied_nexus_<设备能力>*` | `src/<设备能力>/embodied_nexus_<设备能力>*` |
| `src/third_party/lslidar_c16_ros2/` | `src/lidar/third_party/lslidar_c16_ros2/` |
| `third_party/licenses/` | `src/gripper/third_party/licenses/` |
| 根目录 `docs/`、`examples/` 中的设备专用资料 | 所属模块的 `docs/`、`examples/` |
| SDK 安装包 `embodied-nexus-camera-client` | `embodied-nexus-client` |
| SDK 导入 `embodied_nexus_camera_client` | `embodied_nexus_client` |
| SDK 相机子模块 `client.py` | `camera.py` |
| `sdk/python/test_gripper_sdk.py` | `sdk/python/tests/test_gripper_sdk.py` |

SDK 直接改名，不提供旧名称兼容入口。更新外部程序的安装依赖和导入；如果原环境装过旧分发包，先执行 `python -m pip uninstall embodied-nexus-camera-client`，再安装新包，避免环境中残留旧版本。公开客户端、帧、租约与操作句柄的类名和方法签名不变。ROS 包名、launch 入口、设备命名空间及 HTTP/WebSocket 协议不变。

雷达模式验收脚本使用已安装 SDK，不再按父目录层数推导 SDK 源码位置。迁移已有工作区后重新构建并 source 新的安装环境，不复用指向旧源码位置的构建产物。

## 开发与验证命令

以下命令从仓库根目录执行。Windows 使用 Python 核心和 SDK 测试；ROS 2 命令在 Ubuntu 22.04 / Humble 环境执行。

安装统一 SDK：

```bash
python -m pip install -e sdk/python
python -m unittest discover -s sdk/python/tests -v
```

Linux/macOS shell 的核心测试命令，每个包独立运行：

```bash
PYTHONPATH=src/camera/embodied_nexus_camera python -m unittest discover -s src/camera/embodied_nexus_camera/test -v
PYTHONPATH=src/gripper/embodied_nexus_gripper python -m unittest discover -s src/gripper/embodied_nexus_gripper/test -v
PYTHONPATH=src/lidar/embodied_nexus_lidar python -m unittest discover -s src/lidar/embodied_nexus_lidar/test -v
```

PowerShell 对应写法如下；切换设备时同步替换 `PYTHONPATH` 和测试目录：

```powershell
$env:PYTHONPATH = "$PWD/src/camera/embodied_nexus_camera"
python -m unittest discover -s src/camera/embodied_nexus_camera/test -v
```

核心测试依赖见各包的 `setup.py` / `package.xml`；Web 测试还需要 `starlette`、`httpx`。SDK 数据流使用 `websockets`。

Humble 环境中发现包并分别构建设备：

```bash
source /opt/ros/humble/setup.bash
colcon list --base-paths src
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_camera
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_gripper
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_lidar
source install/setup.bash
python3 -m pip install -e sdk/python
python3 src/lidar/embodied_nexus_lidar/test/validate_lidar_modes.py
```

应发现 8 个 ROS 包：三类设备各一个功能包和接口包，以及 `lslidar_driver`、`lslidar_msgs`。雷达验收脚本只运行 mock 与已有 ROS 话题后端，覆盖 `ros2/web/both` 和 SDK 数据流；实机验收另按设备指南及[总体规划](../todo/设备中间层总体规划.md)执行。

## Jetson 远端工作流

本机 `main` 是源码修改主工作区；Jetson 通过 SSH 别名 `yahboom` 管理 `/home/jetson/EmbodiedNexus`，用于构建和运行。Jetson 仓库启用 `receive.denyCurrentBranch=updateInstead`，本机推送会直接更新它检出的 `main` 工作树。GitHub `origin` 保持不变，Jetson 不需要访问 GitHub。

源码修改在本机完成并提交后，显式同步到 Jetson：

```bash
git push yahboom main
```

该命令只发送已提交的 `main`，不会改动 `origin` 上游或同步本机未提交文件。Jetson 不直接修改源码；若推送因远端已跟踪文件修改而被拒绝，先检查远端工作区并保留、协调改动后再推送，不使用强制覆盖。`build/`、`install/` 和 `log/` 由 `.gitignore` 排除，不参与同步。

推送后在 Jetson 检查版本并构建：

```bash
ssh yahboom "cd ~/EmbodiedNexus && git status --short && git rev-parse --short HEAD"
ssh yahboom "cd ~/EmbodiedNexus && source /opt/ros/humble/setup.bash && colcon list --base-paths src"
ssh yahboom "cd ~/EmbodiedNexus && source /opt/ros/humble/setup.bash && colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_camera && colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_gripper && colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_lidar"
```

## 迁移验证基线

2026-10-18 在迁移前的 Windows / Python 3.12 环境检查：相机 17 项、雷达核心 15 项、SDK 1 项通过；夹爪和雷达各 2 项 Web 测试因依赖缺失跳过。这些结果不替代 Humble 构建或实机验证，历史设备记录也不代表本次迁移后已重新验收。

夹爪包内 `backend.py`、`cli.py`、`controller.py`、`gripper_node.py`、`ros_gateway.py`、`web_server.py` 在迁移前已存在末尾字面量 `\n`，导致语法检查失败并阻止夹爪核心测试导入。本次目录与 SDK 命名迁移保留这些文件内容，作为既有问题单独记录。

当前 Windows 环境没有 `colcon`、`ros2`；迁移后的本地检查与尚待 Humble 环境执行的检查分别报告。
