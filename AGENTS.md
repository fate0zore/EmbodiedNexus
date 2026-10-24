# EmbodiedNexus 开发约定

## 项目与工作基线

EmbodiedNexus 是机器人设备读取与操作中间层。机器人端使用 Ubuntu 22.04 / ROS 2 Humble；统一 Python SDK 支持 Python 3.10 及以上的 Windows/Linux 客户端。

开始工作前阅读 [README](README.md)、[目录与架构说明](docs/architecture.md)及[已确认的总体规划](todo/设备中间层总体规划.md)。保留用户未提交的修改，不将 IDE 文件或构建产物纳入变更。

## 目录归属

- `src/` 按设备能力聚合 `base/`、`arm/`、`camera/`、`gripper/`、`lidar/`，并包含跨设备管理模块 `core/`；具体厂商和型号属于设备模块内部的后端、驱动或配置。
- 仿真器无关后端与共享模型放在 `src/sim/common/`，各仿真器独立放在 `src/sim/<仿真器名>/`；ROS 包归属对应仿真器或共享分组目录。
- 模块目录收拢功能包、接口包、专用第三方源码、文档和示例。模块根目录不放 `package.xml` 或 `setup.py`；内部 ROS 包仍独立构建、安装和启动。
- `config/`、`launch/`、`resource/`、`test/` 留在所属 ROS 包内，保留安装后的包资源查找方式。
- 设备指南、来源说明和验收记录放在模块的 `docs/`，设备专用示例放在模块的 `examples/`。
- 根目录 `docs/` 只放总体架构和跨模块说明，`todo/` 保留规划；统一 SDK 放在 `sdk/python/`。
- 新增设备沿用以上结构，只创建实际需要的目录，不预建空模块或无实际使用者的公共层。

## 模块边界与依赖

- 每种设备独立配置、启动和停止；其他设备缺席不能成为启动前提。
- 跨模块协作使用公开 ROS 接口或远程接口，不导入其他设备的内部实现，不通过源码目录拼接或 `sys.path` 注入建立依赖。
- `core/` 通过设备公开 launch、ROS 接口与 HTTP/WebSocket 入口管理独立设备进程；不得导入设备模块内部实现。
- 网关通过设备服务访问能力，硬件连接与控制规则归设备服务及其驱动；修改通信层时保持已有控制权、停止和反馈语义。
- 普通可安装依赖在 `package.xml`、`setup.py` 或 `pyproject.toml` 声明。必须随仓库保存的专用第三方源码放在所属模块的 `third_party/`。
- 第三方代码保留许可证、上游版本及本地补丁说明。来源记录中的路径应明确属于上游仓库还是当前仓库。

## Python SDK

- 安装包名为 `embodied-nexus-client`，导入名为 `embodied_nexus_client`；不新增旧相机包名兼容入口。
- 客户端按设备能力放入 `base.py`、`camera.py`、`gripper.py`、`lidar.py` 等模块，并在顶层导出公开类型；SDK 测试放在 `sdk/python/tests/`。
- SDK 不依赖 ROS，不直接打开机器人硬件。示例和验收脚本使用已安装 SDK，不依赖它在仓库中的相对位置。
- 在仓库根目录执行 `python -m pip install -e sdk/python` 安装开发版。

## 修改与验证

- 移动目录或修改公开接口时，同步更新代码引用、资源路径、安装说明、文档链接、示例和验证命令。
- 文档中的仓库相对命令默认从根目录执行；Markdown 链接相对当前文档。历史验收记录保留原始事实，路径更新不代表重新完成实机验收。
- ROS 包使用 `colcon list --base-paths src` 发现，使用 `colcon build --base-paths src --symlink-install --packages-up-to <包名>` 构建；新增目录层级不能改变包名或破坏递归发现。
- Python 测试按功能包分别运行 `python -m unittest discover -s <包目录>/test -v`，未安装包时为当前进程设置该包目录的 `PYTHONPATH`；SDK 测试运行 `python -m unittest discover -s sdk/python/tests -v`。
- 根据变更运行相关检查，区分本地 Python、Humble 构建与 mock 验证、实机验证。报告实际通过、跳过、失败与未执行项；既有问题与本次回归分别说明。
- 实机动作验证遵循总体规划：用户明确确认机器人就绪后进行。目录或文档维护使用本地检查与 mock 验证。

## Jetson Git 同步

- 本机 `main` 是源码修改主工作区，Jetson 的 `~/EmbodiedNexus` 是 Git 管理的构建与运行工作区；不在 Jetson 上直接修改或提交源码。
- 本机提交变更后显式执行 `git push yahboom main` 同步。该命令只推送已提交内容；不要自动暂存或提交用户修改，也不要改变 `origin` 或 `main` 的上游设置。
- Jetson 通过 `receive.denyCurrentBranch=updateInstead` 接收推送并更新当前工作树。若远端有已跟踪文件修改，推送会被拒绝；先检查并保留、协调这些改动，不用强制推送覆盖。
- Jetson 构建仍使用 Humble 和 `colcon --base-paths src`；工作区操作命令见 [架构说明](docs/architecture.md#jetson-远端工作流)。

当前目录迁移的测试基线与已知夹爪语法问题见[架构说明中的验证记录](docs/architecture.md#迁移验证基线)。
