# 目录结构与模块边界

## 型号适配和注册

设备能力仍是公开 ROS 接口与 Python SDK 的边界；设备配置中的实例 ID 与 `model` 分别表示设备实例和型号，`backend`（相机使用 `source.backend`）表示实际设备、mock 或 ROS 话题/仿真来源。不同实例应使用独立 ID、配置与命名空间。相机型号字段为 `model: gemini335`；已有相机配置未提供该字段时采用 Gemini335 默认值。

各设备包内的 `adapters/<型号>/` 保存现有型号的参数校验、协议、控制或采集实现。公共设备服务负责租约、操作生命周期、状态与停止，型号适配器解释硬件数据和动作；仿真后端继续由 `embodied_nexus_sim` 提供，不导入其他设备的内部代码。型号提供方按设备类型注册到以下 Python entry point 分组；每个名称必须唯一，缺失或重复时拒绝配置：

| 设备 | 型号 entry point | 当前内置型号 | 提供方职责 |
| --- | --- | --- | --- |
| 机械臂 | `embodied_nexus_arm.models` | `aubo_i5` | 校验配置，创建后端和语义操作适配器 |
| 底盘 | `embodied_nexus_base.models` | `scout_2` | 提供默认配置、运行时可变字段、校验、后端和状态策略 |
| 夹爪 | `embodied_nexus_gripper.models` | `ms42dc` | 提供默认配置、运行时可变字段、校验、后端和动作/反馈适配器 |
| 相机 | `embodied_nexus_camera.models` | `gemini335` | 提供默认配置、运行时可变字段、校验和真实采集入口 |
| 雷达 | `embodied_nexus_lidar.models` | `lslidar_c16` | 提供默认配置、运行时可变字段、校验、控制和 ROS 驱动启动入口 |

第三方提供方注册的 entry point 指向无参构造的提供方类，方法契约以各内置型号的 `model.py` 和相应设备的控制器为准；提供方必须声明真实支持的能力并保持停止、反馈与控制权语义。`mock` 是模拟数据而非硬件反馈；ROS 话题来源不隐含支持雷达电机控制。添加型号时同时提供相应配置和契约测试，不应以命令目标冒充测量值。

配置同时填写 `device_id`、`model` 与 `backend`；它们分别标识运行实例、设备型号和连接/数据来源。相机把来源配置放在 `source.backend`。例如同一型号的两台机械臂使用不同的 `device_id`、独立配置文件及命名空间；型号专属字段由对应 `adapters/<型号>/config.py` 校验，型号不支持的后端或能力会拒绝启动或操作。实例 ID 不用于型号选择。

型号专属实现入口如下：Aubo i5 的 RPC 与动作语义在 `src/arm/embodied_nexus_arm/embodied_nexus_arm/adapters/aubo_i5/`；Scout 2.0 的串口、帧协议、反馈解释和运动安全策略在 `src/base/embodied_nexus_base/embodied_nexus_base/adapters/scout_2/`；MS42DC 的串口协议、动作映射和反馈解释在 `src/gripper/embodied_nexus_gripper/embodied_nexus_gripper/adapters/ms42dc/`；Gemini335 的 V4L2 采集在 `src/camera/embodied_nexus_camera/embodied_nexus_camera/adapters/gemini335/`；LSLIDAR C16 的 DIFOP、UDP 电机控制与驱动启动在 `src/lidar/embodied_nexus_lidar/embodied_nexus_lidar/adapters/c16/`。公共层提供统一 ROS 服务、标准状态或图像/点云/扫描、控制租约和停止语义；厂商专属 Python 模块只在型号适配器内导入。型号协议不再通过设备包根级 `protocol.py` 或公共后端模块导出。

跨型号状态字段也保持通用：底盘 `BaseState` 用 `hardware_estop`、`fault_active` 和 `control_mode_confirmed` 表达安全状态，不发布 Scout 状态码；夹爪状态提供 `feedback_moving`、`feedback_reached` 和 `feedback_stationary`，不发布 MS42DC 原始字节字段；雷达状态将型号状态反馈的新鲜度统一称为 `device_feedback_age_sec`。设备状态均包含实例 ID、型号和后端；旧 ROS 消息字段需要按新接口重新生成并更新调用方。

接入新型号时，在所属设备包新增 `adapters/<型号>/model.py` 及它需要的协议、后端、配置和语义实现；通过 Python entry point 注册该型号，例如 `embodied_nexus_arm.models = ["vendor_model = vendor_package:VendorArmModel"]`。提供方只实现对应设备能力实际使用的方法，遵守本设备控制器消费的适配器契约；另为 `mock` 或实机连接提供明确后端选择，并在 `embodied_nexus_sim` 中需要仿真时注册 `<型号>.ros2_sim` 后端。为型号配置、未知型号/后端、多个实例和设备能力契约增加测试。只为已有实际型号创建目录，不以空适配器宣称支持新硬件。

机械臂 ROS 动作和速度服务现在使用变长关节数组；关节名称、目标和限位数量必须一致。Aubo i5 配置仍为六关节；ROS 客户端需要重新生成接口，具体迁移见[机械臂接口说明](../src/arm/docs/variable-joint-interface.md)。

## 组织方式

项目采用单仓库、统一 ROS 2 工作区，以设备能力作为源码归属边界。一个设备模块聚合多个构建包和设备专用资料；模块目录本身不是 ROS 包。配置、launch 和包测试留在所属构建包内。

```text
EmbodiedNexus/
├── AGENTS.md
├── README.md
├── scripts/       # 全仓构建与跨模块服务入口
├── docs/
│   └── architecture.md
├── todo/
├── sdk/python/
│   ├── pyproject.toml
│   ├── embodied_nexus_client/
│   │   ├── __init__.py
│   │   ├── camera.py
│   │   ├── gripper.py
│   │   ├── base.py
│   │   ├── arm.py
│   │   └── lidar.py
│   └── tests/
└── src/
    ├── core/
    │   ├── scripts/
    │   │   ├── core.sh
    │   │   └── core.ps1
    │   ├── embodied_nexus_core/
    │   └── embodied_nexus_core_interfaces/
    ├── base/
    │   ├── scripts/
    │   │   ├── base.sh
    │   │   └── base.ps1
    │   ├── embodied_nexus_base/
    │   ├── embodied_nexus_base_interfaces/
    │   ├── third_party/licenses/
    │   ├── docs/
    │   └── examples/
    ├── arm/
    │   ├── scripts/
    │   │   ├── arm.sh
    │   │   └── arm.ps1
    │   ├── embodied_nexus_arm/
    │   ├── embodied_nexus_arm_interfaces/
    │   ├── third_party/licenses/
    │   ├── docs/
    │   └── examples/
    ├── camera/
    │   ├── scripts/
    │   │   ├── camera.sh
    │   │   └── camera.ps1
    │   ├── embodied_nexus_camera/
    │   ├── embodied_nexus_camera_interfaces/
    │   ├── docs/
    │   └── examples/
    ├── gripper/
    │   ├── scripts/
    │   │   ├── gripper.sh
    │   │   └── gripper.ps1
    │   ├── embodied_nexus_gripper/
    │   ├── embodied_nexus_gripper_interfaces/
    │   ├── third_party/licenses/
    │   ├── docs/
    │   └── examples/
    └── lidar/
        ├── scripts/
        │   ├── lidar.sh
        │   └── lidar.ps1
        ├── embodied_nexus_lidar/
        ├── embodied_nexus_lidar_interfaces/
        ├── third_party/lslidar_c16_ros2/
        │   ├── LICENSE
        │   ├── UPSTREAM_README.md
        │   ├── lslidar_driver/
        │   └── lslidar_msgs/
        ├── docs/
        └── examples/
    └── sim/
        ├── common/
        │   ├── embodied_nexus_sim/
        │   ├── docs/
        │   └── third_party/
        │       └── embodied_nexus_sim_description/
        └── gazebo/
            ├── scripts/
            │   ├── gazebo.sh
            │   └── gazebo.ps1
            └── embodied_nexus_sim_gazebo/
```

功能包使用 `ament_python`，自定义接口包使用 `ament_cmake` / `rosidl`，因此继续保留独立构建边界。厂商雷达包保持原包名和来源材料。colcon 的递归发现允许设备目录作为中间分组层，构建时用 `--base-paths src` 限定范围，详见 [colcon 官方发现参数](https://colcon.readthedocs.io/en/released/reference/discovery-arguments.html)。

`src/sim/common/` 保存仿真器无关的后端契约和共享模型，具体仿真器使用 `src/sim/<仿真器名>/` 独立接入。Gazebo Fortress 本机运行状态、型号适配重构验收与边界见[仿真实施记录](../src/sim/README.md)、[本轮验收报告](../src/sim/gazebo/embodied_nexus_sim_gazebo/docs/model-adapter-validation-2026-10-28.md)和[首期历史报告](../src/sim/gazebo/embodied_nexus_sim_gazebo/docs/acceptance-20261024.md)。

## 资料与依赖归属

| 内容                             | 归属                                               |
| -------------------------------- | -------------------------------------------------- |
| 设备实现、配置、launch、包测试   | `src/<设备能力>/embodied_nexus_<设备能力>/`      |
| 自定义 ROS 消息、服务、action    | 同模块的 `embodied_nexus_<设备能力>_interfaces/` |
| 设备指南、来源、实机记录         | `src/<设备能力>/docs/`                           |
| 设备专用示例                     | `src/<设备能力>/examples/`                       |
| Core 与设备的 Bash/PowerShell 操作脚本 | `src/<模块>/scripts/`                       |
| 随仓库保存的专用第三方代码与许可 | `src/<设备能力>/third_party/`                    |
| 通用远程 Python 客户端与测试     | `sdk/python/`                                    |
| 总体架构与跨模块说明             | 根目录 `docs/`                                   |
| 设备进程管理与统一网关           | `src/core/`                                      |
| 通用仿真后端与机器人模型         | `src/sim/common/`                                |
| 按仿真器拆分的场景、桥接和启动配置 | `src/sim/<仿真器名>/`                             |
| 已确认规划                       | 根目录 `todo/`                                   |

设备模块通过公开接口协作，不互相导入内部实现。SDK 依赖远程协议，不依赖 ROS 或机器人端源码。普通 Python、系统和 ROS 依赖继续通过包清单声明；只有需要随项目分发的专用依赖才存入模块的 `third_party/`。

设备入口：[底盘](../src/base/docs/scout-base.md)、[机械臂](../src/arm/docs/aubo-i5.md)、[相机](../src/camera/docs/camera.md)、[夹爪](../src/gripper/docs/ms42dc-gripper.md)、[雷达](../src/lidar/docs/lidar-c16.md)。开发约定统一维护在根目录 [AGENTS.md](../AGENTS.md)。

## 迁移说明

| 原位置或名称                                     | 新位置或名称                                  |
| ------------------------------------------------ | --------------------------------------------- |
| `src/embodied_nexus_<设备能力>*`               | `src/<设备能力>/embodied_nexus_<设备能力>*` |
| `src/third_party/lslidar_c16_ros2/`            | `src/lidar/third_party/lslidar_c16_ros2/`   |
| `third_party/licenses/`                        | `src/gripper/third_party/licenses/`         |
| 根目录 `docs/`、`examples/` 中的设备专用资料 | 所属模块的 `docs/`、`examples/`           |
| SDK 安装包 `embodied-nexus-camera-client`      | `embodied-nexus-client`                     |
| SDK 导入 `embodied_nexus_camera_client`        | `embodied_nexus_client`                     |
| SDK 相机子模块 `client.py`                     | `camera.py`                                 |
| `sdk/python/test_gripper_sdk.py`               | `sdk/python/tests/test_gripper_sdk.py`      |

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
PYTHONPATH=src/core/embodied_nexus_core python -m unittest discover -s src/core/embodied_nexus_core/test -v
PYTHONPATH=src/camera/embodied_nexus_camera python -m unittest discover -s src/camera/embodied_nexus_camera/test -v
PYTHONPATH=src/gripper/embodied_nexus_gripper python -m unittest discover -s src/gripper/embodied_nexus_gripper/test -v
PYTHONPATH=src/lidar/embodied_nexus_lidar python -m unittest discover -s src/lidar/embodied_nexus_lidar/test -v
PYTHONPATH=src/base/embodied_nexus_base python -m unittest discover -s src/base/embodied_nexus_base/test -v
PYTHONPATH=src/arm/embodied_nexus_arm python -m unittest discover -s src/arm/embodied_nexus_arm/test -v
```

PowerShell 对应写法如下；切换设备时同步替换 `PYTHONPATH` 和测试目录：

```powershell
$env:PYTHONPATH = "$PWD/src/camera/embodied_nexus_camera"
python -m unittest discover -s src/camera/embodied_nexus_camera/test -v
$env:PYTHONPATH = "$PWD/src/base/embodied_nexus_base"
python -m unittest discover -s src/base/embodied_nexus_base/test -v
$env:PYTHONPATH = "$PWD/src/core/embodied_nexus_core"
python -m unittest discover -s src/core/embodied_nexus_core/test -v
```

核心测试依赖见各包的 `setup.py` / `package.xml`；Web 测试还需要 `starlette`、`httpx`，core 网关回环测试需要 `uvicorn` 和 `websockets>=12`。SDK 数据流使用 `websockets>=12`。

Humble 环境中发现包并分别构建设备：

```bash
source /opt/ros/humble/setup.bash
colcon list --base-paths src
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_camera
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_gripper
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_lidar
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_base
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_arm
source install/setup.bash
python3 -m pip install -e sdk/python
python3 src/lidar/embodied_nexus_lidar/test/validate_lidar_modes.py
```

应发现 14 个 EmbodiedNexus ROS 包：core、底盘、机械臂、相机、夹爪和雷达各一个功能包与接口包，以及 `lslidar_driver`、`lslidar_msgs`。底盘与机械臂预设 mock，真实配置启动前需确认设备就绪；设备实机验收另按对应指南及[总体规划](../todo/设备中间层总体规划.md)执行。

## 统一 Core 管理节点

`src/core/` 是跨设备协调模块，只调用各设备现有的 launch、ROS 接口和 HTTP/WebSocket 入口，不导入设备内部实现。`embodied_nexus_core` 提供进程管理和状态汇总；`embodied_nexus_core_interfaces` 提供 ROS 管理服务。设备仍各自在独立进程中运行，core 启动后不会自动启动设备。

平台配置位于 `src/core/embodied_nexus_core/config/`。`platform_mock.yaml` 默认列出五类 mock 设备，`platform_real.yaml` 引用各设备模块已有实机配置。可选根级 `visualization` 配置控制 Web RViz 桥接、模型发布、模型关节状态来源及可读取资源包；旧配置缺省启用桥接但不启动模型发布器。实机配置启用 `embodied_nexus_sim_description` 的 Arachne Xacro 发布 `/robot_description`、`/tf_static` 和由机械臂关节状态驱动的 `/tf`，Scout 配置发布 `odom → base_link`。`config` 支持相对平台文件的普通路径和 `package://<ros-package>/<share-relative-path>`；设备列表的 ID 必须和该设备配置中的 `device_id` 或 `camera_id` 相同。每个 ID 全局唯一，同类设备可配置多个实例。平台校验检查文件引用和设备 ID；每个设备包在启动时继续校验自身参数。core 提供的 30 秒启动等待、5 秒停止反馈等待、1 秒轮询和 3 秒网关失联判定可在 `core` 段调整。

在 Humble 环境构建全部 core 依赖并启动：

```bash
source /opt/ros/humble/setup.bash
bash src/core/scripts/core.sh build core
source install/setup.bash
export EMBODIED_NEXUS_CORE_TOKEN='replace-with-a-private-url-safe-token'
ros2 run embodied_nexus_core nexus_core --config src/core/embodied_nexus_core/config/platform_mock.yaml
```

core 默认监听 `0.0.0.0:8780`。core token 通过配置指定的环境变量提供；设备网关只监听本机回环地址，使用各自的内部令牌。外部客户端通过 core 地址访问原有 `/api/v1/bases/<id>/...`、`/arms/`、`/grippers/`、`/cameras/` 和 `/lidars/` 接口；状态与数据 WebSocket 位于相应设备路径的 `/ws`。设备 API、租约和操作结果沿用所属设备模块定义。

ROS 管理服务位于 `/embodied_nexus/core/`，状态汇总发布在 `/embodied_nexus/core/status`。HTTP 管理路径为 `/api/v1/core/status`、`/devices`、`/devices/<id>`、`/devices/manage_all`、`/config`、`/config/validate` 和 `/shutdown`。设备管理 POST 的 action 可选 `start`、`shutdown`、`restart`；批量 action 可选 `start_all`、`shutdown_all`、`stop_all`。`stop_all` 只请求执行设备停止，不关闭它们或停止传感器采集；`/shutdown` 关闭受管设备后请求退出 core。

同源根路径提供 Core Web 运维面板，使用上述认证 API 展示状态、管理设备生命周期及查看只读平台配置。面板只在浏览器 `localStorage` 保存用户输入的 Core token；静态页面和资源不含 token。设备实时视频、配置编辑和直接运动控制不属于首版页面。

同源 `/rviz/` 提供 Web RViz：模型、TF、点云、LaserScan、RGB 和深度数据通过只读 topic subscription 显示。浏览器经 Core token 认证代理连接仅绑定 `127.0.0.1` 的 ROS bridge；Core 关闭时清理该进程，桥接不可用不会影响设备管理。前端构建需 Node.js 20+ / npm，ROS 依赖需安装 `ros-humble-rosbridge-server`；配置和话题说明见 [Web RViz 指南](../src/core/docs/web-rviz.md)。

```python
from embodied_nexus_client import CoreClient

core = CoreClient("http://robot-host:8780", token="your-core-token")
print(core.devices())
core.start("camera_front")
with core.base("base_main").acquire_control() as lease:
    lease.enable_motion()
    for status in core.base("base_main").subscribe_status():
        print(status)
        break
```

关闭底盘、机械臂或夹爪前，core 要求设备返回停稳确认；底盘或机械臂会继续读取状态等待确认，夹爪使用硬件 `hardware_stop_confirmed`。反馈未确认时操作失败并保留该进程。core 的正常退出同样会暂停，HTTP 管理入口继续可用；传感器和已确认停止的设备可正常关闭。Aubo 只请求停止，不自动断电。

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
ssh yahboom "cd ~/EmbodiedNexus && source /opt/ros/humble/setup.bash && colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_camera && colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_gripper && colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_lidar && colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_base && colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_arm"
```

## 迁移验证基线

2026-10-18 在迁移前的 Windows / Python 3.12 环境检查：相机 17 项、雷达核心 15 项、SDK 1 项通过；夹爪和雷达各 2 项 Web 测试因依赖缺失跳过。这些结果不替代 Humble 构建或实机验证，历史设备记录也不代表本次迁移后已重新验收。

夹爪包内 `backend.py`、`cli.py`、`controller.py`、`gripper_node.py`、`ros_gateway.py`、`web_server.py` 在迁移前已存在末尾字面量 `\n`，导致语法检查失败并阻止夹爪核心测试导入。本次目录与 SDK 命名迁移保留这些文件内容，作为既有问题单独记录。

当前 Windows 环境没有 `colcon`、`ros2`；迁移后的本地检查与尚待 Humble 环境执行的检查分别报告。
