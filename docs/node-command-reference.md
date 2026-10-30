# Core 与设备节点命令参考

## 根目录统一服务脚本

`scripts/services.sh` 在 Linux / WSL 使用；Windows 使用 `scripts/services.ps1`，参数相同。必须显式选择 `--mode mock|real|sim`；`sim` 需要 `--simulator gazebo|mujoco`。默认通过 Core 管理，`--standalone` 直接启动独立 ROS 设备。`-all` 选择配置中的所有启用设备；设备类型名有多个实例时必须使用实例 ID。

Core 管理模式按环境读取 `src/core/embodied_nexus_core/config/core_tokens.local.yaml`。将同目录的 `core_tokens.example.yaml` 复制为该文件，分别填写 `mock`、`real`、`sim.gazebo` 和 `sim.mujoco`；未使用的环境可留空。令牌仅允许 URL-safe 字符，文件已加入 Git 忽略。设置 `EMBODIED_NEXUS_CORE_TOKEN` 时，环境变量优先于文件。启动、状态和停止命令须使用与运行中 Core 相同的令牌；更换令牌需要先用旧令牌停止，再使用新令牌启动。

此文件由根目录统一服务脚本读取。直接运行 `ros2 run`、`ros2 launch` 或模块专用脚本时，仍按对应入口设置环境变量或 launch 参数。

```bash
export EMBODIED_NEXUS_SETUP=/path/to/install/setup.bash # 默认仓库 install/setup.bash
bash scripts/services.sh start --mode mock base_main camera_front
bash scripts/services.sh status --mode mock
bash scripts/services.sh stop --mode mock -all
bash scripts/services.sh start --mode sim --simulator gazebo -all
bash scripts/services.sh stop --mode sim --simulator gazebo -all
bash scripts/services.sh start --mode sim --simulator mujoco -all
bash scripts/services.sh stop --mode sim --simulator mujoco -all
bash scripts/services.sh start --mode mock --standalone base camera
bash scripts/services.sh stop --mode mock --standalone base camera
```

Windows 示例：`$env:EMBODIED_NEXUS_SETUP='/root/embodied-nexus-run/install/setup.bash'`，设置同一个令牌后执行 `.\scripts\services.ps1 start --mode mock -all`。`--setup` 也可直接指定 Linux 路径；`EMBODIED_NEXUS_WSL_DISTRO` 选择 WSL 发行版。软件渲染可在启动前设置 `LIBGL_ALWAYS_SOFTWARE=1` 和 `GALLIUM_DRIVER=llvmpipe`。Gazebo 默认无界面，`--gui` 打开窗口。

启动和关闭为后台管理，日志及非敏感进程记录位于 `${XDG_STATE_HOME:-~/.local/state}/embodied-nexus/`。`stop -all` 会先请求设备安全关闭，然后关闭 Core 和脚本创建的仿真进程；停稳反馈无法确认时保留设备和管理入口。单设备关闭保留 Core 和仿真器。实机启动使用 `--mode real --confirm-ready`，并在执行前确认设备就绪；`--config` 可指定 mock 或 real 平台 YAML。独立仿真设备要求仿真器已就绪，可先运行 `ros2 launch embodied_nexus_sim_gazebo sim.launch.py gui:=false start_core:=false` 或 `ros2 launch embodied_nexus_sim_mujoco sim.launch.py gui:=false start_core:=false`。不得由独立入口和 Core 同时管理同一设备。

本文整理 Core、底盘、机械臂、夹爪、相机、雷达和 Gazebo/MuJoCo 仿真的 Mock/仿真启动与生命周期命令。MuJoCo 的安装和模型限制见[仿真指南](../src/sim/mujoco/README.md)；真实硬件配置和前置检查继续以各设备指南为准：[底盘](../src/base/docs/scout-base.md)、[机械臂](../src/arm/docs/aubo-i5.md)、[夹爪](../src/gripper/docs/ms42dc-gripper.md)、[相机](../src/camera/docs/camera.md)、[雷达](../src/lidar/docs/lidar-c16.md)。

## 通用环境

在 Ubuntu 22.04 / WSL2 Ubuntu-22.04 中构建并 source 工作区。以下相对配置路径均从仓库根目录执行：

```bash
cd ~/EmbodiedNexus
bash scripts/build.sh all
```

构建脚本会打印工作区的 `setup.bash` 路径。仓库位于 WSL `/mnt/<盘符>/...` 时，build/install/log 默认放在 Linux 用户缓存目录，以支持 colcon symlink install。

WSL 仓库若位于 Windows D 盘，路径通常类似 `/mnt/d/code/project/EmbodiedNexus`；先 `cd` 到实际仓库目录，再执行相同的 Bash 命令。

Windows PowerShell 原生不运行本项目的 ROS 2 节点。全仓构建使用下方根目录包装脚本；执行其他自定义 Bash 命令时，可从仓库根目录调用下面的 WSL 包装函数：

```powershell
function Invoke-EmbodiedNexusWslCommand {
    param([Parameter(Mandatory = $true)][string]$RosCommand)

    $embodiedNexusRepoWin = (Get-Location).Path -replace '\\', '/'
    if (-not (Test-Path (Join-Path $embodiedNexusRepoWin "src"))) {
        throw "请先切换到 EmbodiedNexus 仓库根目录。"
    }
    $embodiedNexusWslDistro = "Ubuntu-22.04"
    $embodiedNexusRepoWsl = (& wsl.exe -d $embodiedNexusWslDistro -- wslpath -u -a $embodiedNexusRepoWin).Trim()
    if ($LASTEXITCODE -ne 0 -or -not $embodiedNexusRepoWsl) {
        throw "无法将当前仓库路径转换为 WSL 路径。"
    }
    $embodiedNexusBashCommand = "cd '$embodiedNexusRepoWsl' && source /opt/ros/humble/setup.bash && if [ -f install/setup.bash ]; then source install/setup.bash; fi && $RosCommand"
    $embodiedNexusCommandBase64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($embodiedNexusBashCommand))
    $embodiedNexusLauncher = "echo $embodiedNexusCommandBase64 | base64 -d | bash"
    wsl.exe -d $embodiedNexusWslDistro -- bash -lc $embodiedNexusLauncher
}
```

根目录构建包装脚本可从 PowerShell 调用 WSL，目标支持 `all`、`core`、`sim` 或 ROS 包名：

```powershell
.\scripts\build.ps1 all
.\scripts\build.ps1 sim --serial
```

## 模块脚本入口

各模块的 Bash 脚本会按自身位置定位仓库根目录；PowerShell 脚本会定位同目录的 Bash 脚本，并通过 `wsl.exe` 调用 `Ubuntu-22.04`。可设置 Windows 环境变量 `EMBODIED_NEXUS_WSL_DISTRO` 更换发行版。PowerShell 脚本会临时通过 `WSLENV` 转发 Core token、`IGN_IP` 和 `ROS_DOMAIN_ID`，调用结束后恢复原值。

Core 脚本提供构建、Mock 启动、状态查询、单设备生命周期及批量管理：

```bash
bash src/core/scripts/core.sh build core
export EMBODIED_NEXUS_CORE_TOKEN=replace-with-a-local-url-safe-token
bash src/core/scripts/core.sh start-mock
bash src/core/scripts/core.sh status
bash src/core/scripts/core.sh device start base_main
bash src/core/scripts/core.sh device restart base_main
bash src/core/scripts/core.sh device shutdown base_main
bash src/core/scripts/core.sh batch stop_all
bash src/core/scripts/core.sh batch shutdown_all
```

PowerShell 从仓库根目录调用同一套命令；Core token 会由脚本转发进 WSL：

```powershell
.\src\core\scripts\core.ps1 build core
$env:EMBODIED_NEXUS_CORE_TOKEN = 'replace-with-a-local-url-safe-token'
.\src\core\scripts\core.ps1 start-mock
.\src\core\scripts\core.ps1 status
.\src\core\scripts\core.ps1 device start base_main
.\src\core\scripts\core.ps1 batch stop_all
.\src\core\scripts\core.ps1 batch shutdown_all
```

五类设备脚本的 `mock` 和 `sim` 参数对应本指南中的显式配置；`sim` 前先启动 Gazebo。独立 launch 不能与 Core 同时管理相同设备 ID：

```bash
bash src/base/scripts/base.sh mock
bash src/base/scripts/base.sh sim
bash src/arm/scripts/arm.sh mock
bash src/arm/scripts/arm.sh sim
bash src/gripper/scripts/gripper.sh mock
bash src/gripper/scripts/gripper.sh sim
bash src/camera/scripts/camera.sh mock
bash src/camera/scripts/camera.sh sim
bash src/lidar/scripts/lidar.sh mock
bash src/lidar/scripts/lidar.sh sim
```

PowerShell 设备入口如下：

```powershell
& .\src\base\scripts\base.ps1 mock
& .\src\base\scripts\base.ps1 sim
& .\src\arm\scripts\arm.ps1 mock
& .\src\arm\scripts\arm.ps1 sim
& .\src\gripper\scripts\gripper.ps1 mock
& .\src\gripper\scripts\gripper.ps1 sim
& .\src\camera\scripts\camera.ps1 mock
& .\src\camera\scripts\camera.ps1 sim
& .\src\lidar\scripts\lidar.ps1 mock
& .\src\lidar\scripts\lidar.ps1 sim
```

Gazebo 脚本可以构建并以前台 GUI 或无界面模式启动；WSL2 GUI 模式会设置 WSLg 环境及 `IGN_IP`：

```bash
bash src/sim/gazebo/scripts/gazebo.sh build
bash src/sim/gazebo/scripts/gazebo.sh start
bash src/sim/gazebo/scripts/gazebo.sh headless
```

PowerShell 对应入口：

```powershell
& .\src\sim\gazebo\scripts\gazebo.ps1 build
& .\src\sim\gazebo\scripts\gazebo.ps1 start
& .\src\sim\gazebo\scripts\gazebo.ps1 headless
```

前台运行按 `Ctrl+C` 退出；`Ctrl+Z` 会挂起进程。真实硬件操作仍按本指南开头链接的设备指南执行。

## Core Mock 启动与管理

Core Mock 配置列出五类 Mock 设备，启动 Core 后这些设备服务仍保持停止。令牌需是 URL-safe 字符；示例值只用于本地 Mock：

```bash
export EMBODIED_NEXUS_CORE_TOKEN=replace-with-a-local-url-safe-token
ros2 run embodied_nexus_core nexus_core --config src/core/embodied_nexus_core/config/platform_mock.yaml
```

PowerShell / WSL：

```powershell
Invoke-EmbodiedNexusWslCommand -RosCommand 'export EMBODIED_NEXUS_CORE_TOKEN=replace-with-a-local-url-safe-token && ros2 run embodied_nexus_core nexus_core --config src/core/embodied_nexus_core/config/platform_mock.yaml'
```

Core 启动后，在另一个已 source 工作区的 Bash 终端中可以查询状态、管理单设备或批量管理。单设备的 `device_id` 可替换为 `base_main`、`arm_main`、`gripper_main`、`camera_front`、`lidar_main`：

```bash
ros2 service call /embodied_nexus/core/get_status embodied_nexus_core_interfaces/srv/GetCoreStatus {}
ros2 service call /embodied_nexus/core/manage_device embodied_nexus_core_interfaces/srv/ManageCoreDevice "{device_id: base_main, action: start}"
ros2 service call /embodied_nexus/core/manage_device embodied_nexus_core_interfaces/srv/ManageCoreDevice "{device_id: base_main, action: restart}"
ros2 service call /embodied_nexus/core/manage_device embodied_nexus_core_interfaces/srv/ManageCoreDevice "{device_id: base_main, action: shutdown}"
ros2 service call /embodied_nexus/core/manage_all embodied_nexus_core_interfaces/srv/ManageAllCoreDevices "{action: start_all}"
ros2 service call /embodied_nexus/core/manage_all embodied_nexus_core_interfaces/srv/ManageAllCoreDevices "{action: stop_all}"
ros2 service call /embodied_nexus/core/manage_all embodied_nexus_core_interfaces/srv/ManageAllCoreDevices "{action: shutdown_all}"
ros2 service call /embodied_nexus/core/manage_all embodied_nexus_core_interfaces/srv/ManageAllCoreDevices "{action: shutdown_core}"
```

PowerShell / WSL 的单设备示例：

```powershell
Invoke-EmbodiedNexusWslCommand -RosCommand 'ros2 service call /embodied_nexus/core/manage_device embodied_nexus_core_interfaces/srv/ManageCoreDevice "{device_id: base_main, action: start}"'
```

`stop_all` 只发送安全停止请求，设备进程和传感器服务保持运行；`shutdown_all` 逐个请求停稳并关闭服务；`shutdown_core` 在受管设备安全关闭后退出 Core。管理命令作用于当前 Core 加载的平台配置。只有确认 Core 使用 Mock/仿真配置时，才复制启动类命令。

## 独立设备节点

以下命令用于**不由 Core 托管**的独立 ROS launch。所有示例显式选用 Mock 或仿真配置，并设置 `interface_mode:=ros2`，不会另外打开设备 Web 网关。仿真配置需在 Gazebo 已经运行后启动。

| 节点 | Mock Bash 命令 | 仿真 Bash 命令 |
| --- | --- | --- |
| 底盘 | `ros2 launch embodied_nexus_base base.launch.py config_file:=src/base/embodied_nexus_base/config/scout_mock.yaml interface_mode:=ros2 use_sim_time:=false` | `ros2 launch embodied_nexus_base base.launch.py config_file:=src/base/embodied_nexus_base/config/scout_sim.yaml interface_mode:=ros2 use_sim_time:=true` |
| 机械臂 | `ros2 launch embodied_nexus_arm arm.launch.py config_file:=src/arm/embodied_nexus_arm/config/aubo_i5_mock.yaml interface_mode:=ros2 use_sim_time:=false` | `ros2 launch embodied_nexus_arm arm.launch.py config_file:=src/arm/embodied_nexus_arm/config/aubo_i5_sim.yaml interface_mode:=ros2 use_sim_time:=true` |
| 夹爪 | `ros2 launch embodied_nexus_gripper gripper.launch.py config_file:=src/gripper/embodied_nexus_gripper/config/ms42dc_mock.yaml interface_mode:=ros2 use_sim_time:=false` | `ros2 launch embodied_nexus_gripper gripper.launch.py config_file:=src/gripper/embodied_nexus_gripper/config/ms42dc_sim.yaml interface_mode:=ros2 use_sim_time:=true` |
| 相机 | `ros2 launch embodied_nexus_camera camera.launch.py config_file:=src/camera/embodied_nexus_camera/config/gemini335_mock.yaml interface_mode:=ros2 use_sim_time:=false` | `ros2 launch embodied_nexus_camera camera.launch.py config_file:=src/camera/embodied_nexus_camera/config/gemini335_sim.yaml interface_mode:=ros2 use_sim_time:=true` |
| 雷达 | `ros2 launch embodied_nexus_lidar lidar.launch.py config_file:=src/lidar/embodied_nexus_lidar/config/c16_mock.yaml interface_mode:=ros2 use_sim_time:=false` | `ros2 launch embodied_nexus_lidar lidar.launch.py config_file:=src/lidar/embodied_nexus_lidar/config/c16_sim.yaml interface_mode:=ros2 use_sim_time:=true` |

PowerShell / WSL（底盘 Mock 示例）：

```powershell
Invoke-EmbodiedNexusWslCommand -RosCommand 'ros2 launch embodied_nexus_base base.launch.py config_file:=src/base/embodied_nexus_base/config/scout_mock.yaml interface_mode:=ros2 use_sim_time:=false'
```

独立 launch 与 Core 托管的相同 `device_id` 不可同时运行，以免重复进程或端口冲突。独立前台 launch 按 `Ctrl+C` 正常退出。不要按 `Ctrl+Z` 代替退出：它会挂起进程并继续占用资源。

## Gazebo Fortress 仿真与 Core

WSL2 需要将 `IGN_IP` 设置为 Windows 可访问的 WSL IPv4；可用 `ip -4 -br addr` 查看网络接口。下面命令为 WSLg GUI 模式，`sim.launch.py` 会启动 Gazebo、控制器和 Core，Core 中的设备服务仍保持停止：

```bash
export ROS_LOCALHOST_ONLY=1
export IGN_IP=REPLACE_WITH_WSL_IPV4_REACHABLE_FROM_WINDOWS
export EMBODIED_NEXUS_CORE_TOKEN=replace-with-a-local-url-safe-token
export DISPLAY=:0
export XDG_RUNTIME_DIR=/tmp/embodied-nexus-gui-runtime
mkdir -p /tmp/embodied-nexus-gui-runtime
chmod 700 /tmp/embodied-nexus-gui-runtime
export QT_QPA_PLATFORM=xcb
unset WAYLAND_DISPLAY
ros2 launch embodied_nexus_sim_gazebo sim.launch.py gui:=true core_token:=$EMBODIED_NEXUS_CORE_TOKEN
```

无图形服务器将 `gui:=true` 改为 `gui:=false`，并省略 `DISPLAY`、`XDG_RUNTIME_DIR`、`QT_QPA_PLATFORM` 和 `WAYLAND_DISPLAY` 设置。退出整组仿真和 Core 时，在启动 launch 的前台终端按 `Ctrl+C`。

Core Web 运维页面顶部的“命令参考”及设备卡片的“命令”按钮提供相同的 Mock/仿真命令，可切换 Bash/WSL 和 PowerShell 格式并复制。页面只展示、复制命令，不会执行系统命令。真实硬件命令不在本表中；启动前检查和型号专属选项请使用对应设备指南。
