# MuJoCo 仿真启动指南

MuJoCo 配置接入项目的标准 ROS 2 设备后端和 Core 生命周期。它加载共享 Arachne URDF，发布仿真时钟、底盘里程计、关节状态、相机 RGB-D 和雷达点云；设备 API、SDK 和 Core 配置沿用 Gazebo 的 `ros2_sim` / `ros2_topics` 接口。

## 环境和构建

运行环境为 Ubuntu 22.04 / ROS 2 Humble。MuJoCo Python 包固定为 3.2.7；若当前 WSL Python 环境尚未安装：

```bash
python3 -m pip install --user "mujoco==3.2.7"
```

从仓库根目录构建仿真依赖和两个仿真器入口：

```bash
bash scripts/build.sh sim
```

Windows PowerShell 下使用 WSL 包装脚本：

```powershell
.\scripts\build.ps1 sim
```

在 D 盘挂载目录中，构建脚本会将产物放在 WSL 的用户缓存目录，并显示应 source 的 `setup.bash`。`scripts/services.sh` 会自动发现此安装目录。

## 启动五类设备

Core 管理模式会启动 MuJoCo、Core，并通过设备服务启动平台配置里的底盘、机械臂、夹爪、相机和雷达。先复制 `src/core/embodied_nexus_core/config/core_tokens.example.yaml` 为同目录的 `core_tokens.local.yaml`，填写 `sim.mujoco` 令牌。环境变量 `EMBODIED_NEXUS_CORE_TOKEN` 可覆盖文件中的值。Windows PowerShell 示例：

```powershell
.\scripts\services.ps1 start --mode sim --simulator mujoco -all
```

默认无窗口运行，并使用 EGL 渲染传感器图像。使用 `--gui` 打开 MuJoCo viewer：

```powershell
.\scripts\services.ps1 start --mode sim --simulator mujoco --gui -all
```

Linux / WSL Bash 启动与查看状态：

```bash
bash scripts/services.sh start --mode sim --simulator mujoco -all
bash scripts/services.sh status --mode sim --simulator mujoco
```

停止所有托管设备和仿真器：

```bash
bash scripts/services.sh stop --mode sim --simulator mujoco -all
```

Windows 下对应命令为 `.\scripts\services.ps1 stop --mode sim --simulator mujoco -all`。Core 默认地址为 `http://127.0.0.1:8780`；启动日志和进程记录位于 `${XDG_STATE_HOME:-~/.local/state}/embodied-nexus/`。

## 场景配置与图形编辑

Core 启动后，在 `http://127.0.0.1:8780/` 使用相同 Core token 登录，再打开“MuJoCo 场景编辑器”。编辑器入口仅在 MuJoCo 平台配置中启用。场景以 YAML 保存到 `src/sim/mujoco/embodied_nexus_sim_mujoco/config/scenes/`；上传的 STL/OBJ 文件保存在对应的 `assets/<scene-id>/` 子目录，可与 YAML 一起提交到 Git。

默认场景由 `config/scenes/default.yaml` 管理，机器人仍从共享 Arachne URDF 加载。场景物体可使用 `box`、`sphere`、`cylinder` 或导入网格，并配置位置、姿态、颜色和静态/动态属性。盒子的 `size` 是完整 X/Y/Z 尺寸；球和圆柱使用 `radius`，圆柱另有完整 `height`。动态物体须给出正数 `mass_kg`。配置中的姿态 `rotation` 使用弧度；编辑器属性面板以度显示并在保存时换算。动态导入网格使用 MuJoCo 网格碰撞近似。

从仓库根目录按场景启动：

```powershell
.\scripts\services.ps1 start --mode sim --simulator mujoco --scene default -all
```

`--scene` 接受 YAML 文件名对应的场景 ID。编辑器预览不会修改运行场景。点击“应用场景”会校验并保存当前草稿，再请求运行中的 MuJoCo 节点重新编译并切换模型；MuJoCo 与 ROS 节点进程、仿真时间和机器人位姿保持不变，场景动态物体回到 YAML 中的初始位姿，进行中的底盘、机械臂和夹爪运动命令会清除。Viewer 绑定创建时的模型，热更新期间窗口会短暂关闭并重新打开。校验失败不会覆盖 YAML；若运行时服务不可达或应用失败，YAML 已保存，原运行模型继续工作。不使用图形编辑器时，也可以停止并用新场景重新启动：

```powershell
.\scripts\services.ps1 stop --mode sim --simulator mujoco -all
.\scripts\services.ps1 start --mode sim --simulator mujoco --scene lab-a --gui -all
```

直接使用 ROS launch 时可指定 `scene:=lab-a`；如需让 Core 编辑器保存回仓库而不是安装目录，传入 `scene_directory:=<仓库>/src/sim/mujoco/embodied_nexus_sim_mujoco/config/scenes`。服务脚本会自动将该仓库目录同时传给 MuJoCo 和 Core。

Core 的认证 REST API 位于 `/api/v1/core/scenes`：`GET` 列表，`GET /{id}` 读取，`POST /{id}/validate` 校验，`PUT /{id}` 校验后原子保存，`POST /{id}/apply` 校验、保存并热应用当前草稿，`DELETE /{id}` 删除；`PUT/GET /{id}/assets/{filename}` 上传或读取场景内 STL/OBJ。所有接口使用 Core `Authorization: Bearer <token>`。Core 通过 `embodied_nexus_sim_mujoco_interfaces/srv/ApplyScene` 将应用请求转发给 MuJoCo 节点；当前机器人碰撞几何仍关闭接触响应，本期场景动态物体只与地面及其他场景物体交互。

## 模型范围

- 底盘按命令速度更新里程计和模型根位姿；轮子关节同步响应，但底盘运动为运动学更新。
- 机械臂和夹爪通过 MuJoCo 位置执行器运行，并启用重力补偿；Control 滑块使用 URDF 关节限位作为目标范围。转动关节的角度单位为 rad，移动关节使用 m；接触碰撞响应关闭，以保证共享 URDF 简化模型的关节状态稳定。输入命令仍由各自设备服务执行校验、租约与停止逻辑。
- RGB-D 相机与 Gazebo 共用 `embodied_nexus_sim_description/config/gemini335_extrinsics.json` 中的实测安装外参，以及 `embodied_nexus_camera/config/gemini335_factory_calibration.json` 中的 RGB-D 内参和深度到彩色相机变换。MuJoCo 分别渲染彩色和深度视角，并用相同数据发布 CameraInfo 与机器人 TF；仿真图像为理想针孔图像，畸变设为零，与 Gazebo 仿真配置一致。
- 雷达使用水平 360 度 MuJoCo 射线检测发布点云。
- URDF 转换使用碰撞网格和基本形状，没有转换 DAE 可视网格。渲染依赖 OpenGL；若运行环境没有可用的 MuJoCo 渲染上下文，ROS 相机话题会发出占位帧并在日志中报告初始化错误。
- 此仿真用于算法与接口联调，不替代 Gazebo 场景验收或实机运动验收。
