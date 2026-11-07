# MuJoCo Planned 抓取验收

本指南用于 Ubuntu 22.04 / WSL2、ROS 2 Humble 和 RTX 3090 工作区。所有抓取轨迹都经过 Nexus 设备接口；不启动真实设备。

`grasp_demo.launch.py` 默认加载 `scene-1793463913737` 和 `grasp_cylinder.yaml`：目标是地面上的横放绿色圆柱 `cylinder-1`，长度 10 cm、直径 5 cm。它没有桌面，旧的桌面盒体验收必须显式指定下述盒体场景与配置。圆柱场景的观察位姿、识别和可达性仍需本机验收；启动不代表抓取成功。

## 当前本机运行环境

本次绿色圆柱复验的阶段结果和执行器参数见[2026-11-01 本机复验记录](mujoco-acceptance-20261101.md)。

当前验证环境为 Windows 工作区 `D:\code\project\EmbodiedNexus`，通过 WSL2 的 Ubuntu 22.04 运行 ROS 2 Humble 和 MuJoCo；仓库在 WSL 中对应 `/mnt/d/code/project/EmbodiedNexus`。当前已验证的构建安装 overlay 位于 `/home/dministrator/.cache/embodied-nexus/grasp-hardness-install`。旧的 `operator-install` 缺少 `embodied_nexus_sim.adapters`，不能用于本机启动。Python 为 3.10.12，GPU 为 NVIDIA RTX 3090，现有运行库为 PyTorch `2.5.1+cu121`、Ultralytics `8.3.253`，CUDA 可用。

视觉运行依赖已存在于本机 `/mnt/d/enx-local-cache/operator-runtime-py310`。ROS 启动的 operator 使用系统 Python，因此启动前必须把该目录加到 `PYTHONPATH`；漏掉这一步会报 `ModuleNotFoundError: ultralytics`，即使依赖本身仍在磁盘上。当前模型文件也已存在于 `/home/dministrator/.cache/embodied-nexus/models/`：`yoloe-11s-seg.pt`、`mobileclip_blt.ts` 和 `yoloe-green-cylinder-text.npz`。正常启动和运行抓取不需要联网、重新安装依赖或下载模型；设置 `YOLO_AUTOINSTALL=False` 可防止 Ultralytics 尝试自动安装。

本机启动 operator 前使用：

```bash
source /opt/ros/humble/setup.bash
source /home/dministrator/.cache/embodied-nexus/grasp-hardness-install/setup.bash
export PYTHONPATH=/mnt/d/enx-local-cache/operator-runtime-py310:${PYTHONPATH:-}
export YOLO_AUTOINSTALL=False
```

该 Python 依赖目录是当前主机的本地运行环境，不应在清理临时文件时删除。其他主机应填写其自己的依赖目录与安装 overlay。

在 WSL 中可从仓库根目录使用快捷脚本；启动只准备环境，不会自动触发机械臂抓取：

```bash
bash grasp_test.sh start          # 启动 core、MuJoCo、MoveIt 和 operator
bash grasp_test.sh restart        # 停止并重新启动脚本管理的进程
bash grasp_test.sh grasp          # 手动触发一次绿色圆柱抓取，打印 Action 反馈
bash grasp_test.sh status         # 查看脚本管理的进程
bash grasp_test.sh stop           # 停止脚本启动的进程
```

需要 MuJoCo 窗口时使用 `bash grasp_test.sh start --gui` 或 `bash grasp_test.sh restart --gui`。脚本优先使用本机已验证的 `grasp-hardness-install`，并在启动设备前检查仿真后端能否导入；启动时会打印选用的 overlay。Web RViz 为 `http://127.0.0.1:8780/rviz/`，启用“夹指碰撞盒与接触点”后可查看物理碰撞盒和当前接触点。脚本的 MoveIt 和 operator 日志保存在 `~/.local/state/embodied-nexus/grasp-test/`；ROS overlay、视觉运行库及 token 文件路径分别可通过 `ENX_GRASP_SETUP`、`ENX_GRASP_RUNTIME`、`ENX_GRASP_TOKEN_FILE` 覆盖。

圆柱提示使用本机 YOLOE 权重和 MobileCLIP 生成的 `yoloe-green-cylinder-text.npz` 与同名 manifest。确保现有模型文件齐全后，可从仓库根目录运行：

```bash
test -f ~/.cache/embodied-nexus/models/yoloe-11s-seg.pt
test -f ~/.cache/embodied-nexus/models/mobileclip_blt.ts
export YOLO_AUTOINSTALL=False
python3 src/operator/embodied_nexus_operator/scripts/prepare_yoloe.py \
  --embedding ~/.cache/embodied-nexus/models/yoloe-green-cylinder-text.npz \
  --class-prompt "green cylinder"
ros2 launch embodied_nexus_operator grasp_demo.launch.py gui:=true
```

圆柱抓取使用 ROS Action 的 `target_class: 'green cylinder'`、`object_id: cylinder-1`；HTTP `/plan` 的空 JSON 对象使用配置中的默认目标。operator 会在执行前将规划关节轨迹发布到 `/embodied_nexus/operators/grasp/planned_trajectory`，并将各阶段的点、时间、速度和加速度记录到 `~/.cache/embodied-nexus/trajectories/`。不完整的笛卡尔轨迹也会保存并发布，用于在 Web RViz 中查看规划终点；它不会因此被执行。

仿真启动时的机械臂姿态保存在场景 YAML 的 `initial_arm_joint_positions_rad`。operator 的 `observation_joints_rad` 是独立配置，触发抓取时机械臂会先规划移动到该观察姿态。

## 1. 安装依赖并构建

本节是新环境的首次准备流程。当前 WSL 已安装 Humble/MoveIt2、完成构建并缓存视觉依赖；普通启动或本机复验时跳过本节，不要重复执行安装和下载命令。

从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-moveit
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
python3 -m pip install --user torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cu121
python3 -m pip install --user -r src/operator/embodied_nexus_operator/requirements-vision.txt
colcon build --base-paths src --symlink-install
source install/setup.bash
python3 src/operator/embodied_nexus_operator/scripts/prepare_yoloe.py
```

模型准备脚本会下载本机运行所需资产，并写入权重/embedding 校验和清单。检查 `cuda_available` 为 `true`、设备为 RTX 3090；若不满足，先修复 WSL CUDA/PyTorch 环境再继续。权重未准备好时 operator 会在启动阶段报错，不会自动切换到物体真值或在线下载。

## 2. 启动 demo 并检查输入

```bash
export EMBODIED_NEXUS_OPERATOR_HTTP_TOKEN="$(openssl rand -hex 32)"
ros2 launch embodied_nexus_operator grasp_demo.launch.py gui:=true scene:=grasp_demo \
  operator_config:="$(ros2 pkg prefix --share embodied_nexus_operator)/config/grasp.yaml"
```

启动后确认以下资源存在且时间持续前进：

`grasp_demo` 的 `target_box` 是平放的 `8 × 4 × 3 cm` 红盒，长边方向由顶面点云 PCA 估计，夹爪应旋转 90° 后跨短边夹持。盒体尺寸同时配置在 `embodied_nexus_operator/config/grasp.yaml` 和 MuJoCo 场景中；更换物体尺寸时两处需保持一致。

绿色圆柱场景在 `finger_contact_boxes` 中为左右夹指各配置两个局部碰撞盒：根部盒和沿指尖网格倾斜方向旋转的末端盒。它们是对网格的近似，Web RViz 可同时显示实际盒体、可见网格和接触点；修改后需重启仿真才能生效。场景中显式配置的这些盒体优先于单盒尺寸参数。未配置多盒的场景仍可通过 `finger_contact_box_half_size_m` 调整单盒半尺寸；改用 URDF 网格碰撞时传入 `simplified_finger_collision:=false`。

圆柱抓取配置使用 `[0.10, 0.05, 0.05]` 米的长度和直径，俯视时 `grasp_frame` 目标高于估计圆柱中心 9 mm。接近路径要求完整到达目标，并保留地面碰撞检查；日志的 `grasp_geometry` 记录尺寸、估计中心及抓取和预抓取位置。下移后应在 Web RViz 检查左右夹指网格、碰撞盒和地面间距，再观察闭合接触与抬升结果。

绿色圆柱场景配置 `contact_cone: elliptic`，夹指 `finger_contact_material` 设置 `friction: [5.0, 0.02, 0.001]`、`condim: 4`、`solref: [0.008, 1.0]` 和 `solimp: [0.95, 0.99, 0.001, 0.5, 2.0]`。三个摩擦数值依次为滑动、扭转和滚动参数；`condim: 4` 启用滑动及扭转摩擦，滚动摩擦不启用。椭圆摩擦锥作用于场景所有摩擦接触；夹指接触优先使用上述材料参数，圆柱与地面的接触仍使用其自身材料参数。`solref` 和 `solimp` 调整接触约束的柔顺性，这些数值是待验收的仿真初值，不是橡胶实测数据。模型仍使用刚体碰撞盒，不会产生真实橡胶的几何变形和包覆；夹持效果还取决于碰撞形状、接触位置和夹紧力。修改参数后需重启仿真或重新应用场景。

```bash
ros2 topic hz /sim/arachne/joint_states
ros2 topic hz /embodied_nexus/cameras/camera_front/color/image_raw
ros2 topic hz /embodied_nexus/cameras/camera_front/depth/image_raw
ros2 topic echo --once /embodied_nexus/cameras/camera_front/color/camera_info
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
ros2 action list | grep -E 'operators/grasp|execute_trajectory|grippers/.*/move'
ros2 service list | grep -E 'apply_planning_scene|compute_cartesian_path|control/acquire'
```

检查 MuJoCo 场景为 `grasp_demo`，设备状态 backend 均为 `ros2_sim`，图像帧时间与深度帧时间差不超过配置窗口。单独执行一次检测预览：

```bash
ros2 run embodied_nexus_operator preview_yoloe \
  --camera-id camera_front --class-prompt "red rectangular box" \
  --output /tmp/yoloe-red-rectangular-box-preview.png
```

打开 `/tmp/yoloe-red-rectangular-box-preview.png`，确认掩码覆盖目标盒体而非桌面或夹爪。没有检测结果就先调整观察位姿、灯光或盒体外观；这一环节不允许读取 MuJoCo 物体 ID/真值替代识别结果。

## 3. RGB-D 位姿和 MoveIt 场景

触发完整抓取前，检查 camera info 焦距、彩色/深度帧、depth→color TF、深度到 `base_link` TF 均可用。视觉估计会在 action 结果中返回 `object_pose` 和 `grasp_pose`；核对坐标相对 MuJoCo 桌面、目标中心和夹爪闭合方向合理。operator 的 RGB-D 点云拟合只消费公开图像、相机内参及 TF。

MoveIt2 的 `move_group` 从 `/sim/arachne/joint_states` 获取实测关节状态；规划场景加入固定桌面、视觉估计目标和 operator 配置里的已知障碍。`grasp_demo` 初始没有额外障碍。故障试验可以复制一份 `grasp.yaml` 并在 `collision_obstacles` 中增加已知障碍，例如：

```yaml
collision_obstacles:
  - id: test_barrier
    center_base_m: [0.50, 0.0, 0.34]
    size_m: [0.16, 0.12, 0.25]
```

这个配置是规划先验，不从 MuJoCo 真值话题自动导入。

## 4. ROS Action 和 HTTP 入口

ROS 触发命令：

```bash
ros2 action send_goal /embodied_nexus/operators/grasp \
  embodied_nexus_operator_interfaces/action/PlanGrasp \
  "{target_class: 'red rectangular box', object_id: target_box, camera_id: camera_front, arm_id: arm_main, gripper_id: gripper_main}" \
  --feedback
```

HTTP 触发通过 `POST http://127.0.0.1:8782/plan`；返回 `task_id` 后用 `GET /tasks/{task_id}` 查询，或 `DELETE /tasks/{task_id}` 取消。具体认证头和 JSON 见[抓取设计](grasp-design.md#ros-和-http-触发)。两种入口都映射同一个 ROS Action，重复并发目标应被拒绝。

检查反馈依次覆盖观察、检测、定位、规划、移动/接近、闭合和抬升。MoveIt 只产出规划轨迹，轨迹应通过 `/embodied_nexus/arms/arm_main/execute_trajectory`，并持有有效 Nexus 控制租约。夹爪需要报告目标物体接触；停住但无接触时不能视为闭合成功。动作结果的 `grasp_verified` 在独立真值验收前应为 `false`。

## 5. 五次独立真值验收

使用验收器重载 `grasp_demo`、等待目标稳定、触发 Action 并独立记录 MuJoCo 真值：

```bash
ros2 run embodied_nexus_operator grasp_acceptance \
  --trials 5 --object-id target_box --camera-id camera_front \
  --arm-id arm_main --gripper-id gripper_main \
  --target-class "red rectangular box"
```

每次输出包含按阶段记录的反馈、目标估计位置误差、动作结果和抬升指标。验收器订阅 `/sim/arachne/ground_truth/scene_objects`，只有验收器读取物体真值和夹爪接触；它要求盒体相对本次起始位置抬高至少 **5 cm**、连续保持 **2 秒**，期间夹爪仍接触目标且目标相对 `grasp_frame` 的平移偏差不超过 **2 cm**。5 次都满足才返回进程退出码 0。

记录并归档：YOLOE 检测置信度/掩码、视觉中心与 MuJoCo 真值的位置误差、MoveIt 预抓取/笛卡尔路径结果、Nexus 轨迹执行结果、接触闭合结果、抬升距离和保持时间。真值位置仅在验收器中用于评分，不传回 operator。

## 6. 故障覆盖

逐项执行并记录 action/HTTP 的终态、停止反馈和 `grasp_verified`：

| 场景 | 操作方法 | 预期结果 |
| --- | --- | --- |
| 目标不存在 | 暂时遮住盒体或请求不匹配的类别 | 检测失败并停止；不进入夹爪闭合，不报告成功 |
| 目标不可达 | 使用只用于验收的场景副本，把目标放到机械臂工作空间外 | MoveIt 规划失败并停止 |
| 障碍碰撞 | 用配置加入挡住路径的已知障碍 | MoveIt 拒绝路径或笛卡尔段不完整，机械臂不继续接近 |
| RGB-D 缺失 | 停止对应 camera ROS 节点 | 等待超时并失败，不使用缓存旧帧 |
| TF 缺失 | 在隔离试验中停止发布所需相机 TF | 定位失败并停止 |
| 取消 | 接受任务后调用 HTTP DELETE，或 ROS Action cancel | 子动作取消，两个设备收到 stop，结果为取消且验证为 false |
| 租约过期 | 验收时阻断续租服务或注入服务拒绝 | operator 中止并请求停止，不继续执行轨迹 |
| 仿真暂停 | 暂停 MuJoCo stepping | 收不到新鲜推进状态时超时或失败；不把冻结状态当作完成 |
| 仿真时钟回退 | 通过测试场景重新编译/重载仿真并观察设备后端时钟保护 | 后端标记反馈失效，抓取任务失败并停止 |

每次故障场景后恢复 demo、确认设备可重新获取租约，再继续下一项。不要在带真实设备实例的 launch 上运行此验收。

当前工作区的构建、测试和 MuJoCo 试跑结果见[本地验收记录](mujoco-acceptance-20261031.md)。五次连续抓取和故障矩阵仍需按本指南执行。

## 后续验收记录模板

| 项目 | 结果 | 说明 |
| --- | --- | --- |
| 本地静态检查 | 通过 | Python AST、XML/YAML 解析、ROS 包发现、模块导入及两个 launch 的 `--show-args` 均通过；未运行测试套件 |
| Humble / MoveIt 构建 | 通过 | WSL Ubuntu 22.04 / ROS 2 Humble，`scripts/build.sh embodied_nexus_operator --serial`，21 个包成功；有两个既有依赖包的非致命 CMake 警告 |
| YOLOE 检测预览 | 未执行 | 保存图像路径、置信度与掩码质量 |
| 5 次 MuJoCo 抓取 | 未执行 | 每次填位姿误差、规划、执行和抬升数据 |
| 故障覆盖 | 未执行 | 逐项填写停止/失败行为 |

## 绿色圆柱柔性夹指垫

`scene-1793463913737.yaml` 的 `finger_flex_pads.style` 可选 `fin_ray` 或 `pad`。当前使用 `fin_ray`：远端刚性碰撞盒停止参与接触，替换为两侧带七条肋的柔性二维网格；近端刚性碰撞保护保留，网格根部固定到原夹指，柔性节点的面外运动使用弹簧阻尼近似约束。接触摩擦沿用场景中滑动系数 5，积分步长为 0.5 ms。当前加硬参数为 Fin Ray 边约束时间常数 0.5 ms、面外弹簧刚度 500 N/m；原值分别为 1 ms 和 100 N/m。MuJoCo 和 Web RViz 显示柔性网格时隐藏原刚性指网格，以便观察变形。切换为 `pad` 可使用三维柔性垫和刚性背板。参数尚未按实物标定；变形日志为网格边长相对初始值的最大变化，并非材料的最大位移或应变。

`bash grasp_test.sh start --gui` 启动时为 MoveIt 加入原柔性垫的固定碰撞近似。使用其他方式单独启动 MoveIt 时，绿色圆柱场景需向 `move_group.launch.py` 传入 `soft_finger_pads:=true`；其他场景默认关闭。该近似未按 Fin Ray 全部肋条重新生成，也不能反映实时变形；不能以此断言 Fin Ray 指体和地面之间的路径已被完整验证。

Web RViz 中同时打开“柔性夹指垫”和“夹指碰撞盒与接触点”，可对照柔性网格、刚性夹指及接触点。MuJoCo 日志 `MuJoCo flexible finger contact` 包含各指接触位置、法向力、边长变化及双指接触对象；日志按仿真时间限频，供闭合和抬升排查。接触反馈仍通过 Nexus 夹爪接口传递。

由用户手动执行一次抓取后，记录两侧接触、网格变形、穿透和物体状态，独立确认物体抬升至少 5 cm 并保持 2 秒。上述仿真参数、网格和日志本身不代表抓取成功；MuJoCo 物体真值不输入视觉定位或规划。
