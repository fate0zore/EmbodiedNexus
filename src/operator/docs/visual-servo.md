# 独立视觉伺服模块（MuJoCo）

`embodied_nexus_visual_servo` 独立于已有抓取 operator，通过 Nexus 设备公开 ROS 接口执行动作。YOLOE 使用本地权重和预先准备的文本 embedding 分割目标，RGB-D 与 TF 提供位置估计；MoveIt 规划观察、预抓取和抬升，MoveIt Servo 生成末端接近速度，Nexus 负责租约与关节执行。规划和感知不读取仿真物体真值；单独的验收器读取物体真值判断是否抬起。

当前仅支持 MuJoCo，禁止用于实机。两次圆柱抓取已通过独立验收，但尚未完成圆柱和盒体各五次验收，也未完成全部故障注入测试。

新增实验场景 `visual_servo_convex` 使用原 STL 外观和离线分解的刚性凸碰撞网格，MoveIt 与 MuJoCo 使用同一网格和变换。历史的 `visual_servo_contact` 与此前验收记录保留，不能用旧场景的结果证明新几何通过抓取验收。凸网格由 `src/sim/mujoco/examples/decompose_ms42dc_fingers.py` 生成，版本、参数及输入 SHA-256 在网格目录的 `manifest.json` 中；生成需要额外安装 `coacd`、`trimesh`，运行时不依赖这些包。`validate_ms42dc_fingers.py` 需要 `trimesh` 和 `rtree`，可对分解表面的确定性采样检查与原网格的距离；采样不是完整表面误差证明。

早期诊断中，原始圆柱曾在接近阶段丢失视觉目标或超时；坐标系和速度跟踪修正后，新凸分解几何已完成单次独立抬升验收，五次重启验收仍须以实际记录为准。入口默认场景保留旧碰撞盒，旧验收结果不代表当前版本通过。

视觉追踪使用连续五个完整观测初始化，随后按滑动窗口更新目标；检测失败会在后续帧按角度轮换重试，仍受帧新鲜度限制。闭合前要求位置误差不超过 3 mm、姿态误差不超过 3 度，连续稳定 0.5 秒后停稳复查。仿真接触要求左右指各自对目标的法向力达到 0.05 N；该力阈值和仿真摩擦参数尚未进行实物标定。可设置 `ENX_VISUAL_SERVO_DEBUG_DIR` 保存 RGB、检测掩膜和 JSONL 检测诊断。

MuJoCo 视觉接近使用 MoveIt Servo 的逆雅可比速度求解，规划仍使用 MoveGroup 的 KDL。Servo 规划坐标系与机械臂运动组的 `aubo_base_link` 一致，接收的 `base_link` 速度命令由 Servo 转换；该安装关系的两个坐标系相差 90 度，不能直接把底盘坐标速度当成 KDL 基坐标速度。

## 依赖和启动

Ubuntu 22.04 / ROS 2 Humble 需要 MoveIt、`ros-humble-moveit-servo`、PyTorch 和支持 YOLOE 的 Ultralytics。Python 依赖在功能包 `setup.py` 中声明；本机 WSL 调试使用 PyTorch 2.5.1 和 Ultralytics 8.3.253。准备好 `config/visual_servo.yaml` 指定的本地权重与 embedding；运行时不会下载模型。

本机 WSL 可在仓库根目录使用独立入口管理视觉伺服仿真：

```bash
bash servo_test.sh start          # 启动节点，不触发抓取
bash servo_test.sh start --core   # 同时启动 Core 网页和 Web RViz
bash servo_test.sh grasp          # 触发一次绿色圆柱抓取并输出 Action 反馈
bash servo_test.sh verify         # 重置场景并运行一次独立真值验收
bash servo_test.sh status         # 查看脚本管理的进程与日志路径
bash servo_test.sh restart        # 重新启动仿真，使机械臂回到初始姿态
bash servo_test.sh stop           # 仅停止此脚本启动的进程
bash servo_test.sh start --scene visual_servo_convex  # 启动新夹指实验场景
bash servo_test.sh start --scene SCENE --scene-directory DIR --target-class 'red rectangular box' --object-id target_box
```

需要 MuJoCo 窗口时，使用 `start --gui` 或 `restart --gui`；也可组合 `start --gui --core` 和 `restart --core --gui`。`--core` 启动 Core 页面 `http://127.0.0.1:8780/` 和 Web RViz `http://127.0.0.1:8780/rviz/`，并使用 9091 端口的本地 rosbridge。Core token 读取环境变量 `EMBODIED_NEXUS_CORE_TOKEN`，未设置时使用仿真默认值 `sim-local-token-change-me`。视觉伺服的设备进程由 launch 启动，而非由 Core 管理；Core 的设备管理状态不代表这些进程的运行状态，也不应通过 Core 重复启动同一设备。

重复抓取前先执行 `restart`；`verify` 会重置场景，不要在机械臂仍靠近物体时直接重复运行。脚本默认使用本机 `grasp-hardness-install` underlay、`vs2-install` overlay 和 `/mnt/d/enx-local-cache/operator-runtime-py310`；可分别通过 `ENX_SERVO_BASE_SETUP`、`ENX_SERVO_SETUP`、`ENX_SERVO_RUNTIME` 指定其他位置。日志位于 `~/.local/state/embodied-nexus/servo-test/`。脚本不下载模型，不启动旧抓取 operator。

从仓库根目录构建并启动：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_visual_servo
source install/setup.bash
export YOLO_AUTOINSTALL=False
export MUJOCO_GL=egl
ros2 launch embodied_nexus_visual_servo visual_servo.launch.py gui:=false
```

如果 Python 依赖安装在其他目录，应按实际环境配置 `PYTHONPATH`。默认场景 `visual_servo_contact` 从原始 `scene-1793463913737` 派生：圆柱及其初始位置不变，使用简化指尖碰撞盒、较硬接触参数、固定底座约束和防滑求解。MoveGroup 与 MoveIt Servo 使用同一场景指尖盒；地面碰撞检查仍然启用。固定底座的场景忽略底座速度命令；切换到此场景时底座须在初始位置。原始场景文件保持不变。

每轮试验前重启仿真，使机械臂回到场景初始姿态。验收器重置场景；不要在夹爪靠近物体时直接重复重置。启动后运行：

```bash
ros2 run embodied_nexus_visual_servo visual_servo_acceptance --trials 1 \
  --object-id cylinder-1 --camera-id camera_front --arm-id arm_main \
  --gripper-id gripper_main --target-class 'green cylinder'
```

验收器独立检查目标相对初始位置抬高至少 5 cm、连续保持 2 秒且保持目标接触。Action 的 `grasp_verified` 保持 `false`，因为 Action 本身不使用仿真真值。检测丢失、过期数据、规划失败或控制异常时任务会停止，不将动作执行完成当作抓取成功。

指定其他场景时，应显式设置 `scene`、`scene_directory` 和与场景匹配的夹指碰撞选项；`finger_collision_geometry: convex` 使用离线生成的分段凸网格，MoveGroup 和 Servo 同步使用这些资产。柔性指垫实验场景尚未完成 MoveIt 碰撞几何匹配和抓取验收。`visual_servo_ground_box` 是盒体实验场景，也尚未验收。

## 2026-11-08 验证记录

在本地 WSL Ubuntu 22.04、Humble 和 MuJoCo 中，使用 `visual_servo_contact` 完成一轮从观察、YOLOE 检测、预抓取规划、视觉伺服接近、夹爪闭合到抬升及保持的完整操作。独立验收结果：

- 最大抬升 0.06139 m，连续保持至少 2 秒，判定通过；采集到 217 个双指目标接触样本。
- 首次接触时圆柱世界坐标约 `[0.88047, 0.00031, 0.01923]` m，试验中最大水平位移约 3.77 mm；视觉位置估计误差约 2.34 mm。
- Action 成功，`grasp_verified=false`；独立验收器报告 `passed=true`。

修复设备启动参数作用域和共享仿真包子包安装后，以默认启动命令从新构建再次完成一轮，最大抬升 0.06146 m、双指接触采样 223 次、最大水平位移 3.32 mm、视觉位置估计误差 2.98 mm，独立验收再次通过。两次试验均在场景初始位置进行，不代表五次不同位置稳定性或盒体通过。

调试时，原始网格指尖与柔性指垫场景均出现滑脱或被碰撞检查拦停。离线动力学对比发现，底座自由度受夹持反作用后被周期性复位会使圆柱横向移动；实验场景采用固定底座约束，并用防滑求解和接触参数减少接触蠕滑。验收仍以独立真值和接触反馈为准。


## 凸分解夹指的实验记录

保留原始夹指外观，并在 `visual_servo_convex` 中使用离线 CoACD 分段凸碰撞网格；MoveGroup、Servo 与 MuJoCo 使用同一组网格。网格来源、参数和源文件哈希位于 `src/sim/common/third_party/embodied_nexus_sim_description/meshes/gripper/ms42dc/convex/`。离线采样中左右夹指的最大表面偏差分别约 1.477 mm 和 1.489 mm；这是采样结果，不构成对整个表面的严格证明。

修正 Servo 的规划坐标系和关节速度跟踪后，基线圆柱、偏移圆柱、细圆柱、旋转圆柱及红色盒体均有过单次独立抬升通过的实验。红色盒体的一次成功实验水平位移达到约 3.6 cm，说明闭合过程仍可能推挤目标；单次成功不能替代每场景五次重启测试。`artifacts/vs3-matrix/` 保存重复测试的配置、逐轮结果、检测记录及运行日志；以各轮实际结果判断是否通过。

法向力阈值为仿真判定阈值，仿真接触力和摩擦参数未经实物标定。已有静态探针结果不能证明所有凹陷部位均不存在提前接触；故障注入和不同场景的重复测试必须分别记录，不能由单次成功推断。

在重复验收中曾观察到目标主轴跨越零点时夹爪期望方向跳变约 180°。伺服接近和停稳复查现从等价的平行夹爪姿态中选择最接近上一帧的方向；旧失败日志保存在 `artifacts/vs3-matrix-before-orientation-fix/`。闭合接触保持期间也会检查取消和外部停止，不能在取消后报告成功。

可复现的凸分解验收场景随 MuJoCo 包安装：`visual_servo_convex` 为原圆柱，`cylinder_shift` 为偏移圆柱，`cylinder_thin` 为细圆柱，`cylinder_yaw` 为旋转 45° 圆柱，`red_box` 为红色盒体。重新构建 `embodied_nexus_sim_mujoco` 后，可使用 `bash servo_test.sh start --scene cylinder_shift` 等选择场景；盒体需同时设置 `--target-class 'red rectangular box' --object-id target_box`。

2026-11-09，修复姿态方向折返和接触保持取消处理后，`visual_servo_convex` 的五次独立重启验收为 5/5 通过，最大抬升分别约 61.88、61.76、61.79、61.76、61.74 mm，每轮均连续保持 2 秒并记录到双指有效接触。其余场景及故障注入以实际记录为准。

同一版本下，`cylinder_shift`（初始 x 偏移 +2 cm、y 偏移 +4 cm）五次独立重启验收为 5/5 通过，最大抬升约 61.94–62.01 mm，最大水平位移不超过 1.42 mm。

`cylinder_thin` 五次独立重启验收为 5/5 通过，最大抬升约 61.88–62.02 mm，最大水平位移约 1.51 mm。轮次重启期间观察到 ROS 节点清理异常，完整验收结果已在关闭前写出。

`cylinder_yaw`（旋转 45°）五次独立重启验收为 5/5 通过，最大抬升约 61.70–61.79 mm，最大水平位移约 5.03 mm；其位移高于基线和偏移圆柱。

盒体渐进遮挡修正后，`red_box` 再次完成五次独立重启验收，5/5 通过；最大水平位移为 0.48–0.57 mm，每次抬升 62.10–62.15 mm。对照修正前首批 5 次，虽然都达到了抬升门槛，但其中两轮水平位移为 35.03 mm 和 39.07 mm，三轮低于 0.6 mm；修改后五轮位移稳定低于 0.6 mm。修正通过持续记录的最大可靠长轴尺寸识别渐进缩小的分割区域，并沿用裁切目标追踪。

视觉单元测试 13 项通过，夹爪核心测试 16 项通过。Humble colcon 构建相关 22 个包通过。实机验证未执行。

五场景每轮结构化结果汇总见仓库忽略目录中的 `artifacts/vs3-validation-summary.json`；该文件同时保留相位反馈、位移、双指峰值力和有效接触采样数。盒体渐进遮挡修正针对盒体分支，圆柱场景沿用同一份不变代码和配置。

取消故障已通过运行中 Action 注入：伺服阶段接受取消、Action 返回 `grasp cancelled` 且 `success=false`，随后 41 个关节状态样本的最大关节位置变化为 9.53e-9 rad。盒体视觉运行曾自然丢失一帧段目标，节点发布停止后重新识别并继续；这不是人为注入的目标丢失测试。深度/TF 故障、MoveIt 障碍拦截和租约失效的端到端注入尚未执行。
