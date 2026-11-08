# 独立视觉伺服模块（MuJoCo）

`embodied_nexus_visual_servo` 独立于已有抓取 operator，通过 Nexus 设备公开 ROS 接口执行动作。YOLOE 使用本地权重和预先准备的文本 embedding 分割目标，RGB-D 与 TF 提供位置估计；MoveIt 规划观察、预抓取和抬升，MoveIt Servo 生成末端接近速度，Nexus 负责租约与关节执行。规划和感知不读取仿真物体真值；单独的验收器读取物体真值判断是否抬起。

当前仅支持 MuJoCo，禁止用于实机。两次圆柱抓取已通过独立验收，但尚未完成圆柱和盒体各五次验收，也未完成全部故障注入测试。

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

指定其他场景时，应显式设置 `scene`、`scene_directory` 和与场景匹配的 `contact_finger_boxes`；禁用该选项时使用原始指尖网格。柔性指垫实验场景尚未完成 MoveIt 碰撞几何匹配和抓取验收。`visual_servo_ground_box` 是盒体实验场景，也尚未验收。

## 2026-11-08 验证记录

在本地 WSL Ubuntu 22.04、Humble 和 MuJoCo 中，使用 `visual_servo_contact` 完成一轮从观察、YOLOE 检测、预抓取规划、视觉伺服接近、夹爪闭合到抬升及保持的完整操作。独立验收结果：

- 最大抬升 0.06139 m，连续保持至少 2 秒，判定通过；采集到 217 个双指目标接触样本。
- 首次接触时圆柱世界坐标约 `[0.88047, 0.00031, 0.01923]` m，试验中最大水平位移约 3.77 mm；视觉位置估计误差约 2.34 mm。
- Action 成功，`grasp_verified=false`；独立验收器报告 `passed=true`。

修复设备启动参数作用域和共享仿真包子包安装后，以默认启动命令从新构建再次完成一轮，最大抬升 0.06146 m、双指接触采样 223 次、最大水平位移 3.32 mm、视觉位置估计误差 2.98 mm，独立验收再次通过。两次试验均在场景初始位置进行，不代表五次不同位置稳定性或盒体通过。

调试时，原始网格指尖与柔性指垫场景均出现滑脱或被碰撞检查拦停。离线动力学对比发现，底座自由度受夹持反作用后被周期性复位会使圆柱横向移动；实验场景采用固定底座约束，并用防滑求解和接触参数减少接触蠕滑。验收仍以独立真值和接触反馈为准。
