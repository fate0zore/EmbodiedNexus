# 独立三维动态目标抓取（MuJoCo）

`embodied_nexus_dynamic_grasp` 是单独启动的实验路线，使用 RGB-D、YOLOE 和采集时刻的 TF 估计目标位姿及运动，使用 MoveIt/MoveIt Servo 和 Nexus 设备公开接口执行。它不启动或调用静态视觉伺服任务，也不使用仿真真值作感知或控制输入。目标状态发布到 `/embodied_nexus/dynamic_grasp/target`，任务入口是 `/embodied_nexus/dynamic_grasp/grasp`，类型为 `embodied_nexus_dynamic_grasp_interfaces/action/DynamicGrasp`。

当前只支持固定底座的 MuJoCo 实验，禁止用于实机。场景、夹指摩擦和质量是仿真参数，尚未标定到实物。三维几何由单视角部分点云和主轴估计，遮挡、近似对称或不可观测的姿态会导致估计失败；圆柱绕自身长轴的旋转不可观测，不计入停稳角速度。目标丢失、数据过期、TF 不可用、租约失效和碰撞拦截时，动作停止或失败，不能把 Action 运行完成当作抓取成功。

## 运行

需要 Ubuntu 22.04、ROS 2 Humble、MoveIt Servo、MuJoCo、PyTorch、OpenCV 和支持 YOLOE 的 Ultralytics，以及配置文件指定的本地权重和文本 embedding；模型资产准备方式见[视觉伺服指南](visual-servo.md#依赖和启动)。运行时不下载模型。下面的命令从仓库根目录执行：

先构建持久的独立工作区（重新构建也使用这些目录）：

```bash
dynamic_cache="${XDG_CACHE_HOME:-$HOME/.cache}/embodied-nexus/dynamic-grasp"
bash scripts/build.sh embodied_nexus_dynamic_grasp --serial \
  --build-base "$dynamic_cache/build" --install-base "$dynamic_cache/install" \
  --log-base "$dynamic_cache/log"
```

构建完成后，可以使用独立入口启动仿真并演示动态追踪：

```bash
bash dynamic_grasp_test.sh start                 # 无界面启动
bash dynamic_grasp_test.sh restart --core --gui  # 重启并启动 Core 和 MuJoCo 界面
bash dynamic_grasp_test.sh grasp                 # 准备目标，追踪阶段触发运动
bash dynamic_grasp_test.sh status
bash dynamic_grasp_test.sh stop
```

`--core` 和 `--gui` 也可单独使用。默认读取上述缓存目录中的 `install/setup.bash`；自定义安装路径可用 `ENX_DYNAMIC_SETUP`，可选底层环境和 Python 运行时分别使用 `ENX_DYNAMIC_BASE_SETUP`、`ENX_DYNAMIC_RUNTIME`。启动参数和日志保存在用户状态目录的 `embodied-nexus/dynamic-grasp-test/` 中。

默认场景 `dynamic_grasp_tabletop_cylinder` 在低摩擦支撑面上沿圆柱长轴向机械臂方向施加水平力；速度参考从 10 mm/s 线性降至零，175 个仿真秒后移除外力，简化动力学测试中的位移约 50 cm。物体始终是自由刚体，释放后由接触动力学决定何时停止；实际墙钟时间取决于仿真速度，完整轨迹是否保持可见和可达需要验收。为避免长时间缓慢移动被过早判断为静止，桌面场景采用连续 5 秒、线速度低于 1 mm/s、位置波动低于 2 mm 的判定；仍要求新鲜观测。红色盒体示例：

本机整机试跑中，目标尚未走完轨迹时即被腕部相机画面裁切，视觉追踪失败；演示客户端随后释放目标，实际位移没有达到 50 cm。上述 50 cm 目前仅在简化自由刚体测试中验证，不能作为整机抓取效果保证。

桌面盒体模式允许在只观测到水平顶面时，利用已知支撑面高度估计盒体厚度；它依赖目标保持接触该支撑面的场景前提，不用于三维下降或悬空目标。

```bash
bash dynamic_grasp_test.sh start --gui --scene dynamic_grasp_tabletop_box \
  --target-class 'red rectangular box' --object-id target_box --object-mass-kg 0.08
bash dynamic_grasp_test.sh grasp
```

演示不会重置场景；重复运行前使用 `restart` 恢复初始场景和机械臂姿态。演示结果是 Action 结果，不构成独立抬升验收。

也可以直接启动 launch：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_dynamic_grasp
source install/setup.bash
export YOLO_AUTOINSTALL=False MUJOCO_GL=egl
ros2 launch embodied_nexus_dynamic_grasp dynamic_grasp.launch.py gui:=false
```

启动只建立节点，不自动触发抓取或目标运动。另开终端并 source 相同的 ROS 环境，运行一次独立验收：

```bash
ros2 run embodied_nexus_dynamic_grasp dynamic_grasp_acceptance \
  --trials 1 --scene-id dynamic_grasp_tabletop_cylinder --object-id cylinder-1 \
  --camera-id camera_front --arm-id arm_main --gripper-id gripper_main \
  --target-class 'green cylinder' --fixture-phase track \
  --output-dir artifacts/dynamic-grasp-tabletop
```

验收器重置场景，等待目标在支撑面稳定后发送 Action，在指定阶段触发水平滑动；机构发布零力释放目标，不重写物体位置或速度。重置会改变场景，不要在机械臂靠近物体时重复运行；每轮完整重复测试应先重启仿真以恢复初始机械臂姿态。`--fixture-phase` 可选 `plan`、`track`、`approach`。新场景自动选用 `tabletop` 运动模式；原有三维下降场景可显式选择 `dynamic_grasp_cylinder` 或 `dynamic_grasp_box`，继续使用 `combined` 等原模式。

`--output-dir` 保存独立真值轨迹、动作反馈、采集时刻估计误差和观测年龄；跟踪误差按动作反馈采样，不能代表未采样时间的最大误差。

红色盒体使用独立场景和匹配参数：

```bash
ros2 launch embodied_nexus_dynamic_grasp dynamic_grasp.launch.py \
  scene:=dynamic_grasp_tabletop_box target_class:='red rectangular box' \
  object_id:=target_box object_mass_kg:=0.08

ros2 run embodied_nexus_dynamic_grasp dynamic_grasp_acceptance \
  --trials 1 --scene-id dynamic_grasp_tabletop_box --object-id target_box \
  --camera-id camera_front --arm-id arm_main --gripper-id gripper_main \
  --target-class 'red rectangular box' --fixture-phase track
```

也可直接发送 Action，不启动验收器：

```bash
ros2 action send_goal /embodied_nexus/dynamic_grasp/grasp \
  embodied_nexus_dynamic_grasp_interfaces/action/DynamicGrasp \
  '{target_class: "green cylinder", object_id: "cylinder-1", camera_id: "camera_front", arm_id: "arm_main", gripper_id: "gripper_main"}' --feedback
```

直接发送 Action 时测试机构不会自动运动；需先向 `/embodied_nexus/dynamic_grasp/fixture_prepare` 发送目标 ID，等待 `/embodied_nexus/dynamic_grasp/fixture_event` 发布 `ready:<目标ID>`，再在指定时机向 `/embodied_nexus/dynamic_grasp/fixture_start` 发送目标 ID。`test_fixture:=false` 可以关闭机构。该机构和 `/sim/arachne/mujoco/test_object_wrench` 均仅用于仿真实验场景，外部力有范围限制和失联超时。

## 控制与记录

任务申请并续租机械臂和夹爪控制权，先获取目标和执行受目标变化监控的预抓取规划；发生位移、观测丢失或过期时取消子动作并确认机械臂停止，再重新规划。动态伺服保持目标上方 12 cm 预抓取偏移，结合速度前馈、位置及姿态误差反馈。目标连续至少 1 秒满足线速度、可观测角速度和位姿波动阈值后才进行最终接近；接近或闭合前再次检测到运动时停止并返回追踪。

Action 反馈包含阶段、目标状态和追踪误差；任务运行事件记录在 `~/.cache/embodied-nexus/logs/dynamic-grasp-runs/`。Action 结果的 `grasp_verified` 始终为 `false`，独立验收器检查至少 5 cm 的抬升和 2 秒保持，并检查移动夹具、提前接触、静止闭合和跟踪误差。具体本地试验结果见[动态抓取验证记录](dynamic-grasp-validation-2026-11-10.md)。

新桌面场景的实测轨迹及当前近距离视觉限制见[桌面动态追踪试跑记录](tabletop-dynamic-grasp-validation-2026-11-10.md)。

在仓库根目录运行单元测试：

```bash
PYTHONPATH=src/operator/embodied_nexus_dynamic_grasp python -m unittest discover -s src/operator/embodied_nexus_dynamic_grasp/test -v
```

涉及 ROS 消息和 Action 的测试需要已构建并 source 的 Humble 工作区。MuJoCo 测试按包运行 `python -m unittest discover -s src/sim/mujoco/embodied_nexus_sim_mujoco/test -v`。
