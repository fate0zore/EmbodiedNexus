# MuJoCo 绿色圆柱抓取复验（2026-11-01）

## 本机环境

- 工作区：Windows `D:\code\project\EmbodiedNexus`；WSL 路径 `/mnt/d/code/project/EmbodiedNexus`
- 系统与 ROS：WSL2 Ubuntu 22.04、ROS 2 Humble、Humble MoveIt2
- 构建 overlay：`/home/dministrator/.cache/embodied-nexus/operator-install`
- Python：3.10.12
- GPU 与视觉库：NVIDIA GeForce RTX 3090，PyTorch `2.5.1+cu121`，Ultralytics `8.3.253`，CUDA 可用
- 本地 Python 运行依赖：`/mnt/d/enx-local-cache/operator-runtime-py310`
- 本地模型资产：`/home/dministrator/.cache/embodied-nexus/models/` 下的 `yoloe-11s-seg.pt`、`mobileclip_blt.ts`、`yoloe-green-cylinder-text.npz`

ROS 启动的 operator 使用系统 Python，因此必须把本地依赖目录加入 `PYTHONPATH`，并设置 `YOLO_AUTOINSTALL=False`。上次出现 `No module named ultralytics` 是启动环境漏设 `PYTHONPATH`；依赖和模型实际一直保留在本机。此次运行没有连接服务器、联网安装依赖或下载模型。

本次重新构建 `embodied_nexus_sim_mujoco` 与 `embodied_nexus_operator` 依赖链：Humble colcon 构建 **21 个包全部通过**。

## 使用参数和触发

使用专用横放绿色圆柱场景 `scene-1793463913737.yaml`，并重新启动 MuJoCo 以加载：

```yaml
arm_position_actuator:
  kp: 2500.0
  kv: 80.0
  max_force: 500.0
```

通过本机 HTTP `POST /plan` 仅触发一次，目标是 `green cylinder` / `cylinder-1`。任务 ID：`6284c9246f83440da2eb5c3bb2b5a09e`；ROS action goal ID：`2de3c9c7752e4d189e83be876f33db19`。

## 阶段结果

| 阶段 | 结果 | 日志证据 |
| --- | --- | --- |
| 观察位姿 | 通过 | 84 点轨迹执行完成，最大关节误差 `0.000981 rad` |
| YOLOE 与 RGB-D 定位 | 通过 | 置信度 `0.192775`，487 个目标点；RGB 与深度时间戳相同；估计中心 `[0.881188, 0.000351, -0.221145] m`（`base_link`） |
| 预抓取规划 | 通过 | 笛卡尔路径完成率 `1.0`，276 点轨迹 |
| 预抓取执行 | 通过 | 最大关节终点误差 `0.006839 rad`，小于 `0.01 rad` 到位阈值；arm 状态确认静止。对比前次 `kp=1500` 时的 `0.011383 rad`，本次误差下降约 40% |
| 最后接近规划 | **失败** | MoveIt 笛卡尔路径完成率 `0.807692`，低于 `0.90` 门槛；26 个部分路径点仅记录用于预览，没有执行 |
| 闭合与抬升 | 未执行 | 规划在接近阶段中止；没有完成夹爪闭合、接触夹持或抬升，`grasp_verified=false` |

任务在接近规划失败后结束，耗时约 49.5 秒。机械臂状态随后报告操作完成且已静止。当前日志能确认路径未达到配置要求，但不能仅据 `fraction=0.807692` 判定具体是碰撞约束、IK 分支还是运动学奇异点；需结合 MoveIt 的规划场景和部分路径终点继续分析。

## 日志与轨迹

- 抓取事件日志：`/home/dministrator/.cache/embodied-nexus/logs/grasp-runs/2de3c9c7752e4d189e83be876f33db19.jsonl`
- 观察轨迹：`/home/dministrator/.cache/embodied-nexus/trajectories/1793506352251264922-observe-3a305f70.json`
- 预抓取轨迹：`/home/dministrator/.cache/embodied-nexus/trajectories/1793506370578798256-pregrasp-49c486dc.json`
- 部分接近轨迹（只预览，未执行）：`/home/dministrator/.cache/embodied-nexus/trajectories/1793506406606348986-cartesian-partial-3491e0c6.json`
- MoveIt 启动日志：`/home/dministrator/.cache/embodied-nexus/logs/move_group-20261101.log`
- operator 启动日志：`/home/dministrator/.cache/embodied-nexus/logs/grasp_operator-20261101.log`

本次验证了执行器增益调整后预抓取关节终点误差低于阈值。整体抓取仍未通过，因为 MoveIt 在最后接近阶段只返回了 80.8% 的路径；夹爪接触和物体抬升验收尚未到达。

## 接近段修正后的复验

绿色圆柱的 `finger_contact_center_offset_grasp_m` 从 `[0, 0, -0.010]` 调整为 `[0, 0, 0.018]` 米。俯视时工具 Z 朝下，这使 `grasp_frame` 相对估计的圆柱中心上移 18 mm；按张开夹指的 MoveIt 网格估算，目标处指尖距地面约 7 mm。规划仍保留地面碰撞检查。

重启本机仿真后通过 HTTP `/plan` 触发一次，任务 ID 为 `58cd5b591c7348f2be300e8b7de0e02c`，action goal ID 为 `c9df1cee5ca9411ba39dcf22880fe4a8`。operator 包 Humble 构建通过。观察、YOLOE 检测与定位、预抓取均完成；最后接近的 MoveIt 路径完成率为 `1.0`，30 点轨迹执行结束时最大关节误差为 `0.007653 rad`，末端位置跟踪误差约 `0.0067 m`。

状态机进入了闭合后抬升阶段；根据闭合后的代码检查，这表明夹爪动作报告成功、确认接触且物体 ID 为 `cylinder-1`。日志没有把闭合结果字段单独记录，以上接触结论依据状态机进入抬升前的检查。抬升轨迹规划和执行完成，但抬升后未保持目标接触，任务最终失败，`grasp_verified=false`。未进行物体抬升高度与保持时间的独立真值验收。

本次事件日志为 `/home/dministrator/.cache/embodied-nexus/logs/grasp-runs/c9df1cee5ca9411ba39dcf22880fe4a8.jsonl`，接近轨迹为 `/home/dministrator/.cache/embodied-nexus/trajectories/1793507340037363777-approach-4c907f41.json`。

后续核对发现任务结束后的 MuJoCo 夹指关节仅约为左 `+0.0503 rad`、右 `-0.0503 rad`，仍接近张开状态。因此上面的接触确认只能说明设备动作在某时刻报告了双指接触，不能据此断言发生了足够的闭合行程或形成稳定夹持。此次没有记录闭合期间的逐帧关节和接触数据，无法还原具体何时触发接触。当前仿真的默认简化指尖碰撞盒半尺寸为 `[0.023, 0.052, 0.012]` 米，其张开状态下的内侧间距估计约 27 mm，小于目标圆柱的 30 mm 直径，可能导致过早报告双指接触；该原因仍需结合逐帧数据验证。

## 夹爪闭合行程复验

圆柱场景新增 `finger_contact_box_half_size_m: [0.010, 0.045, 0.011]`，只覆盖该场景的简化指尖碰撞盒；未指定该字段的场景仍使用启动参数。重新构建 MuJoCo 包并重启本机仿真后，通过 HTTP `/plan` 执行一次，任务 ID `c919682f8ee44b5bbb06114565d2271a`，action goal ID `021bf8aa77214047b069adbe30417ab1`。

本次额外记录 `/sim/arachne/joint_states` 和 `/sim/arachne/gripper/contact_state`。夹指从接近 `0 rad` 运动到左 `+0.2100 rad`、右 `-0.2100 rad`，与上次任务结束时约 `±0.0503 rad` 相比闭合行程更明显；接触话题的 1473 个采样中有 22 个报告接触。任务仍在抬升后因目标接触未保持而失败，不能据此认定抓取成功；没有进行独立的物体抬升和保持时间验收。

逐帧记录位于 `/home/dministrator/.cache/embodied-nexus/logs/gripper-joint-timeline-20261101.log` 和 `/home/dministrator/.cache/embodied-nexus/logs/gripper-contact-timeline-20261101.log`；动作事件位于 `/home/dministrator/.cache/embodied-nexus/logs/grasp-runs/021bf8aa77214047b069adbe30417ab1.jsonl`。MuJoCo 模型转换测试 8 项通过；第一次测试命令误覆盖了 ROS Python 搜索路径，恢复 overlay 环境后重跑通过。
