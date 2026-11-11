# 动态抓取仿真试跑记录（2026-11-10）

## 环境和结果

本地 WSL Ubuntu 22.04、ROS 2 Humble、MuJoCo 和 NVIDIA RTX 3090；使用 `dynamic_grasp_cylinder`、绿色圆柱、`combined` 运动和 `--fixture-phase track`。启动了独立相机感知、动态 Action、MoveIt Servo、Nexus 机械臂与夹爪、MuJoCo 测试机构及独立验收器。此轮为全链路试跑，**抓取未通过**：Action 最终返回 `dynamic visual tracking timed out`，没有进入夹爪闭合或抬升。

测试机构先使目标悬停，机械臂进入预抓取跟踪后触发目标运动；目标在最终接近期间再次移动，控制器报告返回追踪，目标随后落到支撑面并释放。多次短时视觉丢失触发停臂与重新获取；本轮最终未重新收敛到闭合所需位置误差，超时退出。没有检测到提前接触。单次试验不能证明控制误差、遮挡恢复或抓取成功率满足验收标准。

| 指标 | 本轮结果 |
| --- | --- |
| 目标最大实测线速度 | 0.00869 m/s |
| 目标最大实测角速度 | 0.04047 rad/s |
| 目标最大水平位移 | 0.01443 m |
| 追踪误差最大值 | 0.01919 m，来自 101 条阶段反馈采样，非全时最大值 |
| 单次视觉位置估计误差 | 0.01441 m，以最近时间戳的仿真真值对照 |
| 提前接触、闭合及抬升 | 未检测到提前接触；未进入闭合和抬升 |

验收器记录了 2,840 个仿真真值样本；完整的本轮汇总和逐阶段反馈见[结构化结果](dynamic-grasp-run-2026-11-10.json)。Action 自身不读取这些真值；试验过程的独立设备与控制日志保存在 WSL 的 `~/.cache/embodied-nexus/logs/dynamic-grasp-runs/0abe7ee381d0456bafef3f14c03c5657.jsonl`。

同一开发环境下，Humble `colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_dynamic_grasp` 构建 22 个包通过；动态抓取单元测试 19 项、MuJoCo 包单元测试 37 项通过。圆柱和盒体两种动态场景均通过场景加载和模型检查。盒体端到端运行、其他轨迹、故障端到端注入、重复抓取成功率和实机动作验证均未执行。
