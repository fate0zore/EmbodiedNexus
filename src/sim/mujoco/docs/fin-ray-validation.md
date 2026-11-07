# 独立 Fin Ray 夹爪验证

本例在 MuJoCo 3.2.7 中单独模拟双指闭合、夹取横放圆柱和抬升，不连接 ROS、Nexus 或机械臂。构型参考 [MuJoCo 官方 Fin Ray 示例](https://github.com/google-deepmind/mujoco/blob/main/model/flex/gripper_2d.xml)，网格由本例按参数生成；上游当前示例使用的二维节点自由度不是 3.2.7 可直接照搬的配置。

每根手指由两条柔性边和七层横肋构成，根部固定于刚性滑动夹座。3.2.7 为未固定的柔性顶点生成三个滑动自由度；示例对垂直指体平面的自由度设置弹簧和阻尼，使之接近平面运动，但不等于严格的二维运动。软体由三角形网格和边长约束建模；柔性轮廓的碰撞半径为 3 mm。参数用于验证建模链路，尚未按实物尺寸、材料和夹紧力标定。

横放圆柱半径 25 mm、长度 100 mm、质量 100 g；夹指和圆柱摩擦系数为 5。仿真步长 0.5 ms：静置 0.5 秒、闭合 2.5 秒、等待 0.5 秒、抬升 2 秒、保持至第 8 秒。脚本每 0.05 仿真秒记录物体位置、各指接触数量和法向力总和、相对于夹座的最大顶点位移。判定要求保持阶段物体相对初始高度至少抬升 5 cm，并持续至少 2 秒。

在 WSL 仓库根目录执行，使用本机已安装的 MuJoCo：

```bash
python3 src/sim/mujoco/examples/fin_ray_validation.py --output /tmp/fin-ray-validation.json
python3 src/sim/mujoco/examples/fin_ray_validation.py --gui --output /tmp/fin-ray-validation-gui.json
```

`--gui` 按实时速度显示 MuJoCo 原生窗口，运行约 8 秒；无 `--gui` 为无窗口快速运行。运行不会触发真实设备或修改现有机器人模型。输出包括完整时间序列；终端输出摘要。模型是仿照 Fin Ray 的二维肋条近似，不能据此推断与实物形状或橡胶力学一致。

## 2026-11-02 本机验证

最终配置在 WSL Ubuntu 22.04、MuJoCo 3.2.7 完成一次 8 秒仿真，没有触发数值警告。出现 43 个双指同时接触采样，左右指最大相对位移分别为 15.21 mm 和 15.60 mm；圆柱最高位置比初始高度高约 0.43 mm，保持阶段回落到地面，**未通过抬升 5 cm、保持 2 秒的验收**。采样到的左右单指法向力总和峰值分别为 0.441 N 和 0.441 N；峰值不代表持续夹紧力。

时间序列保存在本机 `/home/dministrator/.cache/embodied-nexus/logs/fin-ray-validation-20261102-final.json`。后续需要对照实物的夹指刚度、截面、接触方向与闭合行程继续调整，并检查各时刻的接触和摩擦；当前结果不能归因于单一参数。

## Arachne 场景的 Fin Ray 配置

绿色圆柱场景当前使用原始 MS42DC 夹指 STL（`finger_collision_geometry: mesh`），关闭 Fin Ray 柔性体，并保留 `finger_contact_material` 中的摩擦和柔顺接触参数。原始网格沿用 URDF 的 0.001 缩放，单侧源网格轴对齐包围尺寸约为 44.94 × 102.60 × 22.80 mm。MuJoCo 对刚体网格接触使用凸包近似，凹陷部分的接触轮廓未必与 STL 一致；刚体接触柔顺也不会产生可见网格形变。Web RViz 的碰撞盒图层只显示盒体，不显示此网格碰撞近似。

Fin Ray 模式仍可供单独实验：将 `finger_collision_geometry` 改为 `boxes` 并启用 `finger_flex_pads`。三维四面体肋条模型的 `size_m` 三个值分别为肋条和外侧梁在夹指平面内的厚度、沿夹指的长度以及垂直该平面的宽度；夹指轮廓的最大横向展开仍由末端碰撞盒决定。`grid` 分别指定每侧梁厚度方向的节点数、沿长度方向的肋条层数，以及宽度方向的节点层数。质量按 MuJoCo 的 flexcomp 规则分配到节点，固定节点属于夹指父体。

`young_pa`、`poisson` 和 `damping` 配置三维弹性，`radius_m` 配置柔性元素接触半径；`fin_ray_edge_time_constant_s` 配置边长等式约束，`fin_ray_out_of_plane_stiffness_n_m` 配置节点离面弹簧刚度。材料弹性与边长约束共同影响形变，强边长约束可能掩盖部分材料参数的作用。摩擦由 `finger_contact_material` 单独配置。

为避免 MuJoCo 将边约束时间常数限制为至少两个仿真步长，Fin Ray 的步长取 `min(0.0005, fin_ray_edge_time_constant_s / 2)` 秒。绿色圆柱场景保留的 Fin Ray 实验参数为 `grid: [2, 3, 2]` 和 `fin_ray_edge_time_constant_s: 0.001`，启用时对应每指 48 个顶点和 0.0005 秒步长；早先的 `[2, 7, 3]` 和 `0.0005` 配置每指有 168 个顶点，步长为 0.00025 秒。粗网格和较大的约束时间常数会改变肋条数量与柔顺性，不能认为两者接触力相同。需要高分辨率时可以在场景中改回先前配置，代价是仿真速度下降。

启用 Fin Ray 时采用 CG 求解器、最多 50 次迭代。实际仿真速度、碰撞精度和完整抓取效果需要在目标机器上验收；本节不改变上面的独立二维实验及其历史结果。
