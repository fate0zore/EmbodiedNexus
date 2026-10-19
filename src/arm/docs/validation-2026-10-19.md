# Aubo i5 机械臂验证记录（2026-10-19）

## 本地检查

- Windows / Python 3.12：机械臂测试运行 13 项，12 项通过；Starlette HTTP 单测因本地 Python 环境未安装 Starlette 而跳过。统一 SDK 测试 5 项通过。
- `compileall` 与 `git diff --check` 通过。

## Jetson Humble mock 联调

- 验证源码位于 `/tmp/EmbodiedNexus-arm-validation-6d091c`，未修改 Jetson 的 Git 管理源码工作区。
- `colcon list --base-paths src/arm` 发现 `embodied_nexus_arm_interfaces` 和 `embodied_nexus_arm`；`colcon --base-paths src/arm --symlink-install --packages-up-to embodied_nexus_arm` 构建通过。
- Jetson 上机械臂测试 13 项和 SDK 测试 5 项全部通过，包含 HTTP bearer 鉴权、WebSocket 状态订阅、SDK 租约和 mock 关节目标 action。模拟运动到位、停止请求被接受并确认静止；网关退出路径正常。

## 实机只读检查

- 通过真实 Aubo JSON-RPC 连接，成功读取六关节位置和速度、`Running` 模式、`Normal` 安全状态、示教关闭、硬件急停未触发、控制器关节范围、负载 0.818 kg 和当前零 TCP 偏移。
- 探测有 20 秒进程时限；连接前检查已知 Arachne 控制进程和标记。过程只发送状态与配置读取请求，没有发送停止、上电/下电、使能、运动或示教命令。
- 未执行实机关节运动、速度控制、生命周期或示教动作验收，后续仍需另行安排实机动作测试。

在 Jetson 加载 Humble 与工作区后，可运行以下同类只读探测：

```bash
timeout --signal=TERM 20s python3 src/arm/examples/probe_aubo_readonly.py \
  --config src/arm/embodied_nexus_arm/config/aubo_i5_real.yaml
```

## J6 单关节短动作与相机验收（未通过）

- 执行前再次确认控制器已连接，反馈新鲜度为 0.048 秒，模式 `Running`、安全状态 `Normal`，示教关闭、急停未触发、运动未使能、无其他租约或操作、静止反馈成立，关节限位有效。J6 起始角度为 0.889301 rad，唯一目标为 0.969301 rad（+0.08 rad），速度 0.05 rad/s、加速度 0.10 rad/s²。
- 动作目标被控制器接受，但 action 在 4.041 秒时以超时中止（`Aubo joint target did not reach tolerance before timeout`）；action 结果为失败。动作结果采样时 J6 正向变化 0.050544 rad，符合 0.04–0.12 rad 的角度范围，但未满足 action 成功条件。
- 眼在手相机验收使用静态环境纹理，无需机械臂本体入镜。动作前基线两组跟踪 173/168 个特征，P95 为 0.193/0.197 px；动作后跟踪 174 个特征，其中 155 个判为移动特征，中位位移 12.046 px，大于 1.0 px 阈值，相机判据通过。
- action 结束后的首次停止响应显示停止请求及控制器接受，但该响应尚未确认静止。随后状态轮询确认静止；释放租约成功，释放响应也确认停止与静止。机械臂运动已禁用且租约为空。最终只读探测显示 J6 为 0.941301 rad（相对起点约 +0.0520 rad），速度为零，模式仍为 `Running`、安全状态 `Normal`，急停未触发。
- **整体验收未通过**：action 超时失败，且首次停止响应未即时确认静止；关节变化与相机图像均确认发生了运动。停止后机械臂保持在当前姿态；没有重试，也没有发送返位目标。相机图像、状态和完整报告保存在 Jetson `/tmp/EmbodiedNexus-arm-motion-20261019/`。机械臂临时 ROS 节点退出码为 0，后续 ROS 图中相机服务仍运行。
