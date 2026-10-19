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
