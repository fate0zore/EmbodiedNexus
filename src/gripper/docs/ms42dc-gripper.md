# MS42DC 夹爪接入与验证

本文说明当前 MS42DC 平行夹爪适配。实现基于本地 Arachne 直连串口驱动，不包含其他夹爪型号、夹指宽度标定、力反馈或抓取策略。

## 架构与状态

`embodied_nexus_gripper` 独占串口并提供设备状态、控制租约和运动 action。HTTP 网关只通过 ROS 2 请求该节点，不连接硬件。`backend=mock` 使用相同的控制状态机与反馈判定，可注入反馈超时、停止失败和断连。

设备启动时运动禁用。真实配置中的 `feedback.confirmed` 默认为 `false`；只有针对实际设备核对反馈字段后，才可在设备配置中设为 `true`。2026-10-18 在 Yahboom/MS42DC 上验证了反馈字节 1 的 `reached=0` 表示运动中、`reached=1` 表示到位：开合期间速度反馈非零且电机角度变化，到位后速度为零。该结论仅适用于本次验证的设备和固件，不应直接推广到其他固件构型。到位判定要求本次命令后先观察到运动中、再收到新鲜的到位反馈。状态保留完整的 32 位反馈电机角度。未做夹指标定的开口宽度、夹指位置和力保持未知。设备描述给出驱动/接口版本，配置和状态包含会话内递增的 `config_version`；GET `/config` 返回最终生效配置，方便实验记录保存。

## 构建与运行

以下相对路径均以仓库根目录为起点。在已 source ROS 2 Humble 的 Ubuntu 22.04 工作区中安装 Python 依赖并构建：

```bash
sudo apt install python3-serial python3-yaml python3-starlette python3-uvicorn
colcon build --base-paths src --packages-up-to embodied_nexus_gripper
source install/setup.bash
```

先用 mock 启动并检查配置：

```bash
gripper_check --config src/gripper/embodied_nexus_gripper/config/ms42dc_mock.yaml --show-config
ros2 launch embodied_nexus_gripper gripper.launch.py \
  config_file:="$(ros2 pkg prefix embodied_nexus_gripper)/share/embodied_nexus_gripper/config/ms42dc_mock.yaml"
```

HTTP 节点要求配置的环境变量中存在 bearer token：

```bash
export EMBODIED_NEXUS_GRIPPER_TOKEN='replace-with-a-local-secret'
```

设备默认使用 `gripper_main`，HTTP 地址为 `http://robot-host:8776/api/v1/grippers/gripper_main`。用 `interface_mode:=ros2` 可只启动设备节点；`both`（默认）或 `web` 会额外启动 HTTP 网关节点。网关启动后，HTTP 接口要求 `Authorization: Bearer <token>`。

## ROS 2 与 HTTP 接口

ROS 2 名称前缀为 `/embodied_nexus/grippers/gripper_main`，包含 `/state` 状态话题、`/info`、`/get_state`、`/config`、`/config/validate`、`/config/apply`、`/control/acquire`、`/control/renew`、`/control/release`、`/motion/enable` 和 `/stop` 服务，以及 `/move` action。移动命令必须通过 action 提供有效租约和唯一的 `client_operation_id`；没有旁路运动 topic。

HTTP 对应接口：

| 方法         | 路径                                                                     | 用途                                          |
| ------------ | ------------------------------------------------------------------------ | --------------------------------------------- |
| GET          | `/info`、`/status`、`/config`                                      | 设备能力、状态和配置                          |
| POST         | `/control/lease`、`/control/lease/renew`、`/control/lease/release` | 获取、续约和释放控制权                        |
| PUT          | `/motion/enabled`                                                      | 显式启用或禁用运动                            |
| POST         | `/operations`                                                          | 创建 `open`、`close` 或 `relative` 动作 |
| GET / DELETE | `/operations/{operation_id}`                                           | 查询或取消动作                                |
| POST         | `/stop`                                                                | 请求停止；无需租约                            |
| POST / PUT   | `/config/validate`、`/config`                                        | 检查配置或持租约应用允许运行时调整的字段      |

操作成功代表收到符合已确认配置的新鲜到位反馈。停止响应分别给出 `stop_sent` 与 `hardware_stop_confirmed`。停止报文发送成功不代表硬件已确认停稳。

## Python SDK

统一 SDK 的安装包名为 `embodied-nexus-client`，导入名为 `embodied_nexus_client`，同时提供相机、夹爪与雷达客户端：

```bash
pip install -e sdk/python
```

```python
from embodied_nexus_client import GripperClient

gripper = GripperClient("http://robot-host:8776", token="local-secret")
with gripper.acquire_control("pick-demo") as lease:
    lease.enable_motion()
    operation = lease.move_open()
    result = operation.wait(timeout=12)
    if not result["success"]:
        raise RuntimeError(result["error"])
```

SDK 无 ROS 依赖。`GripperLease` 在后台自动续约，操作句柄提供 `wait()`、`status()` 和 `cancel()`。正的相对电机角度代表闭合，负值代表打开；单位为度，设备命令使用十分之一度分辨率。请求非有限值或超过配置边界会被拒绝。

## 配置和故障注入

真实配置基线为 `/dev/motor_serial`、115200 波特率、设备编号 1、细分 32、模式 2、开合角度 1965.6 度和原始速度 150。角度是电机运动参数，不是夹指开口宽度；接线、方向和实际行程必须在现场核对。运动租约默认 0.5 秒续约、2 秒过期。

启动参数使用 `--config PATH`，或在 launch 中设置 `config_file:=PATH`。配置工具支持检查、输出合并配置和差异比较：

```bash
gripper_check --config config.yaml
gripper_check --config config.yaml --show-config
gripper_check --config config.yaml --diff-from config.previous.yaml
```

mock 中可通过应用配置注入故障，例如持有租约且设备空闲时设置 `{"mock":{"fail_feedback":true}}`。真实后端拒绝 mock 故障字段。串口及协议字段需重启生效；运行时参数修改需有效控制租约且无活动动作，整批校验通过后才会应用。

## 自动化与实机验收

协议及控制器测试：

```bash
PYTHONPATH=src/gripper/embodied_nexus_gripper python -m unittest discover \
  -s src/gripper/embodied_nexus_gripper/test -v
```

自动化覆盖协议帧、校验、32 位反馈、运动使能、租约冲突/过期、完成反馈、取消、超时和停止失败。ROS 2 节点与网关构建需在 Humble 工作区验证。

2026-10-18 通过 SSH 在 Yahboom（Ubuntu 22.04、ROS 2 Humble、Jetson aarch64）完成实机闭合和打开测试。Arachne 已停止，夹爪节点独占 `/dev/ttyACM0`（配置使用对应的 USB by-id 路径）。测试使用临时验证配置，在 10 秒动作超时、租约自动续约和严格反馈判定下分别运行完整 `close` 与 `open`：闭合时电机角度原始值由 79 增至 19697，打开时由 19697 降至 144；两个动作均观察到 `reached=1 → reached=0 → reached=1`，运动速度峰值约 147，action 成功，耗时约 2 秒。动作结束后均禁用了运动并释放控制租约。另一次 +3° 相对运动未产生可观察的反馈变化并按超时路径停止，因此小幅相对运动仍需单独评估。

验证结束后已恢复 `feedback.confirmed: false` 的安全启动配置；当前真实节点连接、空闲、运动禁用且无租约。相机服务同时保持运行，WebSocket 收到有效 640×480 JPEG 帧。仓库默认配置保持运动禁用；部署到已验证的同一设备时，仍需由操作者明确核对并启用已验证的反馈参数。若更换设备或固件，应重新验收。开口宽度、夹指位置、力反馈及抓取策略仍未验证或标定。

## 来源与许可

协议抽取自工作区本地 Arachne：`src/arachne_hardware/arachne_hardware/ms42dc_direct_serial_driver.py`，并参考其随附的 `third_party/ms42dc_step_motor_ros2/Step_Motor_ROS2/src/step_motor/src/motor_node.cpp`。本地 Arachne HEAD 为 `170ce0bf713bd1a6626677e70b3dc12293858147`；总体规划中的 `41caa56` 在本地仓库不可解析，因此此处记录了实际读取的 HEAD。Arachne 仓库包含 MIT License；抽取协议辅助逻辑保留该许可文本，具体来源记录见 [ms42dc-source.md](ms42dc-source.md)。
