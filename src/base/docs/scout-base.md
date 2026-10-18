# Scout 2.0 底盘接入与控制

本模块将 Scout 2.0 底盘独立成 ROS 2 设备服务。当前提供 mock 与 Waveshare USB-CAN-A 串口后端；服务启动默认禁用运动，运动前需要有效控制租约、显式使能和新鲜反馈。

## 本地 mock

在 Ubuntu 22.04 / ROS 2 Humble 环境从仓库根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_base
source install/setup.bash
ros2 launch embodied_nexus_base base.launch.py
```

默认配置为 `scout_mock.yaml`，只运行模拟后端。`interface_mode` 默认 `ros2`，需要 HTTP/WebSocket 时应显式配置 `both`，并设置 `EMBODIED_NEXUS_BASE_TOKEN`。默认服务端口为 `8778`。

## Scout Waveshare 配置

`config/scout_waveshare.yaml` 对应机器人 Arachne 中已使用的 USB-CAN-A 接法：稳定串口路径 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`、USB 串口 `2000000` baud、CAN `500000` bit/s、标准帧。部署时先确认设备枚举及串口权限，再使用该配置启动；Web 接口还需配置与客户端一致的令牌。例如：

```bash
export EMBODIED_NEXUS_BASE_TOKEN='由部署环境提供的URL安全令牌'
ros2 launch embodied_nexus_base base.launch.py \
  config_file:="$(ros2 pkg prefix embodied_nexus_base)/share/embodied_nexus_base/config/scout_waveshare.yaml" \
  interface_mode:=both
```

本次没有运行上述真实配置，也没有向机器人发送运动命令。来源和 Scout payload 字节序待实机核实事项见[驱动来源记录](scout-source.md)。

## ROS 接口

设备命名空间为 `/embodied_nexus/bases/<device_id>`，默认 ID 为 `base_main`。

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `state` | `embodied_nexus_base_interfaces/msg/BaseState` | 设备连接、控制权、硬件状态和反馈有效性 |
| `odom` | `nav_msgs/msg/Odometry` | 反馈速度积分所得的估算里程计 |
| `info`、`get_state`、`config/get` | services | 设备描述、状态与配置读取 |
| `control/acquire`、`control/renew`、`control/release` | services | 独占控制租约 |
| `motion/enable`、`velocity`、`stop` | services | 运动使能、租约保护的速度控制和停止 |
| `move_relative`、`turn_relative` | actions | 可取消直行距离和相对转角操作 |

速度单位为 m/s、rad/s，角度使用弧度。正线速度向前，正角速度逆时针。相对动作沿起始航向追踪里程，角度跨界时以角速度增量累计。速度请求超限会被拒绝，不会静默放大或截断。默认线速度为 `0.16 m/s`、角速度为 `0.60 rad/s`，配置上限为 `1.5`。里程计由速度积分估算；没有收到新鲜运动反馈时，状态中的实测速度和位姿有效标记会反映缺失。

默认租约为 2 秒，SDK 以约 0.5 秒续约。连续速度命令超过 0.5 秒未刷新会归零；租约失效、反馈陈旧、底盘故障或断连会停止并解除运动使能。每个相对 action 在成功、取消或失败后同步写零速度；停止响应分别报告停止请求和零速度写入结果，不能把写入成功视作硬件急停确认。

## HTTP、WebSocket 与 SDK

HTTP API 前缀为 `/api/v1/bases/{device_id}`，使用 `Authorization: Bearer <token>`。主要路径包括 `info`、`status`、`config`、`control/lease`、`motion/enabled`、`velocity`、`stop` 和 `operations`；持续动作支持 GET 查询与 DELETE 取消。`/ws?streams=status` 以 `en-base-v1` WebSocket 子协议传输带时间戳状态；第二个子协议为 `bearer.<token>`。

安装统一客户端后可以通过 SDK 控制。WASD 示例不导入 ROS 或设备驱动，只使用已安装的 `embodied-nexus-client`：

```bash
python -m pip install -e sdk/python
python src/base/examples/wasd_control.py
```

窗口填写底盘服务地址、设备 ID 和令牌，依次连接、申请控制权并显式使能。W/S 控制前后，A/D 控制旋转，组合键可同时输出线速度和角速度；松键会立即更新目标，空格或“停止”按钮会停止并解除使能。窗口失去焦点、连接失败、续约失败和退出时也请求停止。客户端默认请求速度 `0.16 m/s` 和 `0.60 rad/s`，滑块可降低速度；设备配置上限仍由服务端执行。

WASD 示例可能通过网络控制远端设备。此次只检查了代码和 mock/假串口，不运行真实底盘服务或键盘实机控制。
