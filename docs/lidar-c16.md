# C16 激光雷达适配

`embodied_nexus_lidar` 将 C16 点云和电机控制封装为独立设备服务。设备配置通过 YAML 选择 `c16`、`ros2_topics` 或 `mock` 后端。

## 构建与启动

机器人端使用 Ubuntu 22.04、ROS 2 Humble：

```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-angles libpcap-dev
cd ~/EmbodiedNexus
colcon build --symlink-install --packages-up-to embodied_nexus_lidar
source install/setup.bash
python3 src/embodied_nexus_lidar/test/validate_lidar_modes.py
```

该验收脚本只启动 mock 与已有话题后端，覆盖 `ros2/web/both`、HTTP/WebSocket 与 Python SDK；真实 C16 电机命令需使用下面的硬件步骤单独验证。

先检查 mock 配置，然后启动：

```bash
ros2 run embodied_nexus_lidar lidar_check --config src/embodied_nexus_lidar/config/c16_mock.yaml --show-config
ros2 launch embodied_nexus_lidar lidar.launch.py \
  config_file:=$(ros2 pkg prefix embodied_nexus_lidar)/share/embodied_nexus_lidar/config/c16_mock.yaml
```

真实 C16 使用随包配置。启动前会检查已知驱动进程及 UDP 端口占用；冲突时退出，不终止其他进程。驱动收到数据后才报告在线：

```bash
ros2 run embodied_nexus_lidar lidar_check --config src/embodied_nexus_lidar/config/c16.yaml --check-hardware
ros2 launch embodied_nexus_lidar lidar.launch.py
```

已有 ROS 2 点云来源使用 `c16_topics.yaml`。该后端不会控制外部雷达电机：

```bash
ros2 launch embodied_nexus_lidar lidar.launch.py \
  config_file:=$(ros2 pkg prefix embodied_nexus_lidar)/share/embodied_nexus_lidar/config/c16_topics.yaml
```

Web 模式需要认证令牌：

```bash
export EMBODIED_NEXUS_LIDAR_TOKEN='replace-with-a-local-secret'
ros2 launch embodied_nexus_lidar lidar.launch.py interface_mode:=both
```

默认设备 ID 为 `lidar_main`，Web 端口为 `8777`。`interface_mode` 为 `ros2`、`web` 或 `both`；`web` 模式仍通过隐藏的设备内 ROS 话题向独立网关传送数据。

## ROS 2 数据与控制

数据命名空间为 `/embodied_nexus/lidars/lidar_main`：

| 名称 | 类型 | 内容 |
|---|---|---|
| `/points` | `sensor_msgs/PointCloud2` | 原始 XYZ、强度、ring 和逐点 time |
| `/scan` | `sensor_msgs/LaserScan` | 按配置高度带从完整点云投影的水平扫描 |
| `/status` | `embodied_nexus_lidar_interfaces/LidarState` | 连接、数据年龄、频率、租约及电机状态 |
| `/control/acquire`、`/control/renew`、`/control/release` | 服务 | 独占电机控制权；有效期默认 2 秒，每 0.5 秒续约 |
| `/motor`、`/motor/speed` | 服务 | 启停电机及设置 5、10、20Hz |
| `/capture`、`/config/*` | 服务 | 启停本封装数据输出，检查和应用运行时配置 |

真实 C16 后端把厂商接收节点放在设备命名空间下，由设备包独占 MSOP/DIFOP 端口。DIFOP 报文经过校验后才可用于启停和调速。启动接收不发送启转命令。启转和调速需要控制租约；停转不需要租约。释放、租约超时和进程退出都会请求停转。退出和停转响应分别报告命令是否发出、硬件状态是否已确认；当前厂商驱动没有确认电机转态的独立反馈，所以真实硬件确认状态保持未知。DIFOP 中可识别的 5/10/20Hz 转速读回会另行报告。

默认 LaserScan 过滤高度为 `[-0.35, 0.35]m`，距离为 `[0.15, 18]m`，角度为 `[-pi, pi)`、1°分辨率；空桶用正无穷。点云和扫描坐标系沿用源 `frame_id`。设置 `filter.target_frame` 后，通过 TF 查询执行 XYZ 变换；变换缺失时该帧被拒绝并记录错误。点云过滤结果按原字段/点步长重新打包为一行数据；Web 订阅可额外配置 `point_stride` 降采样，ROS LaserScan 投影始终使用完整输入帧。

默认 ROS 输出为 `/embodied_nexus/lidars/lidar_main/points` 和 `/.../scan`。传感器话题使用 best-effort QoS。点云坐标单位为米；`time` 是厂商逐点时间字段，消息 header 使用厂商驱动给出的帧时间戳。没有启用外部授时时，不承诺时间戳与硬件时钟同步。

## HTTP、WebSocket 和 Python SDK

HTTP 前缀为 `/api/v1/lidars/lidar_main`，所有 HTTP 和 WebSocket 连接都需要同一令牌。SDK 保留原有包名：

```bash
pip install -e sdk/python
```

```python
from embodied_nexus_camera_client import LidarClient

lidar = LidarClient("http://robot-host:8777", token="local-secret")
print(lidar.status())
with lidar.acquire_control("inspection") as lease:
    print(lease.start_motor())
    print(lease.set_speed(10))
    for frame in lidar.subscribe(streams=("points", "scan"), point_stride=4):
        if frame.metadata["channel"] == "points":
            first_points = list(frame.points(limit=20))
            print(frame.metadata["frame_id"], first_points[:1])
            break
```

WebSocket `/ws` 使用 `en-lidar-v1` 及 `bearer.<token>` 子协议。每帧先发送 JSON 元数据，再发送原始二进制数据。点云二进制保持 `PointCloud2` 字段布局；扫描载荷为 little-endian float32 ranges，随后是 float32 intensities。每个客户端只保留最新帧；较慢的订阅者会跳过旧数据，并在状态中看到累计丢帧数。

`examples/lidar_websocket.html` 可在浏览器内连接 WebSocket，预览三维点云、二维扫描和实时状态。示例在本机处理令牌，仅通过 WebSocket 子协议发送。

## 配置与来源

`c16.yaml` 保存 C16 网络地址 `192.168.1.200`、接收端口 `2368/2369`、命令端口 `2368` 和 `lidar_link`。接收端口、网络地址、厂商解析方式和后端启动参数需要重启；扫描滤波、LaserScan 投影、超时和远程订阅频率支持运行时整批校验后应用。通过 `ros2 run embodied_nexus_lidar lidar_check` 执行配置检查、`--show-config`、`--diff-from` 和只读 `--check-hardware`。

随包厂商驱动来源和许可见 [lidar-source.md](lidar-source.md)。它在 `src/third_party/lslidar_c16_ros2` 中独立构建；设备服务关闭厂商未加控制权保护的原始控制服务，只保留点云与 DIFOP 接收路径。

## 验收

本次软件和真机验收记录见 [C16 hardware acceptance](lidar-hardware-acceptance-20261018.md)。

Humble 机器人端的短时真机电机验收命令：

```bash
python3 src/embodied_nexus_lidar/test/validate_c16_hardware.py --enable-motor-test
```

该脚本先读取并记录 DIFOP 原转速，再执行停转、启转及 5/10/20Hz 读回；结束时恢复原值、释放租约并请求停转，同时输出点云频率、数据年龄、带宽和丢帧。若设备停转后 DIFOP 不再报告可识别转速，可在本次验收已可靠记录先前读回值后，通过 `--recorded-original-speed-hz 10` 提供该恢复值；脚本会先写回并等待 DIFOP 确认，再允许启转。没有有效 DIFOP、已记录恢复值或转速读回时，会跳过启转/调速并报告待验收项。

自动化检查覆盖配置、DIFOP 型号/报文、协议帧、PointCloud2 布局、大端数据、点步长、过滤、扫描边界、租约、速度范围和故障注入。ROS 2 构建及 C16 网络采集、电机测试需在 Humble 机器人端完成。实机电机测试开始前读取 DIFOP 转速；无法识别原转速时不执行调速。测试后恢复原转速设置，然后按控制规则请求停转。
