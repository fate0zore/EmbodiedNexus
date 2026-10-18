# Yahboom Gemini335 相机实机验证记录

- 日期：2026-10-18
- 状态：第一期实机验证记录
- 参考 Arachne 提交：`41caa56`
- 验证范围：相机适配器独立部署在 Yahboom 机器人；没有启动或修改 Arachne 全栈，也没有运行机械臂。
- ROS 版本：ROS 2 Humble；没有 ROS 1 兼容层。

## 设备和运行环境

通过现有 SSH 连接在机器人本体构建并启动 `EmbodiedNexus` 相机包。机器人枚举到 Gemini335 的彩色设备和深度设备：彩色流由稳定的 `/dev/v4l/by-id/...Gemini_335...-video-index0` 路径打开，映射到 `/dev/video6`；Z16 深度流由 `/dev/video0` 打开。两路均支持 640×480。彩色驱动以 30 FPS 运行；深度请求 5 FPS 时驱动实际协商为 6 FPS。

每次独立采集测试前后检查了 V4L2 占用。没有发现已有进程占用时才启动新采集器；停止后两个设备均释放。没有通过脚本终止其他相机进程。

检查了两路设备的 V4L2 控件。彩色设备提供 `auto_exposure`、`exposure_time_absolute` 和 `gain`；深度设备提供 `exposure_time_absolute` 和 `gain`。配置支持按设备设置数值型控件，但本次验证未更改相机曝光或增益。

机器人端构建通过：

```bash
source /opt/ros/humble/setup.bash
cd ~/EmbodiedNexus
colcon build --symlink-install --packages-up-to embodied_nexus_camera
```

## 接口模式

| 模式 | 实机结果 |
|---|---|
| `web` | HTTP 状态与快照可用，WebSocket 能传输 RGB、深度和点云；没有对外创建相机 ROS 发布话题。停止后设备释放。 |
| `ros2` | ROS 2 图像、CameraInfo、点云、状态、配置服务及标定 action 可见；没有启动 Web 服务。传感器话题使用 best-effort QoS，订阅端需兼容。 |
| `both` | ROS 2 和 Web 接口同时可用；共用同一个采集实例，没有重复打开设备。 |

Web 接口使用 bearer token。未认证 HTTP 请求返回 `401`。从局域网 Windows 主机使用仓库 Python SDK 连到机器人，`status()` 返回在线；订阅连续收到 RGB JPEG（640×480，约 28 KB）和无损 16 位深度 PNG（640×480，约 137 KB）。SDK 不需要安装 ROS 2。

另以模拟的 Arachne ROS 2 图像话题启动 `ros2_topics` 输入后端，向输入话题连续发布了 15 组 RGB/深度消息。网关状态报告两路在线、分辨率 640×480，快照返回有效 JPEG。该模式期间 `/dev/video0` 和 `/dev/video6` 均无占用，证明网关可复用现有 ROS 2 图像流而不打开相机。测试中的中间帧出现一次超过 50 ms 配对阈值的点云跳过状态，符合配置的时间配对保护规则。

补齐 ROI、V4L2 控件入口和内参分辨率策略后，再次以 `both` 模式实开 Gemini335：RGB、深度和点云均在线，工厂标定报告可用于点云；640×480 快照成功，点云有 12,204 个点。设备只在采集进程运行期间被占用，节点退出干净，之后设备释放。

## 数据和点云

- 真实彩色画面和深度均通过 WebSocket 收到。深度数据按 `16UC1` 原始毫米值无损编码；这次现场画面的非零深度中位数约 905 mm。
- 深度生成的 XYZRGB 点云包含 12,208 个点，坐标为米，点云有限值范围约为 `x=[-0.757, 0.752]`、`y=[-0.626, 0.378]`、`z=[0.148, 1.209]`。画面和颜色投影可见，输出坐标系为深度光学坐标系。
- 点云深度 ROI 可用 `[x, y, width, height]` 按深度图像像素裁剪，空列表使用整幅图；ROI 越界会报错。内参分辨率不匹配时默认关闭几何处理，只有显式设置 `calibration.allow_resolution_scaling: true` 才允许等比例缩放内参。
- 另外向 `ros2_topic` 点云输入发布了 XYZ-only `PointCloud2`，输出保留原 XYZ 并增加 `rgb` 字段；已有带颜色点云按原字段布局转发。颜色需要可用 RGB 标定、有效时间配对和坐标到 RGB 光学系的变换。
- 输入源为 `ros2_topics` 时，彩色/深度图像转发已验证；现有 ROS 点云接入和着色条件也单独验证过。缺少有效配对或 TF 时会转发原始 XYZ（若输入本身可读）并报告着色原因，不会伪造颜色或目标坐标系。

## 配置和恢复

在机器人上调用 ROS 2 配置服务验证了以下行为：

- 查询返回当前有效模式 `ros2` 和 JPEG 质量 `85`。
- 校验 `jpeg_quality=72` 成功但不改变当前配置；显式应用后返回配置版本 `2`。
- 在线修改 `capture.color_fps` 被拒绝，并报告需要重启。
- 以信号停止相机节点后正常退出，采集线程结束，V4L2 设备释放。

标定文件加载了从 Arachne 配置整理出的 Gemini335 工厂参考参数：保留彩色和深度的完整畸变系数、深度尺度与 RGB-D 外参；没有将 Arachne 相机采集进程或机械臂依赖复制进新包。

## 实测吞吐

默认配置为 RGB 30 FPS、深度 5 FPS、点云 5 FPS。实测 ROS 2 订阅端在机器人本机收到约 14.5–15.5 FPS 彩色、2.8 FPS 深度和 2.4–2.5 FPS 点云；Web-only 模式的相机数据缓存约为 30.9–31.9 FPS 彩色、6 FPS 深度和 2.8 FPS 点云。以上是本次机器人上的测量值，不是对其他负载或网络的性能保证。深度帧率是驱动的 6 FPS 离散档位。输出频率和点云计算仍可通过 YAML 配置调整。

## 自动化检查

- 本地相机核心单元测试：17 项通过，覆盖配置校验、缓存与 WebSocket 帧、内参/畸变模型、深度反投影、ROI、点云单位与 RGB 投影、时间配对、样本删除和运行时配置限制。
- Python 源码编译检查通过。
- 机器人端 `colcon build --packages-up-to embodied_nexus_camera` 通过，包含相机包和 ROS 2 接口包。

## 尚未现场完成的项目

- 本次机器人上没有可用的 AprilTag 标定板参考图或实体标定板，因此没有现场采集样本、运行 RGB 内参求解或应用新标定。RGB 标定的 API、求解 action、候选结果与历史管理已实现，但现场求解精度和标定板检测成功率仍待补测。当前点云使用已存在的工厂标定参数。
- 用户在本机浏览器通过 `examples/camera_websocket.html` 连接机器人 WebSocket，采样预览正常，确认通过。局域网 Python SDK 的 WebSocket 和图像解码也已实测通过。
- 本次没有对长期运行、持续数小时温升、拥塞网络下的带宽和多客户端压力作验收。
