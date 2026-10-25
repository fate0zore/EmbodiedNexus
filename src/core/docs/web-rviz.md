# Core Web RViz

Core 运维面板提供只读的浏览器可视化页面：`http://<机器人主机>:8780/rviz/`。页面复用 Core 登录 token，通过 Core 代理读取 ROS 话题；Core 自行启动的 rosbridge 仅监听 `127.0.0.1:9090`。rosbridge 启动失败时，设备管理和其他 Core API 仍可使用，页面会显示桥接状态。

## 构建与启动

机器人端使用 Ubuntu 22.04 和 ROS 2 Humble。首次安装 rosbridge 和 Node.js 20+（只在构建前端时需要 Node.js）：

```bash
sudo apt install ros-humble-rosbridge-server
python3 -m pip install --user "starlette>=1.3.1" "uvicorn>=0.15" "websockets>=10.4"
```

在仓库根目录构建 Core：

```bash
source /opt/ros/humble/setup.bash
bash src/core/scripts/core.sh build core
source install/setup.bash
export EMBODIED_NEXUS_CORE_TOKEN='token'
ros2 run embodied_nexus_core nexus_core --config src/core/embodied_nexus_core/config/platform_mock.yaml
```

`core.sh build core` 会先以 npm 锁文件安装前端依赖并构建静态文件，再构建 `embodied_nexus_core` ROS 包。Windows PowerShell 使用 `src/core/scripts/core.ps1 build core`；该脚本在 Windows 上构建前端，并将 ROS 包构建委托给 WSL。运行时只需 ROS 2 和已构建的静态文件，不需要 Node.js 或外网 CDN。

打开 Core 首页并登录，然后点击“Web RViz”。token 保存在浏览器的 Core 登录存储中，WebSocket 和模型资源请求均通过同源、需认证的 Core 接口。可视化 WebSocket 只允许订阅及取消订阅支持的话题；发布、服务、参数和动作请求会被拒绝。

## 显示内容与数据来源

页面提供可旋转、平移和缩放的三维场景，可选择固定坐标系、显示 TF 与机器人模型、设置点大小和点数上限，并显示 RGB 与深度图像及话题状态。内置“现有设备”和“Gazebo Arachne”数据源预设；预设只是常用话题建议，所有话题均可手动调整。

Gazebo Arachne 预设使用 `/sim/arachne/robot_description`、`/sim/arachne/joint_states`、`/sim/arachne/lidar/points`、`/sim/arachne/camera/image_raw` 和 `/sim/arachne/camera/depth_image_raw`。其他设备读取其已发布的规范化 ROS 输出。`platform_real.yaml` 会由 Core 启动共享描述包中的 `robot_state_publisher`：它将 Arachne 组合 Xacro 发布到 `/robot_description`，并从 `/embodied_nexus/arms/arm_main/joint_states` 发布机械臂关节 TF 与固定安装 TF。实机 Scout 配置同时启用 `publish_tf`，由底盘节点发布 `odom → base_link`。Core 关闭时会清理自己启动的模型发布进程。模型网格通过 `embodied_nexus_sim_description` 包加载。

模型发布可通过 `visualization.robot_model.enabled` 开关控制，关节状态源可由 `visualization.robot_model.joint_states_topic` 指定。旧平台配置未设置该节时不会启动模型发布器；`platform_mock.yaml` 仍需由仿真或其他模型发布链提供描述与 TF。

仿真相机使用 Yahboom `config` 当前选中的安装外参和工厂内参，RGB 与深度分别配置取景位置及投影，分辨率改变时同步缩放 CameraInfo。仿真输出为理想去畸变图像；默认机械臂姿态下相机朝向地面，RGB 大片灰色与下沿出现夹爪属于该场景的实际观测。参数来源见 [Gazebo 指南](../../sim/gazebo/embodied_nexus_sim_gazebo/docs/gazebo-fortress.md)。

URDF 中的 `package://` 网格从 ament 安装的 ROS 包读取。模型包必须列入平台 YAML 的 `visualization.resource_packages`，资源接口会检查 Core token、包许可和解析后的文件路径。默认许可列表包含共享仿真描述包及设备包；缺少安装包或资源时页面会显示资源加载错误，不复制仓库模型文件，也不依赖源码相对路径。

支持的可视化话题包括 URDF 字符串、TF、PointCloud2、LaserScan、Image 和 JointState。雷达与相机点云可同时显示，分别选择话题、开关图层；相机点云固定使用洋红色以便与雷达区分，雷达点云保留消息中的 RGB 或 intensity 色彩。点云读取字段偏移、点步长、行跨度和大小端，支持 XYZ、RGB/RGBA 或 intensity，跳过无效坐标并按点数上限采样。图像支持 RGB/BGR/RGBA/BGRA、mono8、16UC1/mono16 和 32FC1；深度图使用米制色阶，可调显示范围。

三维场景按需绘制，上限为 30 FPS，没有新数据或视角变化时不持续重绘；像素倍率上限为 1.25。关闭点云图层时取消对应订阅，页面隐藏或离开时断开全部订阅，返回后自动重新连接。图像与点云默认限频至 5 Hz，每个话题只保留最新缓冲帧；浏览器 Worker 解码 CBOR 并转换点云和图像，通过可转移缓冲区交给主线程。TF、模型及关节与传感器使用独立连接；TF 变换图按时间戳缓存，点云复用 GPU 缓冲，默认最多绘制 100,000 点。传感器点云与激光扫描按消息时间查询 TF；没有可用变换时会隐藏并暂停对应图层，收到可用 TF 后自动恢复。仿真时间回跳时会清理动态 TF 缓存。静态 TF 和模型描述使用 transient-local 订阅策略；传感器采用适合传感器数据的 QoS。

本机 Gazebo 默认使用 `balanced` 档：320×240 RGB-D、相机与雷达均为 5 Hz、关闭阴影；相机内参和设备捕获配置自动匹配分辨率。与原来的 640×480、10 Hz 相比，原始 RGB-D 数据量降为八分之一。需要原分辨率与频率时，使用 `ros2 launch embodied_nexus_sim_gazebo sim.launch.py performance:=full`，或设置 `EMBODIED_NEXUS_SIM_PERFORMANCE=full` 后调用 Gazebo Bash/PowerShell 脚本。物理步长、控制频率和雷达扫描覆盖范围保持原配置。

Humble 仓库中的 rosbridge 2.0.8 会根据已发现的 ROS 发布者推断兼容 QoS；协议显式 `qos` 字段从 2.1.0 起提供。Core 会读取已安装的 `rosbridge_library` 版本，在支持时发送显式 QoS；2.0.8 则使用桥接自身的发布者兼容推断逻辑。

## 平台配置

平台 YAML 可省略根级 `visualization` 节；省略时默认启用 Web RViz、使用端口 9090 和默认模型包许可列表，模型发布默认关闭。设置 `enabled: false` 可停用 rosbridge，而不影响 Core 其他功能。

```yaml
visualization:
  enabled: true
  port: 9090
  robot_model:
    enabled: true
    joint_states_topic: /embodied_nexus/arms/arm_main/joint_states
  resource_packages:
    - embodied_nexus_sim_description
    - embodied_nexus_arm
```

## 排查

- 页面提示 rosbridge 不可用：检查已安装 `ros-humble-rosbridge-server`、Core 启动日志和 `127.0.0.1:9090` 端口占用情况。
- 话题列表为空或没有数据：确认对应设备/仿真已启动，并在 ROS 2 环境中检查话题名称、类型和 QoS。Gazebo 首次启动时，等待 Arachne 模型生成后再连接 Gazebo 传感器桥接；页面数据源选“Gazebo Arachne”。
- 模型只有坐标轴或提示等待 TF：确认 URDF、`/tf`、`/tf_static` 和 `/joint_states`（或预设话题）正在发布，且固定坐标系与机器人根链接连通。
- 网格或纹理加载失败：确认对应 ROS 描述包已安装且列入 `resource_packages`，并检查 URDF/DAE 中的资源路径。
- RGB 或深度显示“不支持”：确认话题类型为 `sensor_msgs/msg/Image`，编码属于页面支持范围。

Web RViz 仅用于查看 ROS 数据，不包含导航交互、设备控制或 SDK 接口，也不替代 ROS 2 桌面版 RViz。
