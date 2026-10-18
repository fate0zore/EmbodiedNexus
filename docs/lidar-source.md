# C16 驱动来源与许可

- 驱动：LeiShen `Lslidar_ROS2_driver` C16 ROS 2 V4，源仓库提交 `c73152cd373f7ce42475401cc4dd14f7c3b38b91`。
- 核对环境：Yahboom Jetson、Ubuntu 22.04、ROS 2 Humble；本机源代码位于 Arachne `third_party/Lslidar_ROS2_driver_C16_V4`。
- 适用型号：该驱动 README 列出 C16/C32 等型号；本适配仅声明和配置 C16。
- 上游目录工作区修改：CMake 补充 `-lpcap` 链接设置；C16 参数将帧设为 `lidar_link`、点云 topic 设为绝对路径。EmbodiedNexus 将设备地址、端口和 frame 改为包配置。
- 适配改动：关闭未通过 EmbodiedNexus 租约保护的厂商控制 ROS 服务；从有效 DIFOP 转发原始状态包；检查 UDP send 长度并关闭 socket；驱动停止时等待接收线程退出，并在扫描线程内同步发布点云。
- 许可：随驱动上游根目录提供 Apache License 2.0，原文保留在本目录 `LICENSE`。`lslidar_msgs` 的报文类型也随包保留，作为同一 Apache 2.0 驱动来源的一部分。
- 文件：上游 `UPSTREAM_README.md` 与许可证保存在 `src/third_party/lslidar_c16_ros2`。
