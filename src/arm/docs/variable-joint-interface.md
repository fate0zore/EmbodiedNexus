# 机械臂可变关节接口迁移

机械臂 `MoveArmJoints` action 的 `positions_rad` 和 `SetArmJointVelocity` service 的 `velocity_rad_sec`、`commanded_positions_rad` 使用变长数组。调用方应先读取设备配置中的 `joints.names`，按其顺序提交相同数量的弧度或弧度每秒数值；服务会校验数量、有限值和已配置的限制。

这改变了 ROS 接口类型：需要重新构建接口包及所有依赖包，并重新生成和部署使用这些消息的客户端。Python SDK 的输入仍为列表，调用方应根据设备关节数构造列表。现有 Aubo i5 配置要求六关节；其他型号通过 `embodied_nexus_arm.models` 注册提供方，负责校验其关节数量并提供后端与操作实现。现有 Gazebo 机器人模型仍为六关节；四关节测试仅使用测试提供方，不表示已接入四关节物理型号。

模型操作的 RPC 映射位于 `embodied_nexus_arm/adapters/aubo_i5/`；Aubo 仿真后端按型号注册为 `aubo_i5.ros2_sim`。新型号应分别实现自己的设备和仿真后端，并遵守公共设备服务的租约、停止与反馈契约。
