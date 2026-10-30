# MuJoCo 接入开发中

此目录目前只有共享 Arachne URDF 到 MuJoCo 碰撞模型的转换原型和控制映射测试，尚不是可运行的 ROS 包。转换使用共享模型的碰撞网格及基本形状作为可见几何，没有转换 DAE 可视网格；碰撞、关节控制和惯量需要进一步验证，当前不能作为 Gazebo 的替代品。

运行纯 Python 测试：

```bash
PYTHONPATH=src/sim/mujoco/embodied_nexus_sim_mujoco python3 -m unittest discover -s src/sim/mujoco/embodied_nexus_sim_mujoco/test -v
```

在已安装 `mujoco==3.2.7` 并 source ROS 2 Humble 的环境中，同一命令还会编译共享模型并执行短时物理步进。此检查不验证设备、传感器或完整场景行为。
