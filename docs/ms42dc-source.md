# MS42DC 驱动来源记录

- 项目：本地工作区 Arachne
- 实际读取的 HEAD：`170ce0bf713bd1a6626677e70b3dc12293858147`
- 总体规划记录的参考提交：`41caa56`，该对象在本地 Arachne 仓库不可解析
- 直连驱动：`src/arachne_hardware/arachne_hardware/ms42dc_direct_serial_driver.py`
- 厂商参考驱动：`third_party/ms42dc_step_motor_ros2/Step_Motor_ROS2/src/step_motor/src/motor_node.cpp`
- 许可：Arachne 仓库根目录 `LICENSE` 声明 MIT License；协议辅助逻辑基于该项目实现并按 MIT 条款分发。

随包保留的许可文本位于 `third_party/licenses/arachne-MIT-LICENSE.txt`。

抽取范围限于串口命令/查询帧和反馈字段解析。新代码独立实现 ROS 2 生命周期、配置、租约、网关及 Python SDK。`reached=0` 运动中、`reached=1` 到位的语义已于 2026-10-18 在 Yahboom/MS42DC 上通过闭合和打开动作实测；该结论仅限本次设备/固件组合。
