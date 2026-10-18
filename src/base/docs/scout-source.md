# Scout 2.0 驱动来源

- 参考项目：机器人 Arachne 仓库 `/home/jetson/zhaoyang/Arachne`。
- 读取的提交：`41caa564204eb5f6a39200ce2600ed052ab517b0`。
- 来源文件：`src/arachne_hardware/arachne_hardware/scout_waveshare_serial_driver.py`（Arachne 仓库路径）。
- 许可：Arachne 根目录 `LICENSE` 声明 MIT；许可副本见 [arachne-MIT-LICENSE.txt](../third_party/licenses/arachne-MIT-LICENSE.txt)。
- 改写范围：Waveshare USB-CAN-A 可变帧打包/拆包、Scout 帧标识、速度单位换算和系统/运动反馈字段。ROS 节点、话题、TF、任务服务和面板逻辑没有复制。
- 适配参数：USB 串口默认 2,000,000 baud、CAN 500,000 bit/s、标准帧、CH340 稳定设备路径。

微雪 USB-CAN-A 串口帧边界和标准帧格式参照[官方串口转 CAN 协议说明](https://files.waveshare.com/wiki/USB-CAN-A/Manual/USB-Serial-CAN-protocol-defines-CN.pdf)；Scout 消息 ID 和字段定义与 [AgileX UGV SDK](https://github.com/agilexrobotics/ugv_sdk/blob/main/src/protocol_v2/agilex_msg_parser_v2.c)交叉核对。本适配沿用 Arachne 已部署 Waveshare 驱动的 payload 字节顺序。AgileX SDK 当前解析实现使用的小端字段顺序与 Arachne 驱动不同；由于本次明确不进行实机运动验证，这一差异需在后续设备验收中核实，不能视为已确认的 Scout 固件事实。

里程计以新鲜反馈的线速度和角速度在服务内积分得到，是短时相对运动估算值，不代表底盘提供绝对位置或经过校准的里程计。硬件控制模式只在持有有效租约且显式使能时请求切换。默认 mock 配置不打开串口。
