# MA-USB8 → MaixCAM2 麦克风独立模块测试

更新：2026-10-21。入口是同目录 **`micarray_direction.py`**，在 MaixVision 打开该文件并点击运行即可。单文件无需模型或其他 Python 文件，不打开相机、YOLO、电机、热成像或 USB 音频。本次读取的是驱动板的声源热图，不是八通道原始录音。当前默认还会发送阈值和灯光配置，见 [CONFIG_CONTROL.md](CONFIG_CONTROL.md)。只做此前两线只读测试时，先把文件顶部 `CONFIG_ENABLED=False`。

## 最少接线

| MA-USB8 驱动板 | MaixCAM2 | 说明 |
| --- | --- | --- |
| TX0 | A31 / UART1_RX | 热图数据，必须接 |
| GND | GND | 独立供电也必须共地 |
| RX0 | A30 / UART1_TX | 默认配置控制必须接；仅在 CONFIG_ENABLED=False 时可不接 |
| 3V3 / RST / BOOT | 不接 | 本测试不使用 |

MA-USB8 通过自己的 USB-C 供电，相机使用原有供电，两块板的电源正极不互连。确认 TX0 是 3.3 V TTL，不把 5 V 信号送入相机 IO。

若使用之前的四芯滑环，默认配置控制需要共地、麦克风 TX0 和麦克风 RX0 三个通道，此次电机不连接。此前只读接法只需共地和 TX0 两个通道；要同时保留原来的两个电机信号，需要另行解决通道数量，不能并线。程序只打开 `/dev/ttyS1`，不会打开电机 UART3。首次建议短线直连，再串入滑环，以分离接触问题；断电核实通路，通道编号不代表物理针序。

## 运行与判断

1. 断电完成上述接线，确认麦克风阵列已正确连接驱动板，再分别给两块板供电。
2. 用 MaixVision 连接 MaixCAM2，停止之前的声音跟踪程序或其他 UART1 读取程序，避免争抢同一串口。
3. 打开本目录的 **`micarray_direction.py`**，点击运行。**不要运行 `MaixCam/tests/test_micarray_direction.py`**，后者是电脑端合成测试。
4. 控制台应出现 `MICARRAY_START` 和 `MICARRAY_UART_READY`。`READY` 只说明串口打开，不能证明已经收到麦克风信号。
5. 查看每秒的 `MICARRAY_DIAG`：先确认 `received_bytes` 增加，再确认 `frames` 持续增加；`frames_per_second` 是这段实际时间内的解析帧率，没有固定达标帧率承诺。
6. 在阵列有效拾音范围内，用一个持续声源（说话、哼声等），依次移动到不同方向。观察热图主区域和青色箭头是否随声源移动。先保持房间安静，避免两个接近强度的声源同时出现。
7. 静音或多声源时可以出现 `FRAME_NO_DIRECTION`；只要新帧仍增加，说明数据链路仍正常。需要检查断流时，停止输出或安全断开 TX0 信号，超过约 1 秒应出现 `UART_SILENT`，旧箭头和热图隐藏；恢复信号后自动等待新帧。不通过带电插拔整个电源线束做测试。
8. 用 MaixVision 停止按钮退出，程序释放 UART。保留从 `MICARRAY_START` 开始的日志，尤其是故障时连续几条 `MICARRAY_DIAG` 和完整异常堆栈。

显示初始化或绘制报错时，程序打印 `MICARRAY_DISPLAY_ERROR` / `MICARRAY_DRAW_ERROR` 并继续控制台诊断。也可以把文件顶部 `ENABLE_DISPLAY = False`，只查看串口与方向日志。

## 六种状态

| 状态 | 能证明什么 | 下一步 |
| --- | --- | --- |
| `WAIT_BYTES` | 尚无任何接收字节 | 查驱动板供电、TX0→A31、共地、线束对应和 UART1 占用 |
| `BYTES_NO_FRAME` | 有字节，但尚无新鲜完整热图 | 启动先等第二帧头；持续出现则查 2 Mbaud、输出模式、接触和 HEX |
| `FRAME_NO_DIRECTION` | 收到新鲜热图，但没有明确主声源 | 查看 `reason`，尝试单一持续声源，不直接判成串口故障 |
| `SOURCE_CENTER` | 主区域靠近热图中心 | 显示中心声源，不虚构唯一角度 |
| `DIRECTION_OK` | 有新鲜热图和可用图向量 | 用声源移动检查方向趋势，再检查安装旋转/镜像 |
| `UART_SILENT` | 接收过字节，随后超过有效期未再收到 | 查供电、信号线和滑环接触 |

`first_rx_hex` / `last_rx_hex` 各保留一个接收块开头最多 32 字节，不无限输出原始数据。`discarded_bytes` 是解析时跳过的字节数，**不是已知丢帧数或误码率**。协议没有 CRC，帧数增长不能证明数据完全无误。

## 方向与串口模式

- 图坐标定义：向上 0°、向右 90°、向下 180°、向左 270°。`MAP` / `map_direction` 表示热图坐标方向，**未标定为真实声学 DOA、云台角度、仰角或距离**。先检查不同声源位置是否产生稳定且可区分的图向量，之后再做声源位置与云台角度的实测标定。
- 若安装方向使图左右/上下不一致，可在文件顶部改 `ROTATION_DEG`（0/90/180/270，顺时针）和 `MIRROR_X`；热图与箭头同时变换。质量是区域显著性指标，不是正确概率。
- 本测试使用 2,000,000 baud、8N1，格式为 `16 × 0xFF + 256` 字节二进制热图。解析器需要下一个帧头确认边界，第一帧会等待第二帧头。默认阈值/灯光控制有独立的 `MICARRAY_CONFIG_*` 日志；`DIRECTION_OK` 不证明配置指令已执行。
- 如果之前打开了 ASCII、伪彩或调试模式，字节可能增加但不能解析热图。本程序不会自动发命令重置板子。可先重启驱动板恢复默认模式；若仍不恢复，用之前的板控工具检查 `f/c/d` 关闭文本/伪彩/调试输出，再重新测试。不要仅把相机改成 115200，驱动板 UART 固定 2 Mbaud。
- 两块板间实际电平、2 Mbaud 长时间稳定性以及滑环工作状态都仍需实机确认。

## 验证边界与依据

2026-10-21：19 项电脑端测试通过，包括合成流、方向/断流、配置文件保存/热更新/损坏恢复、指令节奏和模拟主入口。未连接实物，不能据此认定驱动板出信号、配置已执行或实际方向识别通过，也没有驱动板 Flash 掉电保存证据。

协议和接口依据：[MA-USB8 官方用户指南](https://wiki.sipeed.com/hardware/en/modules/micarray_usbboard_bl616.html)、[MaixPy UART](https://wiki.sipeed.com/maixpy/doc/en/peripheral/uart.html)、[MaixCAM2 管脚图](https://wiki.sipeed.com/hardware/assets/maixcam/maixcam2_pins.jpg)。
