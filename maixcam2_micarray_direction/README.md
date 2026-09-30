# MaixCAM2 声源方向显示（独立 MaixVision 程序）

在 MaixVision 打开 **[`micarray_direction.py`](micarray_direction.py)** 并点击运行。它只使用 MA-USB8 的 **TTL UART 16×16 声源热图**，在屏幕显示热图、主声源箭头、图像坐标角度、质量和帧状态。不调用相机、YOLO、Thermal160、USB 音频，也不发送板控命令。选择单文件入口，是为了让 MaixVision 直接运行时无需同步旁边的 Python 模块。

## 接线和串口选择

默认接法：MA-USB8 六针座的 **TX0 → MaixCAM2 A31（UART1_RX）**、**RX0 → A30（UART1_TX）**、**GND → GND**。MA-USB8 正面图以 USB-C 在右侧为准，左边六针座从上到下依次是 `GND, TX0, RX0, RST, BOOT, 3V3`；后三针不接到 MaixCAM2。MaixCAM2 正面管脚图以镜头朝向自己、Type-C 在左下为准：右侧 2×6 排针从左数第 3 列，上针 A31，下针 A30；最左列上下针均标 GND。参考[MA-USB8 官方正面图](https://wiki.sipeed.com/hardware/assets/modules/micarray_usbboard_bl616/product-front.png)与[MaixCAM2 官方管脚图](https://wiki.sipeed.com/hardware/assets/maixcam/maixcam2_pins.jpg)。

MA-USB8 用自己的 USB-C 接 5 V 电源供电；UART 三根信号线与 MaixCAM2 共地。**不要把两块板的 3V3 或 5V 针互连**。接线前核实 MA-USB8 TX0 空闲电平不高于 3.3 V；MaixCAM2 IO 不耐受 5 V。串口固定 2,000,000 bps、8N1；官方只明确验证 MaixCAM2 的 115200 bps，2 Mbaud 仍待板上持续收帧验证。

默认 UART1 独立于 Thermal160 常用的 UART2 B0/B1，也避开可能被 Maix 通信协议占用的 UART4 A21/A22。首次上板先在 MaixVision 运行同目录 [`connection_probe.py`](connection_probe.py)，确认 A30/A31 复用功能和 `/dev/ttyS1`，核对实际线束位置。`open_uart(..., configure_pins=False)` 只适合已在外部正确配置引脚的情形。

官方正面图能读出板上丝印，但实际线束颜色/插头朝向可能与图片不同；务必以板上的 `GND/TX0/RX0` 丝印为准。

## 界面与方向定义

- 屏幕左上 `MICARRAY DIRECTION`；中央 16×16 热图和方向箭头；下方显示 `MAP ANGLE`、质量、帧数、数据年龄及丢弃字节。MaixVision 控制台每约 5 秒输出 `MICARRAY_DIRECTION` 字典。
- `MAP ANGLE` 定义为热图坐标向上 0°、向右 90°，不是经过阵列声学标定的真实 DOA、距离或绝对罗盘方位。修改文件顶部 `ROTATION_DEG`（0/90/180/270）和 `MIRROR_X`，使阵列实际安装方向与屏幕一致。热图与箭头采用同一旋转/镜像。
- 背景过弱、两个声源权重接近、中心附近没有可靠方向或最后一帧超过 1 秒时，程序不显示旧箭头。多声源不自动假装只有一个目标。
- 程序不打开 8 通道音频。此前电脑端依赖音频和热图的自动阈值逻辑不包含在这里；仍可在后续扩展中通过串口手动发送 `t/T`，但本程序默认不发送任何命令。
- MA-USB8 UART 需处于默认**二进制**热图模式。如果此前手动发过 `F/C/D` 打开文本/伪彩/调试输出，请先恢复原模式或重新上电。帧为 16 字节 `0xFF` 头 + 256 字节数据；没有 CRC，解析器用下一帧头确认同步，因此首帧要等第二帧头到来才显示。

## 留给跟踪程序的接口

文件导入时不占用串口，也不初始化显示。以后将此文件以 `micarray_direction.py` 放在目标程序同目录，即可调用：

```python
from micarray_direction import SoundDirectionSensor

sound = SoundDirectionSensor(rotation=0, mirror=False)
sound.open_uart(port="/dev/ttyS1")  # A30/A31 接线正确且端口空闲时调用
try:
    while ...:
        sound.poll(timeout_ms=2)
        direction = sound.snapshot()
        if direction["valid"] and direction["unit_direction_xy"] is not None:
            x, y = direction["unit_direction_xy"]
            bearing = direction["image_bearing_deg"]
            # x 向右为正，y 向上为正；bearings 是热图坐标角。
finally:
    sound.close()
```

若跟踪程序已有自己的串口读取线程，则创建 `SoundDirectionSensor()` 但不调用 `open_uart()`，把收到的字节交给 `sound.feed_bytes(data, now)`，再读 `sound.snapshot(now)`。同一串口应只有**一个读取者**。`HotmapFrameParser`、`SoundDirectionEstimator` 和 `orient_xy` 也可分别导入作纯算法调用。

## 验证状态

本机运行：

```powershell
py -3 -B -m unittest discover -s maixcam2_micarray_direction -p test_micarray_direction.py -v
```

这验证帧边界、噪声后缓冲限制、方向、旋转/镜像、歧义、过期与无串口副作用。尚未在你的 MaixCAM2 上验证 MA-USB8 实际接线、电平、2 Mbaud 稳定性或屏幕效果。首次运行请保留控制台从 `MICARRAY_START` 开始的输出；若帧数持续为 0，应先检查供电、TX/RX 交叉、共地、独占串口和模块是否输出二进制热图。

依据：[Sipeed MA-USB8 用户指南](https://wiki.sipeed.com/hardware/en/modules/micarray_usbboard_bl616.html)、[MaixPy UART](https://wiki.sipeed.com/maixpy/doc/en/peripheral/uart.html)。声源主区域估计算法移植自 ENGG2202 本地 MicArray 方向程序；仓库版已单文件化，不依赖该本地目录。
