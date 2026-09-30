# K230 / CanMV 接入说明

K230 固件常见的是 MicroPython/CanMV 环境，请使用独立文件 `k230_f32c_demo.py`；它不依赖 `pyserial`、`dataclasses` 或 CPython 类型注解。`k230_integration.py` 适合运行完整 CPython 的 K230 Linux 系统，不用于精简的 CanMV 固件。

先按具体 K230 开发板的手册完成 UART pinmux，再建立 `115200-8-N-1` UART。不同开发板的 UART 编号和引脚不同，不能直接照抄引脚号。典型调用结构如下：

```python
from machine import UART
from k230_f32c_demo import run_tracking

uart = UART(UART.UART1, 115200, bits=8, parity=None, stop=1, timeout=20)

def detect_drone_center():
    # 用实际 K230 检测器替换此处。
    # 有目标：return (cx, cy, image_width, image_height)
    # 无目标：return None
    return None

run_tracking(uart, detect_drone_center, y_dead_zone_deg=160.0)
```

如果图像右侧目标导致云台向左转，修改 `K230Tracker` 的 `x_sign` 为 `-1`；上下方向相反则修改 `y_sign`。首次通电应移除危险负载，并用小角度、低速验证方向和 Y 轴机械死角。
