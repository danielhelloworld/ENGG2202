# 实时状态与 Y 轴限位

## Y 轴机械限位

当前 `demo.py` 使用以下定义：

- 软件零点 `0°`：相机垂直于地面时的 Y 轴位置；
- 允许范围：相对软件零点 `-100°～+100°`；
- 禁止范围：上述区间之外的所有位置。

在圆周死区表达中，这等价于“以 180°为中心、宽 160°的禁止区”，因此命令行默认参数是：

```text
--y-dead-width 160
--y-dead-center 180
--y-dead-margin 0
```

例如请求 `--y 150` 时，程序只会下发 `+100°`。

## 状态参数地址

`motor_status.py` 中的 `read_motor_status()` 是独立状态读取函数。这里的参数地址是 F32C 返回帧第 3 字节的“反馈类型”，不是用于发送控制帧的电机 ID：

| 参数地址 | 状态 | 换算 |
|---|---|---|
| `0x00` | 实时速度 | RPM |
| `0x01` | 多圈累计角度 | 原始值 × 0.1° |
| `0x02` | 单圈机械角度 | 原始值 × 0.1° |
| `0x03` | 加速度 | 保留原始值 |
| `0x04` | 母线电压 | 原始值 × 0.01V |

默认运行时以 2Hz 查询速度、累计角度和母线电压：

```powershell
python demo.py --port COM5 manual --x 5 --y 10 --status-rate 2
```

查询 `0x00～0x04` 全部状态：

```powershell
python demo.py --port COM5 manual --x 5 --y 10 --status-rate 2 --status-all
```

关闭状态输出：

```powershell
python demo.py --port COM5 manual --x 5 --y 10 --status-rate 0
```

典型输出：

```text
STATUS X[motor=0x01 0x00=0rpm(0) 0x01=5deg(50) 0x04=12V(1200)] |
       Y[motor=0x02 0x00=0rpm(0) 0x01=10deg(100) 0x04=12V(1200)]
```

其中 `motor=0x01/0x02` 是响应电机地址；`0x00/0x01/0x04` 才是对应的状态参数地址。

