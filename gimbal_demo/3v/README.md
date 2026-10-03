# WHEELTEC F32C 双轴云台 Version 3

跨对话接手信息见 [信息缓存](../../docs/GIMBAL_CONTEXT.md)，最新零点功能交接见 [交接文档](../../docs/handoffs/gimbal-zero.md)。

本 demo 根据资料包中的 2026-07-18 二自由度云台手册、2026-07-21 F32C 电机手册、STM32/TI 位置与速度例程，以及 STM32F4 绘图例程整理。它负责两台电机和视觉闭环，不包含无人机检测模型，也不包含激光发射控制。

## 相比旧版本

- 保留 version2 的 GUI、实时角度与失能读取、速度/加速度、速度环 PID、状态查询、CLI 和 K230 跟踪入口。
- 修复 version2 的 X 轴 GUI 范围与文案不一致，以及 K230 默认 Y 跟踪范围超过 ±100°的问题；统一使用同一个限位异常类型。
- 对 version1 的 9 字节反馈超时，version3 会在使能前探测双轴反馈，支持重试和原始 RX 诊断。GUI 在无反馈或运行中反馈中断时保留串口，持续只读探测并展示状态；有反馈后由用户手动使能。电源、接线或电机 ID 错误仍需在设备上排查。

## 轴与协议

- X 轴：底部 yaw，默认电机 ID `1`。
- Y 轴：上部 pitch，默认电机 ID `2`。
- TTL 逻辑电平 `3.3 V`，串口 `115200-8-N-1`。
- K230 TX 接云台 RX，K230 RX 接云台 TX，信号地与电机电源地必须共地。
- 电机主电源 `8~15 V`，推荐 3S；不要从 K230 给电机主回路供电。
- 所有接口禁止热插拔；滑环电流能力有限，K230 供电需先核对实际峰值电流。
- 相邻协议帧至少间隔 `1 ms`。本 demo 默认 `2 ms`。
- 高频修改目标位置使用手册建议的“多圈位置直通模式”`0x0003`。

Y 轴相对所选零点的所有目标都限制在 `-100°～+100°`。GUI 默认使用电机固定机械零点，CLI 可用 `--reference mechanical` 选择；CLI 默认和独立 K230 示例保留旧的软件零点。视觉跟踪参数不会扩大驱动硬限位。真实垂直方向必须由人标定，见 [机械零点设置说明](MECHANICAL_ZERO.md)。

## 第一次运行

从仓库根目录进入本目录，先用协议模拟验证：

```powershell
cd gimbal_demo/3v
python demo.py --dry-run manual --x 15 --y 10
python demo.py --dry-run simulate --seconds 3
python -B -m unittest discover -s . -p 'test_*.py' -v
```

PC 通过 USB-TTL 控制时：

```powershell
python -m pip install -r requirements.txt
python demo.py --port COM5 probe
python demo.py --port COM5 --reference mechanical --speed 20 manual --x 5 --y 5
python gimbal_gui.py
```

首次带硬件测试请把云台固定牢，移除激光器和贵重负载，把角度限制在小范围。若方向相反，使用 `--x-sign -1` 或 `--y-sign 1`，不要交换电源线。

## 接入 K230 检测结果

PC/进程间可以向标准输入逐行发送检测框中心：

```powershell
'{"cx":400,"cy":220,"width":640,"height":480}' | python demo.py --port COM5 track-stdin
```

未检测到目标时发送：

```json
{"detected": false, "width": 640, "height": 480}
```

在 K230 上，按开发板的 pinmux 创建 `machine.UART` 后，将 YOLO/检测器输出的目标框中心接到 `k230_integration.run_tracking()`。检测回调的返回值为：

```python
(target_center_x, target_center_y, frame_width, frame_height)
```

没有目标则返回 `None`。如果同一帧有多个候选目标，应在检测侧先固定一个 track ID，避免控制器在目标间跳变。

## 控制流程

1. 发送一次唤醒字节并等待电机启动 1.5 秒。
2. 先读取双轴有效反馈；`software` 捕获当前位置，`mechanical` 用已保存的单圈机械零点映射到当前累计寄存器。无反馈时不使能。
3. GUI 连接后保持失能，由用户手动使能；CLI 运动命令自动保持当前位置再执行目标。GUI 使用 T 型轨迹；CLI 默认多圈位置直通模式。
4. K230 每帧给出检测框中心；控制器把像素误差按相机 FOV 换算为角度误差，经比例、像素死区和角速度限制后下发两轴目标。
5. Y 轴命令始终受相对所选零点 ±100° 的驱动限制。
6. 退出时发送两轴失能命令。

## 重要边界

- 代码只解决“检测结果到双轴角度”的闭环。检测模型、目标身份保持、云台/相机外参、镜头畸变和飞行目标提前量需要在实际系统中标定。
- 相机水平/垂直视场角默认 `70°/43°`，必须替换为实际镜头参数。
- 激光对眼睛、车辆和航空器均有严重风险。调试时仅使用低功率可见指示光并配合封闭式挡光靶；不要照射真实飞行器或天空。
