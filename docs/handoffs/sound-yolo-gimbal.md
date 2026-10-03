# 声音寻向与 YOLO 云台系统交接

- 日期：2026-10-03。
- 分支：`feature/maixcam2-micarray-direction`。
- 工作起点：`4a64912`。
- 目录：`maixcam2_sound_yolo_gimbal/`；完整代码自包含，构建和测试不依赖仓库外文件。

## 已完成

- 固定底座 MicArray 声音方向查表、稳定性确认、F32C 到位反馈、YOLO + 原生 ByteTrack 连续 ID 锁定、触屏选择与丢失后重新寻向。
- 以用户指定的 `Daniel's code/mainweb.py` 行为为依据；默认使用板上 YOLO11 模型，模型权重与原 YOLOv8 原型不同，详见 README。
- 固定机械零点每次连接重建累计控制参考，连接保持失能，ARM 时预装当前位置；Y 软件限位相对真实垂直零点 ±100°。
- 独立 `calibrate_zero.py` 标定入口：明确确认、静止检查、0x0A 置零、回读通过后 0x08 保存。
- 停滞／反向运动保护；零点无效、过期、参考不一致、累计角突变后锁定故障、清空运动目标并尝试两轴失能。发送位置／使能前再次检查参考，反馈恢复不自动恢复运动。
- 已提供中文 README、标定 CSV 工具、PC 合成演示、源文件哈希和验证记录。

## 验证及边界

- 本机 51 项测试通过，完整合成流程通过。全部使用 PC 合成数据与假串口。
- 默认 `motor_enabled=False`；同步、构建与测试不启动真实电机。
- 尚未完成 MaixCAM2 UART/NPU/触屏和实际电机联调；机械零点掉电保存、真实限位与方向、保护阈值需实机验证。
- 软件保护不测力度，不能保证检测所有 Flash 零点丢失或送达断线总线上的失能命令；未实现硬件切电。
- 原系统维护位置为本机 MaixCam 应用目录，仓库副本是此次授权同步的发布目录。之后改动需明确维护哪一份，并重新构建后再同步。
- 最新机械参考依据来自本机云台独立 worktree；本次同步其在跟踪器中的适配实现，PC 云台 GUI/CLI 的未提交改动不包含在本提交中。

## 下一步

从仓库根目录运行：

```powershell
python -B maixcam2_sound_yolo_gimbal/build.py
python -B -m unittest discover -s maixcam2_sound_yolo_gimbal/tests -q
python -B maixcam2_sound_yolo_gimbal/simulate.py
```

上板先读该目录 README，用 MaixVision 打开 main.py；核对模型和 UART，完成机械零点与声音标定后再配置真实运行、手动 ARM。calibrate_zero.py 的确认默认关闭。
