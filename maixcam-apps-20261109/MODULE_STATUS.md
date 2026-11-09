# 模块进度与已知问题

资料复核于 **2026-11-09**。对应应用快照提交 `4b550ef`，共 7 个应用、222 个原文件。以下以本地维护源码、随模块附带的测试记录和已保存的实验报告为依据。板上状态均注明对应版本或试验，当前设备状态未复测。

| 模块 | 快照内版本或入口 | 已达到的验证阶段 | 当前主要缺口 |
| --- | --- | --- | --- |
| YOLO 点选跟踪 | `tight-velocity-v43` | v43 本机 307 项及网页测试通过。v42 完成双轴方向和自动回零实测 | v43 匀速跟随与急停对比，pitch 串口失联复查，真实无人机跟踪 |
| F32C 独立电机控制 | `main.py`、`main_web.py` | 本机 108 项及网页测试通过。另有逐轴串口响应试验 | PI 写入与回读差异，自动整定实机复测 |
| 无人机三传感器融合 | 2026-11-06 热像整合版 `main.py` | 本机 28 项通过，含合成几何、热像桥接和模拟板端 | 三路实机、标定、专用模型和目标距离验证 |
| Thermal160 无人机候选检测 | `board_main.py`、`replay.py` | 本机 6 项通过，12 帧合成回放，单文件构建检查 | 真实热帧、板上算力、误报与检测距离 |
| Thermal160 与 YOLO 叠加 | v1.5.0 `main.py` | 历史本机融合、校准与推流验证，FFmpeg 实际解码合成帧 | 当前板端取流、颜色、触摸和测温精度 |
| MA-USB8 声源方向 | `micarray_direction.py` | 历史本机 19 项通过，含配置热加载及协议模型 | UART1 连续收帧、实测方向、阈值与灯光生效 |
| 声音寻向与 YOLO 云台 | `main.py`、`build.py` | 维护源码曾通过 51 项本机测试及合成流程 | 快照 `main.py` 有语法错误，集成链路待上板 |

## YOLO 点选跟踪

- 相机本地运行 YOLO、ByteTrack、点选锁定、姿态补偿、预测和电机控制。网页提供点选、摇杆与状态显示。速度驱动下发有符号 RPM，支持连续 yaw、有限时长漏检续跟与高仰角换向。pitch 工作范围为 −100°～+30°，跟踪目标最低 −90°。 [运行说明](yolo_click_tracker/README.md) · [速度驱动](yolo_click_tracker/VELOCITY_DRIVE.md)
- v42 实机验证两轴速度方向修正。一次自动 HOME 从 yaw −7.2°、pitch 118.0°回到约 0.9°、0.0°，进入 `HOME_HOLD`。用户随后反馈速度控制跟得更快，匀速移动仍有落后。最后一次设备观察到 pitch ID 2 连续 500 ms 无有效回复后失能，串口或供电原因尚待核对。 [方向配置与回零实现](yolo_click_tracker/src/velocity_motor.py)
- v43 在连续同向移动时加入有界积分，停止、换向或观测失效时清除补偿。合成对比显示部分工况的匀速误差下降，同时可能增加急停过冲。v43 的真实跟随误差、急停、长期温升及无人机过顶效果仍待验证。 [速度驱动测试](yolo_click_tracker/tests/test_velocity_drive.py) · [研究状态](RESEARCH_STATUS.md#速度控制与跟随延迟)

## F32C 独立电机控制

- 触屏和网页摇杆提供机械角反馈、使能保持、HOME、重新检查和有界轨迹。网页入口支持反馈记录、逐轮对比和基于已知基线的单轴 PI 自动整定。普通触屏版与网页调试版的启动配置应分别查看。 [运行说明](motor_joystick/README.md) · [网页控制](motor_joystick/WEB_CONTROL.md) · [PI 调参](motor_joystick/PID_TUNING.md)
- 历史实机证据包括独立串口 yaw 候选试验和 pitch ±10°候选、±15°保持检查。yaw 最后发送速度环 24/1，厂家界面回读 23/0。pitch 最后发送 10/9，回读 10/8。差异原因未确认，记录值对应各次试验，实际固件增益与掉电保存仍待核对。 [Yaw 试验脚本](motor_joystick/yaw_serial_tune.py) · [Pitch 试验脚本](motor_joystick/pitch_serial_tune.py)
- 现有保护依据编码器、方向、限位和通信状态。真实负载下的堵转、力矩和高频振动缺少测量。v43 的速度控制属于跟踪模块，旧位置控制的软件整定不能直接用于它。 [整定流程与边界](yolo_click_tracker/AUTOTUNE.md)

## 无人机三传感器融合

- 桌面入口演示融合策略，板端单文件连接 RGB YOLO、声源热图和 Thermal160。热像检测与显示共用一份有效标定和一个读取器，权重默认视觉 70%、声音 15%、热像 15%。声热可提供候选与辅助证据，`VISUAL_CONFIRMED` 需要有效视觉证据。 [融合说明](drone_sensor_demo/README.md) · [整合记录](drone_sensor_demo/INTEGRATION.md)
- 默认板端模型用于检查视觉链路，实际无人机确认需配套训练后的 `.mud` 和 `.axmodel`。热像映射还需检查分辨率、帧率、方向和距离档位，声音映射需按实际安装标定。固定相机是当前融合条件，转动时缺少姿态补偿。5/10 m 确认和 30–100 m 候选发现仍是研究目标。 [融合核心](drone_sensor_demo/core.py) · [整合测试](drone_sensor_demo/test_thermal_integration.py)

## Thermal160 无人机候选检测

- 多尺度局部对比可保留单像素候选，按新有效帧做三帧确认，短时丢失可保持轨迹。重复、过期、温度无效和预测帧不增加融合证据。支持坏点屏蔽，广域变化时降级为 `MOTION_UNCERTAIN`。 [算法与接口](thermal_uav_detector/README.md) · [合成回放结果](thermal_uav_detector/synthetic_results.jsonl)
- 板上入口已生成。固定热坏点、云边、建筑边缘、移动背景与目标交叉仍可能造成误报或关联中断。真实标注帧、板上耗时、每分钟误报率和各距离召回率均待采集。 [验证报告](thermal_uav_detector/TEST_REPORT.md) · [融合交接](thermal_uav_detector/HANDOFF.md)

## Thermal160 与 YOLO 叠加

- 已实现热像伪彩、RGB 检测框、原始热像像素测温、独立热块点选、手动多距离校准和 RTSP/TCP JPEG。屏幕与推流共用合成帧。v1.5.0 加入 RGB 色彩路径修正及 `TEMP`、`HEAT` 开关。 [运行说明](thermal_yolo_fusion/README.md) · [校准流程](thermal_yolo_fusion/CALIBRATION.md) · [本机 RTSP 解码记录](thermal_yolo_fusion/validation/rtsp_decode.json)
- 历史子进程版曾加载模型后停在首帧并显示 `NO THERMAL`，原因尚未确定。当前默认同进程间隔推理，单次推理仍会阻塞显示。需要核对 telemetry30 固件、持续取流、设备颜色、真实点对误差及测温参照。自动距离选择、逐深度对齐和独立热像 YOLO 仍处于设计阶段。 [性能与诊断](thermal_yolo_fusion/PERFORMANCE.md) · [动态对齐研究](thermal_yolo_fusion/DYNAMIC_ALIGNMENT_AND_THERMAL_YOLO.md)

## MA-USB8 声源方向

- 单文件解析 UART1 的 16×16 热图，提取主连通区域并显示方向与诊断状态。支持安装旋转、镜像、静音与断流识别，以及阈值和阵列灯请求的相机配置持久化。 [模块测试说明](micarray_direction/MODULE_TEST.md) · [配置接口](micarray_direction/CONFIG_CONTROL.md)
- 当前方向量处于热图坐标系，物理声源方位仍需标定。TTL 接入只获得热图，八通道音频需要独立 USB 音频路径。阈值和灯光命令没有公开 ACK 或回读，串口发送记录仅能证明请求已下发。滑环通道、电平、2 Mbaud 稳定性和现场反射需验证。 [接线与诊断](micarray_direction/README.md)

## 声音寻向与 YOLO 云台

- 源码包含声音触发、绝对方位转向、小范围搜寻、视觉锁定和短时丢失保持。固定机械零点及参考丢失保护曾用假串口验证。维护入口为 `src/` 和 `build.py`。 [集成说明](sound_yolo_gimbal/README.md) · [历史验证](sound_yolo_gimbal/VALIDATION.md)
- **快照运行入口目前阻塞。** `main.py` 第 201 行字符串未闭合，生成入口与维护源码也存在配置差异。历史 51 项通过记录对应先前源码和生成状态。恢复时需先比较现有配置与重建结果，再验证声音到机械方位的映射及整条链路。 [出错入口](sound_yolo_gimbal/main.py#L201) · [生成脚本](sound_yolo_gimbal/build.py)

## 测试口径

本次资料复核确认本地应用与快照逐文件 SHA-256 一致。前次发布时在快照副本运行的 **449 项 Python 测试**涵盖 YOLO 307 项、电机 108 项、三传感器 28 项、独立热像 6 项，另有两套网页测试。麦克风和热像叠加的历史本机记录包含应用目录之外的测试。板上证据按上文对应版本查看。 [原文件清单](snapshot-manifest.json) · [跨模块研究状态](RESEARCH_STATUS.md)
