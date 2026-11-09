# MaixCAM 应用完整快照 · 2026-11-09

本目录收录本地 `MaixCam/apps` 的全部 **222 个文件**，共 **7 个应用**。目录结构和原始字节均保留。每个文件的大小与 SHA-256 见 [快照清单](snapshot-manifest.json)。

| 应用 | 文件数 | 入口或说明 |
| --- | ---: | --- |
| YOLO 点选跟踪 | 97 | [main.py](yolo_click_tracker/main.py) · `tight-velocity-v43` |
| F32C 电机摇杆 | 37 | [运行说明](motor_joystick/README.md) |
| 无人机传感器融合 | 30 | [运行说明](drone_sensor_demo/README.md) |
| Thermal160 无人机检测 | 13 | [运行说明](thermal_uav_detector/README.md) |
| 热像与 YOLO 融合 | 23 | [运行说明](thermal_yolo_fusion/README.md) |
| 麦克风阵列方向 | 5 | [运行说明](micarray_direction/README.md) |
| 声音寻向与 YOLO 云台 | 17 | [应用说明](sound_yolo_gimbal/README.md) |

构建 YOLO 跟踪入口时保留同级 `motor_joystick/`。个别应用说明引用本地 `MaixCam/research` 或上游参考资料，本次快照范围为 `MaixCam/apps`。

本机验证通过 **449 项 Python 测试**，包括 YOLO 307 项、电机 108 项、传感器融合 28 项、热像无人机检测 6 项。YOLO 与电机的网页测试也通过。v43 为待上板验证的候选版。

旧 `sound_yolo_gimbal/main.py` 第 201 行的字符串未闭合，已按本地原文件收录。运行这个旧入口前需修复语法错误。
