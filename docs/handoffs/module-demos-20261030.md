# MaixCAM2 模块测试与跟踪代码

2026-10-30 同步本地维护版本。

- [麦克风方向与阈值/灯光配置](../../maixcam2_micarray_direction/README.md)：运行 micarray_direction.py。
- [独立电机摇杆及网页控制](../../motor_joystick/README.md)：运行 main.py 或 main_web.py。
- [无声音 YOLO 点选跟踪](../../yolo_click_tracker/README.md)：运行 main.py，版本 manual-half-soft-v6。摇杆 240/180°/s，跟踪 150/120°/s，HOME 108/90°/s；卡阻观察 1.8 秒。
- [声音寻向与 YOLO 集成](../../maixcam2_sound_yolo_gimbal/README.md)：保留独立集成应用，不与无声音版本的配置混用。

YOLO 的构建脚本依赖同级 motor_joystick/，请保留目录关系。生成入口是 MaixVision 单文件，不需要在相机上复制全部源码。运行前停止其他占用相机或 UART3 的脚本；无声音跟踪程序默认启动自动回零。

验证：无声音跟踪 154 项电脑测试、共享电机 81 项回归、Node 网页事件模型通过。这不是实机验收；本次同步不启动电机。实机抓包和 Python 缓存不在同步范围内。
