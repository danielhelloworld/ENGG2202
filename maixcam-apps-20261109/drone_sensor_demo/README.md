# 无人机三传感器权重 Demo

2026-11-06 热像整合版；详细变更、校准复用和验证见 [INTEGRATION.md](INTEGRATION.md)。

目标：5/10m 优先可靠确认，30–100m 探索疑似目标发现。
这是融合策略和输入接入 demo，不包含训练好的无人机模型，不承诺距离或识别率。

## 电脑直接运行

双击 `launch_demo.cmd`，或在本目录运行 `python demo.py`。
依赖 Python 3、numpy、opencv-python；当前电脑已有依赖。

- 顶部滑条调整视觉权重（60%～100%）和辅助权重中声音的比例。
- 默认视觉 70%、声音 15%、热像 15%，视觉始终大于辅助两路之和。
- S 原子保存到本目录 config.json；再次启动自动加载。
- 1～6 切换：5m 三路、10m 纯视觉、远处声热、视觉中断、错误声源方向、过期数据。
- Q / Esc 或关闭窗口退出。场景名中的距离只是演示标签，没有模拟真实光学和声学衰减。
- 电脑演示全部使用明确标识的合成观测，不连接硬件、不执行模型推理。

`python demo.py --headless` 运行六种场景并输出 validation 下 PNG 和 JSON。
`python -m unittest test_demo -v` 验证融合边界。

## 权重规则

融合分数 = 0.70×有效视觉证据 + 0.15×有效声音证据 + 0.15×有效热像证据。
三种输入分数含义不同，目前是启发式融合，**不是识别准确率、后验概率或经校准的概率**。
缺失、过期、未来时间戳、标定无效或关联歧义的证据不贡献分数；不重新分配缺失权重。

确认必须：当前有效视觉类别在 drone_labels 中、模型分数≥0.80，且同一局部轨迹最近0.7s内的最近5个新视觉观测至少3个通过。
只有声热最多贡献0.30，只显示 CANDIDATE。高置信视觉可以独立确认。
视觉过期立即撤销当前确认，已确认轨迹暂存为 HOLD_UNCONFIRMED，过期证据分数归零。
无新观测超过0.8s删除轨迹。重复帧不累计确认次数。
跨传感器关联需位置接近（归一化距离0.12）和时间接近（0.25s）；歧义时拒绝融合。
视觉位置有效时，辅助证据不能拖动其位置。当前为位置门限关联，交叉/高速运动可能换 ID，尚无运动预测或背景运动补偿。

## MaixVision 运行

打开生成的 **main.py**，单文件可运行，第三方协议代码已内嵌，无需上传相邻 Python 文件。
运行前停止其他占用相机/UART1/UART2的程序。

1. 板上安装 numpy、cv2 及配套 MaixPy（沿用现有项目运行环境）。
2. main.py 内 DEFAULT 可编辑；默认 camera_fps=60，与原叠加层默认模式一致，沿用旧校准时须与其保存的帧率相同；若板上存在 `/root/drone_sensor_demo/config.json`，其配置优先。可把桌面 config.json 上传到该路径。
3. 默认读取 `/root/models/yolo11n.mud` 以检查现有视觉链路。通用 COCO 模型通常没有无人机类别；界面显示 `MODEL HAS NO UAV LABEL`，不会把鸟/飞机当无人机确认。
4. 真正识别需提供训练并转换好的无人机 `.mud + .axmodel`，设置 model_path、model_type（YOLO11/YOLOv8，取决于已安装接口）和实际 drone_labels。不得把 COCO 的 airplane 标签改名冒充 drone 训练结果。
5. Thermal160 复用 telemetry30 UART2 B0/B1、A9复位、4Mbaud读取。USB UVC 固件不能直接用于此入口。声音复用 SoundDirectionSensor，UART1 A30/A31、2Mbaud；读取声源热图，不包含音频声纹分类。
6. 屏幕使用原叠加层界面：VIEW/ALPHA/DIST/CAL/ALIGN/SAVE/TEMP/HEAT，保留 FAR/NEAR、四向移动、宽高调整、FINE/RESET。热像候选标记为 T#，它不代表无人机分类。未验证映射可用于粗略显示，不能贡献融合证据。独立检测器使用原始160×120灰度多尺度局部对比并要求3个新帧确认热像轨迹；无效、重复、过期及预测帧不产生新融合证据。

### 标定配置契约

- calibration_resolution: `[RGB模型输入宽, 高]`，不匹配时关闭辅助融合。
- thermal_to_rgb: 3×3单应矩阵，**未翻转热像原始像素** `[x,y,1]` → RGB归一化 `[u,v,1]`，u/v均0～1。不是显示伪彩图坐标。
- sound_to_rgb: 2×3仿射矩阵，SoundDirectionSensor输出 `vector_xy` → RGB归一化位置。
- sound_calibration_bounds: `[vx_min,vy_min,vx_max,vy_max]`，仅允许实测覆盖范围内使用。

所有矩阵默认 null，绝不使用猜测映射。热像现在直接复用 thermal_yolo_fusion 的 Alignment、CalibrationSession 和 TouchUI。系统自动将有效叠加层矩阵去除画布留边和像素中心缩放，再转换到 RGB 归一化坐标；无需手填 thermal_to_rgb。旧的裸矩阵字段保留兼容配置，但板端不再将它作为热像融合依据。见 [整合说明](INTEGRATION.md)。
声音的二维热图不是物理DOA。此简化映射仅适用于**固定相机/阵列姿态和已验证视域**；底座固定阵列与转动云台必须另行接入云台姿态变换后才能使用。声音关联只表明方向相近，不证明无人机发声。
本 demo 不连接电机。保持相机固定做融合测试；会动的云台需增加姿态补偿。

控制台每秒 DRONE_FUSION JSON 包含权重、目标、贡献、热块和标定状态。
屏幕为真实相机底图和证据状态，IDE停止时释放UART与相机；某一路缺失不会伪造数据。
模型/相机启动失败会报错退出；声音连接失败降级运行，热像沿用原读取器重连。

## 如何继续验证

先核对模型无人机类别、两路新帧及静态标定，再录制5m/10m正负样本，记录漏检、每分钟误报和轨迹中断；30/50/100m单独报告检测结果。
相机运动、坏点、树/云边、多个声源、目标交叉与跨模态时间延迟仍需实机测试。
只有当前电脑合成验证；没有训练权重、板上部署、距离识别率或电机联动验收。

维护入口：core.py（融合）、demo.py（PC窗口）、thermal_integration.py（热像与校准桥接）、board_runtime.py（板端）、build.py（单文件打包）。
修改后执行 `python build.py`。原 thermal_yolo_fusion、micarray_direction、sound_yolo_gimbal 均未修改。
协议归属见 THIRD_PARTY.md、LICENSE_SIPEED.txt；THIRD_PARTY为原融合应用归属说明，其RTSP部分不包含于新demo。
