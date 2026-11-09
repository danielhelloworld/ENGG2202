# 功能研究进度与未解决问题

资料复核于 **2026-11-09**。这里汇总跨模块研究结论。源码与各应用的验证阶段见 [模块进度](MODULE_STATUS.md)。

## 速度控制与跟随延迟

- 从软件位置流、驱动内部轨迹发展到模式 0 的相机外环速度控制。v42 依据实机反馈分离原始 RPM 与机械角方向，HOME 先低速核对方向，再使用独立增益和阻尼。从 pitch 118°回零是该次实测范围。偏心负载下失能后下垂、速度指令整数 RPM 的近零分辨率和实际负载制动能力仍影响表现。 [速度驱动说明](yolo_click_tracker/VELOCITY_DRIVE.md) · [维护配置](yolo_click_tracker/src/config.py)
- 相机侧使用双缓冲推理、最新目标邮箱、独立显示及串口工作线程，网页按需取图。历史 v42 空闲采样中取图返回到控制提交中位数约 74.7 ms，UART 周期中位数约 61.2 ms。这两项来自各自统计窗口，尚缺从曝光到电机响应的同步测量。 [相机运行入口](yolo_click_tracker/src/runtime.py) · [姿态补偿与预测](yolo_click_tracker/src/prediction.py)
- v43 试图降低匀速残差。本机一组假设速度增益 0.7、观测延迟 120 ms 的合成对比中，移动误差 RMS 从 3.59°降到 1.57°，急停过冲从 5.83°增到 7.57°。其他假设下收益不同。恢复 pitch 反馈后，需用同一场景对比 v42/v43 的匀速误差、反向、急停和丢失次数。 [速度驱动测试](yolo_click_tracker/tests/test_velocity_drive.py)

## 连续 yaw、过顶与短时丢失

- 历史 v36 在位置驱动模式下实测双向跨越 180°，最大采样 yaw 200.6°，随后按最近方向 HOME。v43 速度模式保留连续角反馈与最近等价零位逻辑。当前速度版本的连续旋转和实际线束适配仍需按装配验证。 [连续角反馈与保护](yolo_click_tracker/src/tracking_motor.py)
- 高仰角按空间视线计算，顶点附近稳定方位，越顶后由 yaw 换向。漏检可短时减速续跟，过顶窗口只接受满足视线与类别条件的唯一续锁候选。已有本机模型覆盖，真实无人机高速越顶及相近目标交叉仍缺实测。 [过顶状态机](yolo_click_tracker/src/overhead.py) · [过顶测试](yolo_click_tracker/tests/test_overhead.py)

## 电机参考、抗扰与参数辨识

- 独立驱动和跟踪入口使用已保存的机械零点，支持限位外向内 HOME、反馈过期停机、固定参考保持及故障后重新检查。历史 v22 用户手推复测确认可保持使能并回位，速度模式下的抗扰能力仍需按当前负载复测。速度模式的停止请求依赖进程与 UART 能发出零速或失能帧，电机端通信超时停车缺少验证。 [保持研究](yolo_click_tracker/HOLD_POSITION.md) · [速度模式停止边界](yolo_click_tracker/VELOCITY_DRIVE.md)
- 位置驱动下试过软件轨迹与电机内部轨迹。用户观察内部轨迹更顺滑，提高速度本身未消除抖动。当前仍缺可复用的实测惯量、负载扭矩、高频振动与不同俯仰姿态的温升数据。 [目标角度试验](yolo_click_tracker/ANGLE_TEST.md) · [历史实机驱动记录](yolo_click_tracker/validation/LIVE_DRIVE_20261105.md)
- 已实现依赖已知基线的单轴 PI 候选评分、基线重测与取消恢复。厂家资料的公开反馈包括角度、速度、加速度与母线电压。电流设置、实时力矩、PI 读回的协议边界以及历史发送值与上位机显示差异限制了自动辨识。下一轮需先核实写入与回读的一致性，再比较实际负载下的响应。 [整定入口](yolo_click_tracker/AUTOTUNE.md) · [PI 与采样说明](motor_joystick/PID_TUNING.md)

## 双光标定与测温

- 已实现每距离独立保存的点对校准，至少 6 组点拟合并使用至少 2 组新点检查。热像翻转、画布留边、缩放与后置微调进入同一几何链。融合模块复用有效的 schema 4 标定，默认短距档 0.5/1/2/3/5 m，融合另设 10/30/50/100 m 待标定档。 [点对校准](thermal_yolo_fusion/CALIBRATION.md) · [融合复用与门槛](drone_sensor_demo/INTEGRATION.md)
- 当前依赖手选距离和固定相机。多深度视差、遮挡、相机运动、两路时间偏差和不同距离测温误差仍待实测。基于测距的自动档位、双目外参与深度重投影、跨模态锚点微调已有设计，尚未进入默认应用。 [动态几何与热像模型研究](thermal_yolo_fusion/DYNAMIC_ALIGNMENT_AND_THERMAL_YOLO.md)

## 热像小目标与多模态确认

- 独立 Thermal160 检测采用多尺度局部对比和跨帧关联，可在合成回放中保留单像素热峰、屏蔽标记坏点并抑制广域变化。热像输出是候选质量和轨迹状态。独立热像 YOLO 的训练、量化、板端吞吐与实际识别率仍待专门数据集。 [独立检测报告](thermal_uav_detector/TEST_REPORT.md)
- 视觉优先的三传感器融合支持真实新鲜观测、独立标定门禁与缺路降级。声音和热像可帮助发现疑似目标，视觉确认由明确的无人机类别证据触发。当前主要缺口是专用视觉模型、同步实拍和分距离标注，需要按完整采集会话统计漏检、误报和跟踪中断。 [融合算法与距离目标](drone_sensor_demo/README.md) · [热像适配契约](thermal_uav_detector/HANDOFF.md)

## 声源热图与物理方位

- 已完成 MA-USB8 二进制热图解析、主区域方向估计、静音和失效诊断，并支持阈值、灯光配置请求。相机配置文件持久化已有本机验证，板端最终参数及 Flash 持久化缺少回读证据。 [控制契约](micarray_direction/CONFIG_CONTROL.md)
- 物理方位需结合阵列安装、固定底座坐标与现场声场标定。多人、反射、遮挡及阵列与相机相对转动尚未验证。八通道原始音频、声源身份识别与转动云台的跨模态坐标变换仍待独立实现和验收。 [方向诊断](micarray_direction/MODULE_TEST.md) · [声音引导状态机说明](sound_yolo_gimbal/README.md)

## 相机资源、热像协议与推流

- 当前方案使用 UART1 读取 MA-USB8 热图，UART2 读取 Thermal160 telemetry30，UART3 控制 F32C。热像复位使用 A9，与 A9 的 I²C7 复用功能冲突。相机和各 UART 的所有权需在多模块集成时核对。热像旧灰度帧、telemetry30 与 USB UVC 对应不同入口，现场模块固件及长时间稳定性仍待核验。 [热像运行要求](thermal_yolo_fusion/README.md) · [三路资源表](drone_sensor_demo/README.md)
- 独立热像叠加的 RTSP 已用本机 FFmpeg 解码合成帧。真实板端吞吐和客户端端到端延迟待测。兼容模式在同进程间隔运行 YOLO，独立推理子进程仍属实验路径。三传感器融合当前输出设备屏幕与诊断日志。 [推流与性能](thermal_yolo_fusion/PERFORMANCE.md) · [融合输出](drone_sensor_demo/INTEGRATION.md)

## 证据来源

快照内的入口、测试、合成报告和验证资料已在各节链接。进一步的本地实验摘要来自 `MaixCam/research/motion_jitter_20261105/` 的速度方向、v43 对比、过顶及延迟报告，以及 `yaw_force_20261105/REPORT.md`、`pitch_force_20261105/REPORT.md` 的逐轴试验记录。平台和热像协议资料来自 `MaixCam/docs/`，麦克风接入依据来自 `MaixCam/research/MICARRAY_MAIXCAM2_CONNECTIONS.md`。上述文件名用于定位研究依据，本文同步结论和验证边界。
