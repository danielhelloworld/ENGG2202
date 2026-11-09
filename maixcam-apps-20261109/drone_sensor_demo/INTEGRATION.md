# 热成像整合到三传感器系统

2026-11-06。本地维护入口是本目录的 `main.py`（MaixVision 单文件），由 `build.py` 生成。
本次没有推送 GitHub，也没有在设备上启动程序或电机。

## 已接好的链路

- 唯一 ThermalReader：UART2 -> telemetry30 解码 -> 独立 thermal_uav_detector.Detector -> adapter.observations -> Fusion，与视觉/声音一起关联。
- 检测器来自 feature/thermal-uav-detector 的 d508eeb；本地 detector.py 与克隆版本 SHA256 一致。检测与适配器保持原实现，由构建器内嵌到独立命名空间。
- 默认视觉/声音/热像权重维持 70/15/15。热像轨迹的 CONFIRMED 只是多帧热像候选，不会变成 VISUAL_CONFIRMED。
- T# 框、伪彩叠加、框内原始像素测温、热像融合坐标共用原 Alignment 的有效几何，不再维护两套相互独立的热像矩阵。
- 沿用固定相机模式；没有接入电机控制和云台姿态补偿。`fixed_camera=false` 时关闭热像融合。
- 当前三传感器入口输出本机显示及 DRONE_FUSION 日志；原独立双光应用的 RTSP 不在本入口中。

## 使用之前的叠加层校准

1. MaixVision 打开本目录 `main.py`。先停止其他占用相机、UART1/2 的应用。
2. 默认从设备 `/root/thermal_yolo_fusion/alignment.json` 读取原 schema4 校准文件。这里只做兼容读取与点误差重查；未获取你的设备文件，不能保证其中已有可用档位。运行前可先备份此文件，SAVE 会沿用原格式保存所有已载入档位。
3. 模型输入尺寸、camera_fps、热像翻转必须匹配原校准。当前默认60fps；若设备的 `/root/drone_sensor_demo/config.json` 中有30fps，会优先使用30fps，需要改成与原校准相同的模式。模式不匹配会拒绝旧映射并提示重新校准。
4. DIST 选择工作距离。原0.5/1/2/3/5m档位保留，新增10/30/50/100m空档，新增档不含虚构矩阵。这里只使用手选的实测档位，没有自动测距、插值或未经验证的距离范围。
5. ALIGN 保留 FAR/NEAR 等比例前后缩放、LEFT/RIGHT/UP/DOWN、W/H、FINE、RESET；ALPHA 调透明度。它们的作用与原应用一致。
6. CAL -> FREEZE -> 至少6组双光同名点 -> FIT -> 至少2组新点独立检查 -> APPLY -> SAVE。详细选点步骤仍见 `../thermal_yolo_fusion/CALIBRATION.md`。
7. 显示 POINT CAL CHECKED 后，该档热像可贡献融合证据。只做手动粗调而没有点校准时显示 NO POINT CAL；后置微调若使原独立检查点误差超过6个RGB输入像素，显示 TUNED MAPPING NEEDS CAL，应重新CAL或RESET。阈值沿用旧配置。

可在 DEFAULT['thermal_alignment'] 内嵌原导出的 `calibration_profiles`、`manual_adjustments` 等叠加层配置，设备同名有效档位仍优先。不要只粘贴一张不带检查点的矩阵。

叠加层矩阵原本是“未翻转热像原始像素 -> 显示画布”。桥接先反算 RGB 的 letterbox（含 OpenCV 像素中心偏移），再按模型输入宽高归一化；翻转及手动微调已包含在原矩阵中，不会重复应用。融合前用有效矩阵重新检查保存的独立点。

切换距离、修改几何、进入或退出CAL/ALIGN时清掉旧融合轨迹，校准期间暂停视觉推理和辅助证据。热像使用读取器计数与接收时间；重复邮箱帧不会增加命中次数。候选显示可暂存最新真实检测，但过期、失效、运动不确定或与RGB不同步时隐藏，不产生预测证据。

## 本地验证

- 三传感器与桥接：28项通过，含独立热像3帧确认、三路同位置融合、禁止热像独立确认无人机、重复/过期/预测帧、校准暂停、原始像素测温、letterbox/翻转、旧schema4重载、帧率/分辨率拒绝、模拟板端清理及候选显示时效。
- 原叠加层校准：18项通过，含完整触摸选点流程、距离档位保存和手动前后缩放。
- 独立热像检测器：6项通过。
- 合计52项电脑测试。测试临时目录设在工作区，避开Windows沙箱系统临时目录的原子替换限制。

```powershell
python -B MaixCam/apps/drone_sensor_demo/build.py
python -B -m unittest discover -s MaixCam/apps/drone_sensor_demo -p 'test*.py'
python -B -m unittest discover -s MaixCam/apps/thermal_uav_detector/tests
# 校准测试如遇系统临时目录权限问题，可在当前终端设置：
$env:TEMP = (Resolve-Path MaixCam/apps/drone_sensor_demo/validation).Path
$env:TMP = $env:TEMP
python -B -m unittest discover -s MaixCam/tests -p test_registration.py
```

这些是合成几何、合成帧和模拟设备验证；尚未验证真实UART、双光对齐精度、板上帧率、无人机识别率或任何距离的检测性能。纯Python热像处理的板上成本也需实测。通用COCO模型仍不能确认无人机。未修改原热像/声音应用与上游参考源码。
