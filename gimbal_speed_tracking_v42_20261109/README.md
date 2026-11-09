# 云台速度跟踪归档（截至 v42，2026-11-09）

本目录同步本地此前的 YOLO 点选跟踪、摇杆控制及其依赖，**以 v42 为使用入口**。v43 混合精细跟踪版本已按用户反馈弃用，不包含在本次同步中。仓库其他目录未改动。

## 运行入口

- **推荐入口：[`apps/yolo_click_tracker/main_wide_motion_v42.py`](apps/yolo_click_tracker/main_wide_motion_v42.py)**，使用此前的大范围速度跟踪方案。
- [`apps/yolo_click_tracker/main.py`](apps/yolo_click_tracker/main.py) 是保留的 v41 基线，供参考及 v42 构建器读取。
- [`apps/motor_joystick/main.py`](apps/motor_joystick/main.py) / `main_web.py` 为独立摇杆控制入口。

MaixVision 运行所选单文件，模型仍需在板端配置路径存在。v42 沿用启动自动回零设置；一次只运行一个占用相机/UART 的程序。

## 内容

- `apps/yolo_click_tracker/src/`、`tests/`、`validation/`：此前速度跟踪源码、测试及记录。
- `apps/yolo_click_tracker/wide_motion_v42/`：v42 独立扩展、构建器、原因分析和验证。
- `apps/yolo_click_tracker/backups/`：v38～v40 等此前备份。
- `apps/motor_joystick/`：构建依赖及独立摇杆程序。
- `UPLOAD_MANIFEST.json`：本次上传文件的 SHA-256 清单。

## v42 行为与验证边界

大误差采用自适应速度追赶；可信短时出画时有界减速续跟，对新 ID 采用连续两帧的几何匹配确认。持续丢失后保持当前位置，不自动回零；主动 HOME 仍可使用。机械限位、反馈与卡阻保护保留。

用户反馈 v42 的大幅度跟踪基本解决；小幅度仍有滞后。此归档不声明已完成无人机实飞验证，也不把模型结果当成实际精度或速度保证。详情见 [v42 分析](apps/yolo_click_tracker/wide_motion_v42/ANALYSIS.md) 和 [使用说明](apps/yolo_click_tracker/wide_motion_v42/README.md)。

## 本地验证

从本目录执行：

```text
python -m unittest discover -s apps/yolo_click_tracker/wide_motion_v42 -p test_wide_motion_v42.py -q
python apps/yolo_click_tracker/wide_motion_v42/verify_compat.py --http
```

重新构建 v42：`python apps/yolo_click_tracker/wide_motion_v42/build.py`。构建器会验证本目录内的旧基线，没有依赖被弃用的 v43。
