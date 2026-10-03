# MaixCAM2 云台快拆转接台

## 选择版本

| 目录 | 用途 | 状态 |
|---|---|---|
| `output_v3_usb/` | 相机外移 20 mm，USB 侧留约 25.5 mm 空间，4 根底柱和 2 颗侧向销 | 当前 USB 优化版 |
| `output_v2_reinforced/` | 保持 V2 相机位置，增加底部热铆柱和侧向固定 | 原位置加强版 |
| `output_v2/` | 已确认的长边对齐版本 | 原版完整保留 |
| `output_v1/` | 早期方向试作 | 历史参考，未采用 |

## 打开与打印

- 在选定版本目录中打开 `MaixCAM2_quick_release_assembly.f3d`，可在 Fusion 中查看和编辑总装。
- STEP 用于交换实体，STL 用于切片；请打印选定零件，不要打印包含相机、金属支架参考体的总装。
- 加强版和 USB 版的热铆组合：`receiver_heat_stake.stl` + 一种 `camera_plate*.stl` + **2 颗** `side_heat_rivet_print_2.stl`。铆销 STL 本身只有一颗，切片数量设为 2。
- `receiver_M2_bolt.stl` 为螺钉固定的备用底座；`fit_coupon_*.stl` 为间隙试件。
- `assembly_preview.png` 为装配预览；USB 版另有 `USB_clearance_preview.png`。

详细尺寸、材料起始设置、螺丝长度和热铆顺序见 [加强版与 USB 优化说明](README_加强版与USB优化.md)及[原快拆设计说明](README_快拆转接台.md)。模型和网格检查已完成，实际摩擦保持力、热铆强度及线缆弯曲仍需试装。

## 源文件与复建工具

| 文件或目录 | 内容 |
|---|---|
| `MaixCAM2_3.0.step` | 本次采用的相机源模型 |
| `云台连接件.step` | 原云台支架源模型 |
| `3D_MaixCam2_379C_2025-09-16.step` | 较早的相机参考模型 |
| `quick_release_tool/` | Fusion 建模脚本、配置、网格校验脚本和验证记录 |
| 各版本的 `build_report.json`、`mesh_validation.json` | 几何与打印网格检查结果 |

Fusion 脚本使用 Fusion 自带的 Python 和 `adsk` API，不依赖本项目的独立虚拟环境。`validate_mesh.py` 和 `validate_optimized.py` 仅使用 Python 标准库。

当前 `command.json` 指向 USB 版复建。运行脚本会写入对应输出目录；已有文件需要另留版本时，应先调整输出目录。原相机和云台连接件文档需在 Fusion 中打开。

## 缓存清理

2026-10-03 清理记录位于 `cleanup_2026-10-03.json`。清理对象为未使用的 `cad_venv/`、脚本 `__pycache__/`、可重新生成的 `inspection.json` 和过期 `error.txt`，采用回收站保留恢复机会。版本目录、源模型、脚本和正式检查报告保持原路径。

后续运行 Python 可能重新生成 `__pycache__/`；执行 Fusion 的 `inspect` 模式会重新生成检查快照。它们不需要作为交付文件保存。
