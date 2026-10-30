# Drone YOLO11 — 机房训练与 MaxiCAM2 交付

这是 RGB 单类别项目：`0: drone`，模型为 YOLO11n Detect。当前可训练数据版本是 `drone_rgb_v3_20261030`；新模型训练后命名 V2。V1 成果保留在 `model_delivery_v1/`，不是新训练结果。

从 U 盘 `READY_TO_TRAIN/START_HERE.md` 开始。环境、数据恢复、检查和短跑由 `setup_windows.ps1` 完成；`lab.py` 统一执行训练、评估、错误图片整理、校准集生成、ONNX 导出和成果保存。

## 训练顺序

1. 新验证集上评估 V1，建立基线。
2. 从 U 盘 `models/v1_best.pt` 开始新任务微调：640 输入、60 epochs、patience=15、batch=16、AdamW、lr0=0.001、seed=2202、resume=False。运行名不可重复，结果保存在本机 SSD。
3. 有时间时以官方 `models/yolo11n.pt` 为对照起点；用同一数据和验证协议比较。
4. 用验证结果选方案，再运行新 `final_test`。旧 `regression_test` 只用于历史对比。
5. 导出固定 640×480 ONNX，生成 100 张训练来源的校准图，保存完整成果到 U 盘。机房详细命令在交付包中。

## 文件入口

| 文件 | 用途 |
|---|---|
| `lab.py` | `restore / check / doctor / smoke / train / eval / errors / calibration / export / save` |
| `setup_windows.ps1` | Python 3.11、CUDA PyTorch、依赖、数据恢复、GPU 短跑 |
| `requirements.txt` | 训练及导出依赖；Windows PyTorch 由安装脚本单独安装 |
| `requirements-macos.txt` | 保留的 Mac 环境配置，不用于机房 Windows |
| `DATASET_REPORT.md`、`reports/` | 最终数量、来源、筛选和校验报告 |
| `DEPLOYMENT.md` | 转换及队友接入步骤 |
| `maixcam2_verify.py` | 板端识别验证示例，不直接驱动云台 |

数据和原始下载包不进入 Git。U 盘包含完整数据、权重、同版代码快照与操作说明。未来 V2 可在此分支提交权重和精简结果；不要提交数据、虚拟环境或缓存。

评估 AP 使用 `conf=0.001`、NMS IoU=0.7；部署起点 `conf=0.25`、NMS IoU=0.45 是另一组用途不同的参数，之后只在 validation 上调整。误检/漏检匹配使用 IoU≥0.5，一张图可同时有 TP 与 FP。
