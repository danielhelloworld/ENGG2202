# YOLO v2 训练准备（本机，未开始正式训练）

仓库 `C:\Users\admin\Documents\Codex\ENGG2202`，分支 `feature/yolo-drone-model-v1`。数据位于本机 `C:\Users\admin\Documents\drone_yolo_v2\dataset_v3`。只使用 `data.yaml` 的 train/val 训练和选阈值；`final_test` 为新来源组的最终评价，旧 v1 test 作为逐字节不变的 `regression_test`，两者均不参与调参。

## 状态

新版 v3 有 9,545 张：train 6,882、val 1,522、final_test 252、regression_test 889。机器全量审计 PASS，详细人工复核边界见 `DATASET_V3_REPORT.md`。离线派生视图 320 张已删除；AOD-4 新增 1,466 张，Halmstad 新 RGB 新增 83 张。正式训练前，数据标注负责人仍需复核 `review_required.csv` 和跨视频近似画面候选。

环境：Python 3.11.17；PyTorch 2.6.0+cu124；Ultralytics 8.4.104；RTX 4090；`torch.cuda.is_available() == True`；PyTorch CUDA runtime 12.4；`pip check` 无损坏依赖。虚拟环境 `.venv`，不重新安装。

## 参数

| 参数 | 设置 | 依据 |
|---|---|---|
| 初始权重 | 本机 v1 `training/best.pt` | 保留 YOLO11n 已学特征 |
| 架构 / 输入 | YOLO11n / 640 | 与 v1 基线保持可比；较多 20–47 px 小目标 |
| epochs / patience | 80 / 20 | 比旧 60/15 设置给新增 1,549 张独立真图更多收敛机会，同时早停 |
| batch / device | 32 / `0` | RTX 4090 24 GiB；小样本 GPU smoke 已通过；若正式训练显存不足，降至 16 并记录 |
| optimizer / LR | AdamW，`lr0=0.0005`，`lrf=0.01`，warmup 3，weight decay 0.01 | v1 权重微调采用较低初始 LR |
| 在线增强 | HSV 0.015/0.45/0.25，旋转 3°，平移 0.08，缩放 0.20，水平翻转 0.5，mosaic 0.25，最后 15 epoch 关闭，mixup 0 | 删除离线派生视图后由训练流程随机增强；限制过强变换以保护微小目标 |
| 部署起始点 | confidence 0.25，NMS IoU 0.45 | v1 validation 30 组扫描 F1 最高：P 0.846、R 0.836、F1 0.841、TP 620、FP 113、FN 122、重复框 0；v2 完成后只在 val 重新选择 |
| 输出 / 恢复 | `runs/v2_train/v2_finetune_v3_640`，`resume=False` | 与 v1 及 smoke 输出隔离；从 v1 权重重新开始微调 |

confidence/NMS 是预测与评价阈值，不进入训练损失。不得使用 `final_test` 或 `regression_test` 选择参数。最终参数锁定后，分别运行两个独立测试集并报告来源组分层结果。

## 可直接运行的 PowerShell 命令

```powershell
Set-Location 'C:\Users\admin\Documents\Codex\ENGG2202\yolo_drone_training'
.\.venv\Scripts\Activate.ps1
python -m pip check
& '.\train_v2.ps1'
```

训练后验证：

```powershell
& '.\evaluate_v2.ps1' -Split val
```

锁定阈值后，才执行 `-Split final_test` 和 `-Split regression_test`。GPU 1 epoch smoke 已单独通过：64 train / 32 val，CUDA RTX 4090，输出 `runs/v2_smoke/gpu_1epoch_v3new` 与 `runs/v2_smoke/smoke_test_result.json`；这不是正式训练，也不作为模型性能结论。所有修改只在本机，未提交或推送 GitHub。
