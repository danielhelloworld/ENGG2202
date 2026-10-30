# Drone YOLO11n 640×480 v1 — independent test

**2026-10-30 数据复核更新：下文是保留的 V1 历史结果。复核发现旧来源之间存在缩放重包及视频场景重叠，因此这里的“independent test”不再作为已确认独立的评估使用。当前数据将它命名为 regression_test，仅作历史对比。新方案使用独立来源划分后的 val / final_test。**

The model was trained on `dataset_v2` at 640×640, starting from the official `yolo11n.pt`. Early stopping ended training after epoch 75; the selected `best.pt` comes from epoch 50. These numbers describe an independent test of that PyTorch model at **640×640**, before ONNX conversion or MaixCAM2 deployment.

Command: `python scripts/evaluate.py --model <best.pt> --data dataset_v2/data.yaml --imgsz 640 --device 0 --conf 0.25`.

| Dataset | Images | Drone instances | Precision | Recall | mAP50 | mAP50–95 |
|---|---:|---:|---:|---:|---:|---:|
| Independent test | 889 | 493 | 0.838825 | 0.758621 | 0.718590 | 0.290373 |

On the Windows RTX 4090 test host, preprocessing took about 0.6 ms/image, inference 0.9 ms/image, and postprocessing 0.3 ms/image. This is not a MaixCAM2 speed measurement.

The test confusion matrix shows 395 matched detections, 98 missed drone instances, and 116 background false positives. The F1 curve peaks near confidence 0.307 (F1 ≈ 0.79). The evaluation fixed `conf=0.25`, so the saved PR curve does not include lower-confidence detections. A separate per-image audit at confidence ≥0.25 and IoU ≥0.5 found examples of a bird false positive, a false box on text in an aircraft image, and missed near and distant UAVs. Its counts can differ from the Ultralytics confusion matrix because the matching rules differ.

The full evaluation directory, curves, confusion matrices, logs, and annotated error examples are included in the USB delivery package. ONNX was exported to fixed `1×3×480×640`, opset 17, with dynamic axes disabled. The test metrics above must not be presented as measured ONNX or board accuracy.

The current model has **not** passed MaxiCAM2 camera validation or F32C closed-loop gimbal validation.

