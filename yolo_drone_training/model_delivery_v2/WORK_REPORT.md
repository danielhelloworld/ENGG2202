# ENGG2202 YOLO11n 无人机模型完整工作报告

日期 2026-11-06。目标是 MaixCAM2 的 RGB 单类别无人机检测，类别 0 为 drone。

## 恢复与训练

- GitHub 来源为 danielhelloworld/ENGG2202 的 feature/yolo-drone-model-v1 分支，起点提交 5d18af78285f6b06346d8975f00d9d71d617b6d6。
- dataset_v3 版本为 drone_rgb_v3_20261030，24,846 张图。训练 21,737 张，验证 1,692 张，最终测试 528 张，历史回归 889 张。原划分保持不变。
- 三个数据分包、原交接文件与逐图数据校验通过。使用 Python 3.11.14、torch 2.6.0+cu124、Ultralytics 8.4.104、RTX 4090。
- 从含 optimizer、scaler、EMA 的 deadline_resume.pt 在第 35 轮继续。配置为 YOLO11n、640、batch 16、AdamW、lr0 0.001、seed 2202、patience 15、计划总计 60 轮。
- 第 40 轮正常早停。最佳模型来自第 25 轮，SHA-256 为 60ae24dd60da5874220cc5de2f1475a7ce794a995e3b6f0ebce9e978df205163。
- 按用户后续指示，通用预训练 yolo11n.pt 独立对照实验暂缓。

## 验证与测试

| 划分 | 图片 | Precision | Recall | mAP50 | mAP50–95 |
|---|---:|---:|---:|---:|---:|
| val | 1,692 | 0.781885 | 0.776749 | 0.772404 | 0.371198 |
| final_test | 528 | 0.937744 | 0.831482 | 0.861112 | 0.480631 |

错误图按置信度 0.25、NMS IoU 0.45、匹配 IoU 0.50 统计。最终测试有 396 个匹配框、38 个误检框、75 个漏检目标。验证集有 798 个匹配框、225 个误检框、245 个漏检目标。完整分类和示例见 ERROR_ANALYSIS.md。最终测试集中小于 20px 的目标有 94 个，其中 45 个漏检。

## 板端转换

- 最新 best.pt 导出为静态 ONNX。输入形状 [1,3,480,640]，opset 17，dynamic=False，nms=False。ONNX 图检查通过。
- 100 张训练集校准图位于 calibration_images.zip 根目录。MaixHub 任务 11310 使用 MaixCAM2、YOLO11 Detect、标签 0=drone，转换成功。
- 板端包含 model_11310.mud、model_11310_npu.axmodel 和 model_11310_vnpu.axmodel。MUD 中的文件引用、模型类型、类别和预处理参数已检查。板端运行、帧率及实拍效果仍需实机验证。

## U 盘交付结构

根目录为 E:\ENGG2202_yolo_training\deliverables\drone_yolo11_v2_from_v1_20261106。

- training 保存最佳权重、各阶段权重、每轮结果、曲线和训练参数。
- validation 与 final_test 保存指标、PR/F1 曲线、混淆矩阵和预测文件。
- error_analysis 保存最终测试的仅误检、仅漏检、同时误检漏检图片和逐图 CSV，validation 子目录保存验证集的同类结果。
- conversion_input 保存 ONNX、100 张校准图及转换配置。
- board_model 保存 MaixCAM2 可加载的 MUD、两份 AXMODEL、原始下载 ZIP 和说明。
- training.log、environment_freeze.txt、reproduce 保存日志、依赖及复现所需的交接信息。

完整训练结果目录 640 个文件、分类误差交付目录 486 个文件和板端包均已逐文件通过 SHA-256 校验。校验清单分别在各自目录中。

## USB 摄像头测试

接入 USB 摄像头后，在本机 PowerShell 运行下列命令。摄像头序号通常从 0 开始。如果 0 号是内置摄像头，将 source 改为 1 或 2。

    cd C:\Users\admin\Documents\Codex\2026-11-06\u-1-u-engg2202-yolo-training
    & .\work\.venv\Scripts\yolo.exe predict model=.\outputs\best.pt source=0 show=True imgsz=640 conf=0.25 device=0

窗口中可实时查看检测框，按 Q 退出。先观察无人机、无目标背景、鸟或飞机干扰等场景。此步骤测试电脑上的 PyTorch 模型。板端量化后的模型需在 MaixCAM2 上用同一场景再测一次。
