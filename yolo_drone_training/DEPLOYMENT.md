# 训练后交给队友

## 模型链路

`best.pt → best.onnx → MaixHub 转换 → .mud + .axmodel → MaxiCAM2 实机验证 → 接现有融合/云台控制`

训练电脑不需要安装相机上的 MaixPy。`.pt` 是训练权重；ONNX 用于转换；`.mud` 描述类别、预处理和模型文件，实际板端模型在 `.axmodel` 中。

## 转换参数

- 目标板：MaixCAM2；YOLO11 Detect；类别按顺序仅 `drone`。
- 使用 `ultralytics==8.4.104`，输入固定 `[1,3,480,640]`，opset 17，dynamic=False，nms=False，simplify=True。
- `lab.py export` 检查 ONNX 格式与输入尺寸。完整 ONNX 和 `calibration_images.zip` 上传至 https://maixhub.com/toolbox 的模型转换器。
- 校准 ZIP 含 100 张图片，图片直接位于 ZIP 根目录；当前来自训练集。拿到真实 MaxiCAM2 画面后可替换为代表性场景重新量化，不需要重训。
- 下载成功后保留所有 `.mud`、`.axmodel`，不要只取 `.mud`，不要用手写 `.mud` 代替实际模型编译。

Sipeed 官方步骤：
- https://wiki.sipeed.com/maixpy/doc/en/ai_model_converter/online_converter.html
- https://wiki.sipeed.com/maixpy/doc/en/ai_model_converter/maixcam2.html

手动 Pulsar2 路线对应 AX620E。若手动提取 YOLO11 特征输出，单类别三层形状应是 `[1,65,60,80]`、`[1,65,30,40]`、`[1,65,15,20]`，应检查实际图中的节点，不能按旧节点名字直接裁剪。常规使用在线转换即可。

## 板端及融合测试

1. 所有转换文件放到 `/root/models/drone_v2/`。在 `maixcam2_verify.py` 填写实际 `.mud` 文件名，用 MaixVision 在 MaxiCAM2 运行。
2. 先验证白天、复杂背景、鸟/飞机干扰下的框、类别、帧率与延迟。与电脑对同一画面的识别比较，检查量化后是否明显下降。
3. 现有融合程序把 YOLO 模型加载类改为 `nn.YOLO11`，模型路径指向新 `.mud`。类别 `0` 现在表示无人机，不再按通用 COCO 类别编号筛选。
4. 接现有目标选择和跟踪接口。检测框中心转换为相对于原始图像的坐标，确认缩放/黑边映射；再确认云台 x/y 正负方向、死区、限位、丢失目标停止与重新搜索。
5. 记录检测与跟踪成功率、误跟随、丢失次数、FPS 和端到端延迟。热成像融合保持独立输入，不能把 RGB 模型当作热成像模型。

本次交付准备的是数据和训练/转换流程。新 V2 权重、其板端编译文件与实机闭环结果在下一次训练和队友测试后生成。已有 V1 的完整板端文件仍保留，可继续整体调试。

仓库 `maixcam2_person_tracker/` 当前使用 YOLO11，但配置和目标选择仍按 person/COCO 假设编写。接入时应一并核对配置、类别检查和目标选择，不能只替换模型文件。此次没有改动该控制项目。

## V2 成果提交

建立 `model_delivery_v2/`，至少包含：best.pt、test/val 指标、results.csv、训练参数、PR 曲线、混淆矩阵、数据版本、输入尺寸、转换参数与 SHA-256。ONNX/板端包体积合适时可一并提交；超过 GitHub 普通文件限制时放 Release，并在说明中链接。不要把 `.venv`、数据或中间缓存提交。
