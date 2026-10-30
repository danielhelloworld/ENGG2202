# V1 模型与板端文件

类别 `0: drone`，YOLO11n Detect。此目录保留 V1，不是新训练的 V2。

- `best.pt`：下一轮微调起点。
- `best.onnx`：原V1固定640×480导出。
- `board_model/model_10515.mud`、`model_10515_npu.axmodel`、`model_10515_vnpu.axmodel`：已完成转换的V1板端包，三个文件一起传到MaxiCAM2。
- 原V1量化校准图保留在U盘V1交付目录，数据图片不推送GitHub。
- `TEST_METRICS.md`：历史评估；旧测试集有来源重叠，当前只作历史对比。
- `SHA256SUMS.txt`：当前目录文件的校验值。

板上加载 `.mud`，模型输入640宽×480高。现有融合程序应使用YOLO11及drone类别，控制接口由队友确认。转换文件存在不代表相机识别和F32C闭环已实机通过。完整V1训练曲线、误检漏检图和旧部署记录保留在U盘同级 `deliverables/`。

新V2训练和转换流程见上级 `README.md`、`DEPLOYMENT.md`，以及U盘 `READY_TO_TRAIN/START_HERE.md`。
