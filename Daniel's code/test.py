import torch

print("PyTorch 版本:", torch.__version__)
print("CUDA 可用:", torch.cuda.is_available())  # 必须 True
if torch.cuda.is_available():
    print("显卡名:", torch.cuda.get_device_name(0))
    print("CUDA 版本:", torch.version.cuda)