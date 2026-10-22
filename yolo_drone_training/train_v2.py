"""Fine-tune YOLO11n from the local v1 checkpoint on audited dataset_v3."""
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
os.environ.setdefault('YOLO_CONFIG_DIR', str(HERE / '.ultralytics'))
os.environ.setdefault('MPLCONFIGDIR', str(HERE / '.mplconfig'))

def main():
    import torch
    from ultralytics import YOLO
    dataset = Path(r'C:\Users\admin\Documents\drone_yolo_v2\dataset_v3\data.yaml')
    weights = Path(r'C:\Users\admin\Documents\drone_yolo_v2\v1\training\best.pt')
    if not dataset.is_file() or not weights.is_file():
        raise FileNotFoundError('Local dataset_v3/data.yaml or v1 best.pt is missing')
    audit = json.loads((dataset.parent / 'dataset_v3_audit.txt').read_text(encoding='utf-8').split('\n', 1)[1])
    if audit.get('status') != 'PASS' or not audit.get('regression_test_unchanged'):
        raise RuntimeError('dataset_v3 audit has not passed')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable; formal training requires device=0')
    YOLO(str(weights)).train(
        data=str(dataset), epochs=80, patience=20, batch=32, imgsz=640,
        device=0, workers=4, optimizer='AdamW', lr0=0.0005, lrf=0.01,
        weight_decay=0.01, warmup_epochs=3, hsv_h=0.015, hsv_s=0.45,
        hsv_v=0.25, degrees=3, translate=0.08, scale=0.20, fliplr=0.5,
        mosaic=0.25, close_mosaic=15, mixup=0.0,
        seed=2202, deterministic=True, cache=False, amp=True,
        project=str(HERE / 'runs' / 'v2_train'), name='v2_finetune_v3_640',
        exist_ok=False, resume=False, plots=True,
    )

if __name__ == '__main__':
    main()
