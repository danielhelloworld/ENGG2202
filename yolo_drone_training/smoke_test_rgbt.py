"""One-epoch CUDA test of genuine paired four-channel RGB-T input."""
from __future__ import annotations

import csv
import json
import os
import platform
import random
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
os.environ.setdefault("YOLO_CONFIG_DIR", str(HERE / ".runtime_rgbt"))
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".mplconfig"))


def choose(rows, split, per_source):
    rng = random.Random(2202)
    selected = []
    sources = ("UAV-CB", "Anti-UAV300") if split == "train" else ("Anti-UAV300",)
    target = per_source if split == "train" else 2 * per_source
    for source in sources:
        pool = [r for r in rows if r["split"] == split and r["source"] == source and r["fusion_eligible"] == "1"]
        rng.shuffle(pool)
        negative = [r for r in pool if r["rgb_objects"] == "0"]
        positives = [r for r in pool if r["rgb_objects"] != "0"]
        n = min(len(negative), target // 2)
        selected.extend(negative[:n] + positives[:target - n])
    if len(selected) != 2 * per_source:
        raise RuntimeError("Insufficient paired smoke samples")
    return selected


def main():
    import torch
    import ultralytics
    from ultralytics import YOLO
    from rgbt_adapter import PairedRGBTTrainer

    dataset = Path(os.environ.get("DRONE_DATASET_ROOT", r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work"))
    with (dataset / "dataset_v3_rgbt_manifest.csv").open(newline="", encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA required")
    subset = HERE / "runs/v2_smoke/rgbt_subset"
    subset.mkdir(parents=True, exist_ok=True)
    selected = {"train": choose(rows, "train", 32), "val": choose(rows, "val", 16)}
    for split, samples in selected.items():
        (subset / f"{split}.txt").write_text("\n".join(str(dataset / r["rgb"]) for r in samples) + "\n", encoding="utf-8")
    yaml = subset / "data.yaml"
    yaml.write_text(f"path: {subset.as_posix()}\ntrain: train.txt\nval: val.txt\nchannels: 4\nnames:\n  0: drone\n", encoding="utf-8")
    weights = Path(r"C:\Users\admin\Documents\drone_yolo_v2\v1\training\best.pt")
    detector = YOLO(str(weights))
    detector.train(
        trainer=PairedRGBTTrainer, data=str(yaml), epochs=1, patience=1,
        imgsz=640, batch=8, device=0, workers=0, cache=False,
        optimizer="AdamW", lr0=0.0002, lrf=0.1, warmup_epochs=0,
        hsv_h=0, hsv_s=0, hsv_v=0, degrees=2, translate=0.04, scale=0.10,
        fliplr=0.5, mosaic=0.0, mixup=0, close_mosaic=0,
        amp=False, plots=False, seed=2202, deterministic=True,
        project=str(HERE / "runs/v2_smoke"), name="rgbt_gpu_1epoch",
        exist_ok=False, resume=False,
    )
    convolution = detector.model.model[0].conv
    thermal_weight_abs = float(convolution.weight[:, 3].abs().mean())
    if convolution.in_channels != 4 or thermal_weight_abs <= 0:
        raise RuntimeError("Thermal input channel did not train")
    run_dir = Path(detector.trainer.save_dir)
    if not (run_dir / "weights/last.pt").is_file() or not (run_dir / "results.csv").is_file():
        raise RuntimeError("Smoke outputs missing")
    result = {
        "status": "PASS", "python": platform.python_version(), "torch": torch.__version__,
        "ultralytics": ultralytics.__version__, "cuda_runtime": torch.version.cuda,
        "gpu": torch.cuda.get_device_name(0), "input_channels": 4,
        "input_order": "RGB + grayscale thermal; simultaneous early fusion",
        "output_coordinates": "RGB", "thermal_channel_mean_abs_weight_after_epoch": thermal_weight_abs,
        "samples": {s: {"pairs": len(v), "sources": dict(Counter(r["source"] for r in v)),
                        "negative_pairs": sum(r["rgb_objects"] == "0" for r in v)} for s, v in selected.items()},
        "run_dir": str(run_dir), "formal_training_started": False,
    }
    (HERE / "runs/v2_smoke/rgbt_smoke_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
