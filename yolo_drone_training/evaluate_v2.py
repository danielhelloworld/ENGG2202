"""Evaluate v2 on validation, sealed final test, or old regression test."""

import argparse
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA = Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3")
os.environ.setdefault("YOLO_CONFIG_DIR", str(HERE / ".ultralytics"))
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".mplconfig"))


def main() -> None:
    from ultralytics import YOLO

    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", default=str(HERE / "runs/v2_train/v2_finetune_v3_640/weights/best.pt"))
    parser.add_argument("--split", choices=["val", "final_test", "regression_test"], default="val")
    args = parser.parse_args()
    weights = Path(args.weights)
    if not weights.is_file():
        raise FileNotFoundError(weights)
    if not (DATA / "data.yaml").is_file():
        raise FileNotFoundError(DATA / "data.yaml")
    data_yaml = DATA / "data.yaml"
    ultralytics_split = "val" if args.split == "val" else "test"
    if args.split == "regression_test":
        config_dir = HERE / ".ultralytics"
        config_dir.mkdir(parents=True, exist_ok=True)
        data_yaml = config_dir / "regression_test_eval.yaml"
        data_yaml.write_text(
            f"path: {DATA.as_posix()}\n"
            "train: images/train\nval: images/val\n"
            "test: images/regression_test\nnames:\n  0: drone\n",
            encoding="utf-8",
        )
    YOLO(str(weights)).val(
        data=str(data_yaml),
        split=ultralytics_split,
        conf=0.25,
        iou=0.45,
        imgsz=640,
        device=0,
        batch=32,
        plots=True,
        save_json=True,
        project=str(HERE / "runs" / "v2_evaluation"),
        name=f"{args.split}_conf025_nms045",
        exist_ok=False,
    )


if __name__ == "__main__":
    main()
