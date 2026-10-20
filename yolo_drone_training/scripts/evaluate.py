#!/usr/bin/env python3
"""Evaluate a trained model on the held-out test split."""

from __future__ import annotations

import argparse
from pathlib import Path

from runtime_compat import configure_project_caches

PROJECT_ROOT = Path(__file__).resolve().parents[1]
configure_project_caches(PROJECT_ROOT)

from ultralytics import YOLO
from runtime_compat import patch_mps_validation_autocast


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--data", type=Path, default=Path("data.yaml"))
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="mps")
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()

    patch_mps_validation_autocast()
    model = YOLO(str(args.model.resolve()))
    metrics = model.val(
        data=str(args.data.resolve()),
        split="test",
        imgsz=args.imgsz,
        device=args.device,
        conf=args.conf,
        project=str(Path("runs").resolve()),
        name="test_evaluation",
        plots=True,
    )
    print("mAP50:", float(metrics.box.map50))
    print("mAP50-95:", float(metrics.box.map))
    print("precision:", float(metrics.box.mp))
    print("recall:", float(metrics.box.mr))


if __name__ == "__main__":
    main()

