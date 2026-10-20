#!/usr/bin/env python3
"""Train a YOLO11 drone detector with reproducible project defaults."""

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
    parser.add_argument("--data", type=Path, default=Path("data.yaml"))
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="mps")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--name", default="drone_yolo11n_640")
    parser.add_argument("--project", type=Path, default=Path("runs"))
    parser.add_argument(
        "--resume-from",
        type=Path,
        help="Resume an interrupted Ultralytics run from its last.pt checkpoint",
    )
    args = parser.parse_args()

    patch_mps_validation_autocast()
    if args.resume_from is not None:
        checkpoint = args.resume_from.resolve()
        if not checkpoint.is_file():
            parser.error(f"resume checkpoint does not exist: {checkpoint}")
        YOLO(str(checkpoint)).train(resume=True)
        return

    model = YOLO(args.model)
    model.train(
        data=str(args.data.resolve()),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(args.project.resolve()),
        name=args.name,
        exist_ok=False,
        seed=2202,
        deterministic=True,
        patience=25,
        cache=False,
        plots=True,
    )


if __name__ == "__main__":
    main()
