#!/usr/bin/env python3
"""Export YOLO11 weights to a fixed-shape ONNX for MaixCAM2 conversion."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    args = parser.parse_args()
    if args.height % 32 or args.width % 32:
        raise SystemExit("height and width must both be multiples of 32")

    model = YOLO(str(args.model.resolve()))
    output = model.export(
        format="onnx",
        imgsz=(args.height, args.width),
        opset=17,
        simplify=True,
        dynamic=False,
    )
    print("MaixCAM2 conversion input:", output)


if __name__ == "__main__":
    main()

