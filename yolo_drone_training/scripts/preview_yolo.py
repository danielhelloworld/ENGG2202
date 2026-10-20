#!/usr/bin/env python3
"""Draw YOLO labels on a deterministic contact sheet for visual QA."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=16)
    parser.add_argument(
        "--split",
        choices=("train", "val", "test"),
        help="preview one split from a standard YOLO dataset root",
    )
    args = parser.parse_args()

    image_dir, label_dir = args.dataset / "images", args.dataset / "labels"
    if args.split:
        image_dir /= args.split
        label_dir /= args.split
    images = sorted(image_dir.glob("*.jpg"))
    if not images:
        raise SystemExit(f"no JPG images found in {image_dir}")
    step = max(1, len(images) // args.count)
    selected = images[::step][: args.count]
    tiles = []
    for image_path in selected:
        frame = cv2.imread(str(image_path))
        if frame is None:
            raise ValueError(f"cannot read {image_path}")
        height, width = frame.shape[:2]
        label_path = label_dir / f"{image_path.stem}.txt"
        for raw_line in label_path.read_text(encoding="utf-8").splitlines():
            _, cx, cy, bw, bh = map(float, raw_line.split())
            left = int((cx - bw / 2) * width)
            top = int((cy - bh / 2) * height)
            right = int((cx + bw / 2) * width)
            bottom = int((cy + bh / 2) * height)
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(frame, image_path.stem, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 255, 255), 1, cv2.LINE_AA)
        tiles.append(cv2.resize(frame, (320, 256)))

    columns = 4
    blank = np.zeros_like(tiles[0])
    while len(tiles) % columns:
        tiles.append(blank.copy())
    rows = [np.hstack(tiles[index:index + columns])
            for index in range(0, len(tiles), columns)]
    sheet = np.vstack(rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), sheet):
        raise OSError(f"failed to write {args.output}")
    print(args.output)


if __name__ == "__main__":
    main()
