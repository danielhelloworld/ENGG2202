#!/usr/bin/env python3
"""Build a deterministic image-only ZIP for MaixCAM2 INT8 calibration."""

from __future__ import annotations

import argparse
import random
import zipfile
from pathlib import Path

from check_dataset import IMAGE_EXTENSIONS


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", type=Path, default=Path("dataset/calibration/images")
    )
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/drone_calibration_images.zip")
    )
    args = parser.parse_args()
    if not 20 <= args.count <= 100:
        raise SystemExit("official converter expects 20..100 calibration images")

    images = []
    if args.source.is_dir():
        images.extend(
            path for path in args.source.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )
    if len(images) < args.count:
        raise SystemExit(f"need {args.count} images, found {len(images)}")

    rng = random.Random(2202)
    selected = sorted(rng.sample(images, args.count), key=lambda path: path.name)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for index, image in enumerate(selected):
            archive.write(image, arcname=f"{index:03d}_{image.name}")
    print(f"wrote {args.output} with {len(selected)} images")


if __name__ == "__main__":
    main()
