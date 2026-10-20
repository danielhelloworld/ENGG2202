#!/usr/bin/env python3
"""Validate a one-class YOLO drone dataset before training."""

from __future__ import annotations

import argparse
import hashlib
from collections import Counter, defaultdict
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
SPLITS = ("train", "val", "test")


def image_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_label(path: Path) -> tuple[int, list[str]]:
    objects = 0
    errors: list[str] = []
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            errors.append(f"{path}:{line_number}: expected 5 fields")
            continue
        try:
            class_id = int(fields[0])
            coordinates = [float(value) for value in fields[1:]]
        except ValueError:
            errors.append(f"{path}:{line_number}: non-numeric value")
            continue
        if class_id != 0:
            errors.append(f"{path}:{line_number}: only class 0 (drone) is allowed")
        if not all(0.0 <= value <= 1.0 for value in coordinates):
            errors.append(f"{path}:{line_number}: coordinates must be in 0..1")
        if coordinates[2] <= 0.0 or coordinates[3] <= 0.0:
            errors.append(f"{path}:{line_number}: width/height must be positive")
        objects += 1
    return objects, errors


def validate_dataset(root: Path) -> tuple[dict[str, dict[str, int]], list[str]]:
    report: dict[str, dict[str, int]] = {}
    errors: list[str] = []
    digest_locations: dict[str, list[str]] = defaultdict(list)

    for split in SPLITS:
        image_dir = root / "images" / split
        label_dir = root / "labels" / split
        if not image_dir.is_dir():
            errors.append(f"missing directory: {image_dir}")
            continue
        if not label_dir.is_dir():
            errors.append(f"missing directory: {label_dir}")
            continue

        images = sorted(
            path for path in image_dir.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )
        counters = Counter(images=len(images), labels=0, objects=0, negatives=0)
        image_stems = {path.stem for path in images}

        for image in images:
            label = label_dir / f"{image.stem}.txt"
            if not label.is_file():
                errors.append(f"missing label for image: {image}")
                continue
            counters["labels"] += 1
            object_count, label_errors = validate_label(label)
            counters["objects"] += object_count
            counters["negatives"] += int(object_count == 0)
            errors.extend(label_errors)
            digest_locations[image_digest(image)].append(f"{split}/{image.name}")

        for label in label_dir.glob("*.txt"):
            if label.stem not in image_stems:
                errors.append(f"label has no matching image: {label}")

        report[split] = dict(counters)

    for locations in digest_locations.values():
        splits = {location.split("/", 1)[0] for location in locations}
        if len(splits) > 1:
            errors.append("duplicate image crosses splits: " + ", ".join(locations))

    return report, errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=Path("dataset"))
    args = parser.parse_args()

    report, errors = validate_dataset(args.dataset.resolve())
    for split in SPLITS:
        stats = report.get(split)
        if stats:
            print(
                f"{split}: images={stats['images']} labels={stats['labels']} "
                f"objects={stats['objects']} negatives={stats['negatives']}"
            )
    if errors:
        print(f"FAILED: {len(errors)} issue(s)")
        for error in errors:
            print("-", error)
        return 1
    if any(report.get(split, {}).get("images", 0) == 0 for split in SPLITS):
        print("FAILED: train, val and test must each contain images")
        return 1
    print("OK: dataset structure and labels are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

