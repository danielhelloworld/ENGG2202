#!/usr/bin/env python3
"""Split Halmstad sessions and convert them to a one-class YOLO dataset."""

from __future__ import annotations

import argparse
import json
import random
from collections import Counter, defaultdict
from pathlib import Path

import cv2
from mcos_decoder import load_groundtruth


def align_boxes(boxes, frame_count: int, is_drone: bool, source: Path):
    if not is_drone:
        return [None] * frame_count
    if len(boxes) == frame_count:
        return boxes
    if len(boxes) == frame_count + 1 and boxes[0] is None:
        return boxes[1:]
    raise ValueError(
        f"{source.name}: {len(boxes)} labels do not match {frame_count} frames"
    )


def yolo_line(box, width: int, height: int) -> str | None:
    if box is None:
        return None
    x, y, box_width, box_height = map(float, box)
    left = max(0.0, min(float(width), x))
    top = max(0.0, min(float(height), y))
    right = max(0.0, min(float(width), x + box_width))
    bottom = max(0.0, min(float(height), y + box_height))
    clipped_width, clipped_height = right - left, bottom - top
    if clipped_width < 1.0 or clipped_height < 1.0:
        return None
    center_x = (left + right) / (2 * width)
    center_y = (top + bottom) / (2 * height)
    return (
        f"0 {center_x:.8f} {center_y:.8f} "
        f"{clipped_width / width:.8f} {clipped_height / height:.8f}\n"
    )


SPLIT_RATIOS = {
    "train": 0.70,
    "val": 0.15,
    "test": 0.10,
    "calibration": 0.05,
}


def split_sessions(videos: list[Path], seed: int) -> dict[str, str]:
    """Assign complete sessions to stratified, deterministic splits."""
    by_class: dict[str, list[Path]] = defaultdict(list)
    for video in videos:
        by_class[video.stem.split("_")[1]].append(video)

    assignments: dict[str, str] = {}
    for source_class, class_videos in sorted(by_class.items()):
        shuffled = sorted(class_videos)
        random.Random(f"{seed}:{source_class}").shuffle(shuffled)
        total = len(shuffled)
        train_end = round(total * SPLIT_RATIOS["train"])
        val_end = train_end + round(total * SPLIT_RATIOS["val"])
        test_end = val_end + round(total * SPLIT_RATIOS["test"])
        boundaries = (
            ("train", 0, train_end),
            ("val", train_end, val_end),
            ("test", val_end, test_end),
            ("calibration", test_end, total),
        )
        for split, start, end in boundaries:
            for video in shuffled[start:end]:
                assignments[video.stem] = split
    return assignments


def convert_video(
    video: Path,
    images_dir: Path,
    labels_dir: Path | None,
    stride: int,
):
    label_source = video.with_name(video.stem + "_LABELS.mat")
    if not label_source.is_file():
        raise FileNotFoundError(f"missing annotation: {label_source}")

    capture = cv2.VideoCapture(str(video))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if frame_count <= 0 or width <= 0 or height <= 0:
        capture.release()
        raise ValueError(f"cannot read video metadata: {video}")

    is_drone = video.stem.startswith("V_DRONE_")
    boxes = align_boxes(load_groundtruth(label_source), frame_count, is_drone, video)
    written = positives = 0
    for frame_index in range(frame_count):
        ok, frame = capture.read()
        if not ok:
            capture.release()
            raise ValueError(f"{video.name}: failed to read frame {frame_index}")
        if frame_index % stride:
            continue
        stem = f"halmstad_{video.stem.lower()}_f{frame_index:06d}"
        image_path = images_dir / f"{stem}.jpg"
        if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            capture.release()
            raise OSError(f"failed to write {image_path}")
        label = yolo_line(boxes[frame_index], width, height)
        if labels_dir is not None:
            label_path = labels_dir / f"{stem}.txt"
            label_path.write_text(label or "", encoding="utf-8")
        written += 1
        positives += int(label is not None)
    capture.release()
    return {
        "session": video.stem,
        "source_class": video.stem.split("_")[1],
        "source_frames": frame_count,
        "width": width,
        "height": height,
        "fps": fps,
        "stride": stride,
        "written_images": written,
        "positive_images": positives,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, default=Path("dataset"))
    parser.add_argument("--stride", type=int, default=15)
    parser.add_argument("--seed", type=int, default=2202)
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="write the session split manifest without extracting frames",
    )
    args = parser.parse_args()
    if args.stride < 1:
        raise SystemExit("stride must be positive")

    videos = sorted(args.source.resolve().glob("V_*.mp4"))
    if not videos:
        raise SystemExit(f"no visible-light videos found in {args.source}")

    dataset = args.dataset.resolve()
    assignments = split_sessions(videos, args.seed)
    split_counts = Counter(assignments.values())
    class_split_counts = Counter(
        (video.stem.split("_")[1], assignments[video.stem]) for video in videos
    )
    split_plan = {
        "seed": args.seed,
        "stride": args.stride,
        "ratios": SPLIT_RATIOS,
        "sessions": [
            {
                "session": video.stem,
                "source_class": video.stem.split("_")[1],
                "split": assignments[video.stem],
            }
            for video in videos
        ],
    }
    dataset.mkdir(parents=True, exist_ok=True)
    (dataset / "halmstad_split.json").write_text(
        json.dumps(split_plan, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print("session split:", dict(sorted(split_counts.items())))
    for source_class in sorted({key[0] for key in class_split_counts}):
        print(
            source_class.lower() + ":",
            {
                split: class_split_counts[(source_class, split)]
                for split in SPLIT_RATIOS
            },
        )
    if args.plan_only:
        print(f"wrote split plan to {dataset / 'halmstad_split.json'}")
        return

    output_directories = []
    for split in ("train", "val", "test"):
        output_directories.extend(
            [dataset / "images" / split, dataset / "labels" / split]
        )
    output_directories.append(dataset / "calibration" / "images")
    for directory in output_directories:
        if directory.exists() and any(directory.iterdir()):
            raise SystemExit(f"output is not empty: {directory}")
    for directory in output_directories:
        directory.mkdir(parents=True, exist_ok=True)

    records = []
    for video in videos:
        split = assignments[video.stem]
        if split == "calibration":
            images_dir = dataset / "calibration" / "images"
            labels_dir = None
        else:
            images_dir = dataset / "images" / split
            labels_dir = dataset / "labels" / split
        record = convert_video(video, images_dir, labels_dir, args.stride)
        record["split"] = split
        records.append(record)
        print(json.dumps(record, sort_keys=True))
    (dataset / "halmstad_import_manifest.json").write_text(
        json.dumps(records, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    image_counts = Counter()
    positive_counts = Counter()
    for record in records:
        image_counts[record["split"]] += record["written_images"]
        positive_counts[record["split"]] += record["positive_images"]
    print("written images:", dict(sorted(image_counts.items())))
    print("positive images:", dict(sorted(positive_counts.items())))
    print(f"wrote {sum(image_counts.values())} images to {dataset}")


if __name__ == "__main__":
    main()
