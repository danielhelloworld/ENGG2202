#!/usr/bin/env python3
"""Create the local, Git-ignored workspace for UAV data and annotations."""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    directories = [
        PROJECT_ROOT / "raw" / "positive_sessions",
        PROJECT_ROOT / "raw" / "negative_sessions",
        PROJECT_ROOT / "dataset" / "incoming" / "positive",
        PROJECT_ROOT / "dataset" / "incoming" / "negative",
        PROJECT_ROOT / "dataset" / "calibration" / "images",
    ]
    for kind in ("images", "labels"):
        for split in ("train", "val", "test"):
            directories.append(PROJECT_ROOT / "dataset" / kind / split)

    for directory in directories:
        directory.mkdir(parents=True, exist_ok=True)
        print(directory.relative_to(PROJECT_ROOT))

    print("Dataset workspace is ready. Raw data and labels remain local only.")


if __name__ == "__main__":
    main()
