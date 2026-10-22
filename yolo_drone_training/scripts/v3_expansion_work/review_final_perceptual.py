"""Screen cross-source/sequence near duplicates in the final staged dataset."""

from __future__ import annotations

import csv
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_new")
WORK = Path(r"C:\Users\admin\Documents\Codex\2026-10-22\except-specified-u-yolo-v2-1\work")


def phash(image: np.ndarray) -> int:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dct = cv2.dct(np.float32(cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA)))[:8, :8]
    median = np.median(dct.ravel()[1:])
    return sum(int(bit) << i for i, bit in enumerate((dct > median).ravel()))


def main() -> None:
    with (ROOT / "dataset_v3_manifest.csv").open(newline="", encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))
    for row in rows:
        image = cv2.imread(str(ROOT / row["image"]))
        row["phash"] = phash(image)
    buckets = defaultdict(list)
    pairs = []
    for i, row in enumerate(rows):
        candidate_indices = set()
        for part in range(4):
            bucket = (part, (row["phash"] >> (16 * part)) & 0xFFFF)
            candidate_indices.update(buckets[bucket])
            buckets[bucket].append(i)
        for j in candidate_indices:
            previous = rows[j]
            if previous["source_group"] == row["source_group"]:
                continue
            if previous["source"] not in ("AOD-4", "Halmstad-new-RGB") and row["source"] not in ("AOD-4", "Halmstad-new-RGB"):
                continue
            distance = (row["phash"] ^ previous["phash"]).bit_count()
            if distance <= 4:
                pairs.append({
                    "distance": distance,
                    "source_a": previous["source"],
                    "split_a": previous["split"],
                    "group_a": previous["source_group"],
                    "image_a": previous["image"],
                    "source_b": row["source"],
                    "split_b": row["split"],
                    "group_b": row["source_group"],
                    "image_b": row["image"],
                })
    with (WORK / "final_near_duplicate_candidates.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(pairs[0]))
        writer.writeheader()
        writer.writerows(pairs)
    counts = Counter(tuple(sorted((x["source_a"], x["source_b"]))) for x in pairs)
    print("cross-group perceptual candidates", len(pairs), counts)
    print("cross-split", sum(x["split_a"] != x["split_b"] for x in pairs))


if __name__ == "__main__":
    main()
