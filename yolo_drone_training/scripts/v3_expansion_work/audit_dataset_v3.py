"""Full integrity and provenance audit for the staged v3 dataset."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2")
DATASET = Path(os.environ.get("DRONE_DATASET_ROOT", str(ROOT / "dataset_v3_new")))
PREVIOUS = ROOT / "dataset_v3_previous"
if not PREVIOUS.is_dir():
    PREVIOUS = DATASET
V2 = ROOT / "dataset_v2"
WORK = Path(r"C:\Users\admin\Documents\Codex\2026-10-22\except-specified-u-yolo-v2-1\work")
SPLITS = ("train", "val", "final_test", "regression_test")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as file:
        return list(csv.DictReader(file))


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_maps() -> dict[str, dict[str, str]]:
    old = {r["image"]: r for r in read_csv(PREVIOUS / "dataset_v3_manifest.csv")}
    aod = {r["image"]: r for r in read_csv(WORK / "aod4_staged" / "staging_manifest.csv")}
    hal = {r["image"]: r for r in read_csv(WORK / "halmstad_staged" / "selected_manifest.csv")}
    dut_path = DATASET / "dut_manifest.csv"
    dut = {r["image"]: r for r in read_csv(dut_path)} if dut_path.exists() else {}
    return {"old": old, "aod": aod, "hal": hal, "dut": dut}


def size_bucket(size: float) -> str:
    if size < 20:
        return "tiny_lt20"
    if size < 48:
        return "small_20_47"
    if size < 96:
        return "medium_48_95"
    if size < 160:
        return "large_96_159"
    return "very_large_ge160"


def main() -> None:
    maps = source_maps()
    issues = []
    manifest = []
    stats = Counter()
    sizes = Counter()
    hashes = defaultdict(list)
    groups = defaultdict(set)
    review_rows = []
    for split in SPLITS:
        image_dir = DATASET / "images" / split
        label_dir = DATASET / "labels" / split
        images = sorted(p for p in image_dir.iterdir() if p.is_file())
        labels = sorted(p for p in label_dir.iterdir() if p.is_file())
        if len(images) != len(labels):
            issues.append(f"{split}: image/label count mismatch {len(images)}/{len(labels)}")
        image_stems = {p.stem for p in images}
        label_stems = {p.stem for p in labels}
        for stem in sorted(image_stems ^ label_stems):
            issues.append(f"{split}: missing image or label for {stem}")
        for image_path in images:
            label_path = label_dir / (image_path.stem + ".txt")
            if not label_path.is_file():
                continue
            relative_image = image_path.relative_to(DATASET).as_posix()
            relative_label = label_path.relative_to(DATASET).as_posix()
            binary = image_path.read_bytes()
            image_sha = hashlib.sha256(binary).hexdigest()
            cached = maps["old"].get(relative_image, {})
            if cached.get("image_sha256") == image_sha:
                h, w = int(cached["height"]), int(cached["width"])
            else:
                image = cv2.imdecode(np.frombuffer(binary, np.uint8), cv2.IMREAD_COLOR)
                if image is None or image.size == 0:
                    issues.append(f"corrupt image: {relative_image}")
                    continue
                h, w = image.shape[:2]
            label_sha = sha(label_path)
            hashes[image_sha].append(relative_image)
            lines = label_path.read_text(encoding="utf-8-sig").splitlines()
            boxes = []
            for index, line in enumerate(lines, 1):
                tokens = line.split()
                if len(tokens) != 5:
                    issues.append(f"bad label columns: {relative_label}:{index}")
                    continue
                try:
                    class_id = int(tokens[0])
                    cx, cy, bw, bh = map(float, tokens[1:])
                except ValueError:
                    issues.append(f"non-numeric label: {relative_label}:{index}")
                    continue
                if class_id != 0 or not all(map(math.isfinite, (cx, cy, bw, bh))):
                    issues.append(f"invalid class/number: {relative_label}:{index}")
                    continue
                if bw <= 0 or bh <= 0 or cx - bw / 2 < -1e-6 or cy - bh / 2 < -1e-6 or cx + bw / 2 > 1 + 1e-6 or cy + bh / 2 > 1 + 1e-6:
                    issues.append(f"out-of-bounds/zero box: {relative_label}:{index}")
                    continue
                box = (cx, cy, bw, bh)
                if box in boxes:
                    issues.append(f"duplicate box: {relative_label}:{index}")
                boxes.append(box)
                size = math.sqrt(bw * w * bh * h) * 640 / max(w, h)
                sizes[(split, size_bucket(size))] += 1
                if size < 20:
                    review_rows.append((relative_image, "tiny", round(size, 2)))
            if len(boxes) > 1:
                review_rows.append((relative_image, "multi_object", len(boxes)))
            stats[(split, "images")] += 1
            stats[(split, "positive" if boxes else "negative")] += 1
            stats[(split, "objects")] += len(boxes)
            if image_path.name in maps["aod"]:
                meta = maps["aod"][image_path.name]
                source = "AOD-4"
                source_file = meta["official_zip_path"]
                group = meta["source_group"]
                subtype = meta["source_subtype"]
            elif image_path.name in maps["hal"]:
                meta = maps["hal"][image_path.name]
                source = "Halmstad-new-RGB"
                source_file = meta["video"] + ".mp4#frame=" + meta["frame"]
                group = meta["source_group"]
                subtype = meta["class_source"]
            elif relative_image in maps["dut"]:
                meta = maps["dut"][relative_image]
                source = meta["source"]
                source_file = meta["official_split"] + ".zip#" + meta["source_file"]
                group = meta["source_group"]
                subtype = "RGB_detection"
            elif relative_image in maps["old"]:
                meta = maps["old"][relative_image]
                source = meta["source"]
                source_file = meta["source_file"]
                group = meta["source_group"]
                subtype = "existing_v2"
            else:
                issues.append(f"missing provenance: {relative_image}")
                source = source_file = group = subtype = "unknown"
            groups[group].add(split)
            manifest.append({
                "image": relative_image,
                "label": relative_label,
                "source": source,
                "source_subtype": subtype,
                "source_file": source_file,
                "source_group": group,
                "split": split,
                "image_sha256": image_sha,
                "label_sha256": label_sha,
                "width": w,
                "height": h,
                "positive": int(bool(boxes)),
                "objects": len(boxes),
                "bytes": len(binary) + label_path.stat().st_size,
            })
    duplicates = {digest: files for digest, files in hashes.items() if len(files) > 1}
    if duplicates:
        issues.append(f"exact duplicate image groups: {len(duplicates)}")
    group_leaks = {group: sorted(splits) for group, splits in groups.items() if len(splits) > 1}
    if group_leaks:
        issues.append(f"source groups across splits: {len(group_leaks)}")
    old_test_ok = True
    for kind in ("images", "labels"):
        old_dir = V2 / kind / "test"
        new_dir = DATASET / kind / "regression_test"
        originals = sorted(p for p in old_dir.iterdir() if p.is_file())
        copies = sorted(p for p in new_dir.iterdir() if p.is_file())
        if [p.name for p in originals] != [p.name for p in copies]:
            old_test_ok = False
        else:
            for original, copy in zip(originals, copies):
                if sha(original) != sha(copy):
                    old_test_ok = False
                    break
    if not old_test_ok:
        issues.append("regression_test differs from dataset_v2/test")
    if stats[("regression_test", "images")] != 889:
        issues.append("regression_test count is not 889")
    if not stats[("final_test", "images")]:
        issues.append("final_test is empty")
    fields = list(manifest[0])
    with (DATASET / "dataset_v3_manifest.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(manifest)
    source_rows = []
    for (split, source, subtype), items in sorted(_group_by(manifest).items()):
        source_rows.append({
            "split": split,
            "source": source,
            "subtype": subtype,
            "images": len(items),
            "positive_images": sum(int(x["positive"]) for x in items),
            "negative_images": sum(not int(x["positive"]) for x in items),
            "objects": sum(int(x["objects"]) for x in items),
            "bytes": sum(int(x["bytes"]) for x in items),
        })
    with (DATASET / "dataset_v3_source_statistics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(source_rows[0]))
        writer.writeheader()
        writer.writerows(source_rows)
    with (DATASET / "dataset_v3_size_distribution.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["split", "size_bucket", "objects"])
        writer.writeheader()
        for (split, bucket), count in sorted(sizes.items()):
            writer.writerow({"split": split, "size_bucket": bucket, "objects": count})
    with (DATASET / "review_required.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["image", "review_reason", "value"])
        writer.writerows(review_rows)
    result = {
        "status": "PASS" if not issues else "FAIL",
        "images": len(manifest),
        "counts": {split: {key: stats[(split, key)] for key in ("images", "positive", "negative", "objects")} for split in SPLITS},
        "exact_duplicate_image_groups": len(duplicates),
        "cross_split_source_groups": len(group_leaks),
        "regression_test_unchanged": old_test_ok,
        "review_queue_rows": len(review_rows),
        "issues": issues[:100],
    }
    (DATASET / "dataset_v3_audit.txt").write_text("dataset_v3 full audit\n" + json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (WORK / "audit_result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


def _group_by(manifest: list[dict]) -> dict[tuple[str, str, str], list[dict]]:
    grouped = defaultdict(list)
    for row in manifest:
        grouped[(row["split"], row["source"], row["source_subtype"])].append(row)
    return grouped


if __name__ == "__main__":
    main()
