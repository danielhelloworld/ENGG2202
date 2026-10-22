"""Audit all paired samples and create reproducible fusion/RGB-only lists."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import cv2

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work")


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as file:
        return list(csv.DictReader(file))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_label(path):
    boxes = []
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        parts = line.split()
        if len(parts) != 5 or parts[0] != "0":
            raise ValueError(f"class/format: {path}")
        x, y, w, h = map(float, parts[1:])
        if not all(map(math.isfinite, (x, y, w, h))) or w <= 0 or h <= 0 or min(x-w/2, y-h/2) < -1e-6 or max(x+w/2, y+h/2) > 1+1e-6:
            raise ValueError(f"invalid/outside box: {path}")
        if (x,y,w,h) in boxes:
            raise ValueError(f"duplicate box: {path}")
        boxes.append((x,y,w,h))
    return boxes


def main():
    source_rows = read_csv(ROOT / "uavcb_pair_manifest.csv") + read_csv(ROOT / "anti300_pair_manifest.csv")
    cache_path = ROOT / "dataset_v3_rgbt_manifest.csv"
    cache = {r["pair_id"]: r for r in read_csv(cache_path)} if cache_path.exists() else {}
    records, issues = [], []
    stats = defaultdict(Counter)
    groups = defaultdict(set)
    pair_hashes = defaultdict(list)
    for i, row in enumerate(source_rows, 1):
        try:
            item = {key: row[key] for key in ("pair_id", "source", "source_group", "split", "rgb", "thermal", "rgb_label", "thermal_label")}
            for modality in ("rgb", "thermal"):
                image_path, label_path = ROOT / row[modality], ROOT / row[modality + "_label"]
                digest = sha(image_path)
                previous = cache.get(row["pair_id"], {})
                if previous.get(modality + "_sha256") == digest:
                    width, height = int(previous[modality + "_width"]), int(previous[modality + "_height"])
                else:
                    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
                    if image is None:
                        raise ValueError(f"corrupt/missing image: {image_path}")
                    height, width = image.shape[:2]
                if (width, height) != (int(row[modality + "_width"]), int(row[modality + "_height"])):
                    raise ValueError(f"image metadata mismatch: {image_path}")
                boxes = audit_label(label_path)
                item[modality + "_sha256"] = digest
                item[modality + "_label_sha256"] = sha(label_path)
                item[modality + "_width"], item[modality + "_height"] = width, height
                item[modality + "_objects"] = len(boxes)
            item["fusion_eligible"] = int(bool(item["rgb_objects"]) or not item["thermal_objects"])
            item["supervision"] = "RGB_coordinate_boxes" if item["fusion_eligible"] else "thermal_only_visibility_requires_RGB_coordinate_annotation"
            key = (item["split"], item["source"])
            stats[key]["pairs"] += 1
            stats[key]["rgb_positive"] += bool(item["rgb_objects"])
            stats[key]["thermal_positive"] += bool(item["thermal_objects"])
            stats[key]["both_negative"] += not (item["rgb_objects"] or item["thermal_objects"])
            stats[key]["fusion_eligible"] += item["fusion_eligible"]
            groups[item["source_group"]].add(item["split"])
            pair_hashes[(item["rgb_sha256"], item["thermal_sha256"])].append(item["pair_id"])
            records.append(item)
        except Exception as error:
            issues.append({"pair_id": row["pair_id"], "error": str(error)})
        if i % 1000 == 0:
            print("paired audit", i, "issues", len(issues), flush=True)
    duplicate_pairs = [v for v in pair_hashes.values() if len(v) > 1]
    leaks = {key: sorted(value) for key, value in groups.items() if len(value) > 1}
    if duplicate_pairs:
        issues.append({"duplicate_pair_groups": duplicate_pairs})
    if leaks:
        issues.append({"source_group_leaks": leaks})
    with (ROOT / "dataset_v3_rgbt_manifest.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    stats_rows = [{"split": key[0], "source": key[1], **value} for key, value in sorted(stats.items())]
    with (ROOT / "dataset_v3_rgbt_statistics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(stats_rows[0]))
        writer.writeheader()
        writer.writerows(stats_rows)
    for split in ("train", "val", "final_test"):
        eligible = ["./" + r["rgb"] for r in records if r["split"] == split and r["fusion_eligible"]]
        (ROOT / f"rgbt_{split}.txt").write_text("\n".join(eligible) + "\n", encoding="utf-8")
    yaml = f"path: {ROOT.as_posix()}\ntrain: rgbt_train.txt\nval: rgbt_val.txt\ntest: rgbt_final_test.txt\nchannels: 4\nnames:\n  0: drone\n"
    (ROOT / "data_rgbt.yaml").write_text(yaml, encoding="utf-8")
    result = {"status": "PASS" if not issues else "FAIL", "pairs": len(records), "images": len(records)*2,
              "counts": stats_rows, "cross_split_source_groups": len(leaks), "exact_duplicate_pair_groups": len(duplicate_pairs), "issues": issues}
    (ROOT / "dataset_v3_rgbt_audit.txt").write_text("dataset_v3 paired RGB-T audit\n" + json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
