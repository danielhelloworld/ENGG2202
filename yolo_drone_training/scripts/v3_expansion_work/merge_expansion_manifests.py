"""Merge RGB and paired audits into one image-level dataset inventory."""
from __future__ import annotations

import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work")


def read(path):
    with path.open(newline="", encoding="utf-8-sig") as file:
        return list(csv.DictReader(file))


def write(path, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    main_rows = read(ROOT / "dataset_v3_manifest.csv")
    if any(r.get("pair_id") for r in main_rows):
        main_rows = [r for r in main_rows if not r.get("pair_id")]
    pairs = read(ROOT / "dataset_v3_rgbt_manifest.csv")
    records = [{**row, "modality": "RGB", "pair_id": ""} for row in main_rows]
    for pair in pairs:
        for modality in ("rgb", "thermal"):
            records.append({
                "image": pair[modality], "label": pair[modality + "_label"],
                "source": pair["source"], "source_subtype": "paired_video_or_curated_frame",
                "source_file": pair["pair_id"] + ":" + modality,
                "source_group": pair["source_group"], "split": pair["split"],
                "image_sha256": pair[modality + "_sha256"], "label_sha256": pair[modality + "_label_sha256"],
                "width": pair[modality + "_width"], "height": pair[modality + "_height"],
                "positive": int(int(pair[modality + "_objects"]) > 0), "objects": pair[modality + "_objects"],
                "bytes": (ROOT / pair[modality]).stat().st_size,
                "modality": "RGB" if modality == "rgb" else "thermal", "pair_id": pair["pair_id"],
            })
    write(ROOT / "dataset_v3_manifest.csv", records)
    statistics = defaultdict(Counter)
    sizes = Counter()
    hashes = defaultdict(list)
    groups = defaultdict(set)
    for row in records:
        key = (row["split"], row["source"], row["modality"])
        statistics[key]["images"] += 1
        statistics[key]["positive_images"] += int(row["positive"])
        statistics[key]["negative_images"] += not int(row["positive"])
        statistics[key]["objects"] += int(row["objects"])
        statistics[key]["bytes"] += int(row["bytes"])
        hashes[row["image_sha256"]].append(row)
        groups[row["source_group"]].add(row["split"])
        width, height = int(row["width"]), int(row["height"])
        for line in (ROOT / row["label"]).read_text().splitlines():
            _, _, _, bw, bh = line.split()
            size = math.sqrt(float(bw)*width*float(bh)*height)*640/max(width,height)
            band = "tiny_lt20" if size < 20 else "small_20_47" if size < 48 else "medium_48_95" if size < 96 else "large_96_159" if size < 160 else "very_large_ge160"
            sizes[(row["split"], row["modality"], band)] += 1
    stat_rows = [{"split": k[0], "source": k[1], "modality": k[2], **v} for k,v in sorted(statistics.items())]
    write(ROOT / "dataset_v3_source_statistics.csv", stat_rows)
    write(ROOT / "dataset_v3_size_distribution.csv", [{"split": k[0], "modality": k[1], "size_bucket": k[2], "objects":v} for k,v in sorted(sizes.items())])
    for split in ("train", "val", "final_test", "regression_test"):
        images = ["./" + row["image"] for row in records if row["split"] == split and row["modality"] == "RGB"]
        (ROOT / f"rgb_{split}.txt").write_text("\n".join(images) + "\n", encoding="utf-8")
    (ROOT / "data.yaml").write_text(f"path: {ROOT.as_posix()}\ntrain: rgb_train.txt\nval: rgb_val.txt\ntest: rgb_final_test.txt\nchannels: 3\nnames:\n  0: drone\n", encoding="utf-8")
    (ROOT / "data_rgbt_regression.yaml").write_text(f"path: {ROOT.as_posix()}\ntrain: rgbt_train.txt\nval: rgb_regression_test.txt\ntest: rgb_regression_test.txt\nchannels: 4\nallow_missing_thermal: true\nnames:\n  0: drone\n", encoding="utf-8")
    original_audit = json.loads((ROOT / "dataset_v3_audit.txt").read_text().split("\n",1)[1])
    paired_audit = json.loads((ROOT / "dataset_v3_rgbt_audit.txt").read_text().split("\n",1)[1])
    duplicates = [[r["image"] for r in v] for v in hashes.values() if len(v)>1]
    cross_duplicates = [v for v in hashes.values() if len({r["split"] for r in v}) > 1]
    leaks = {k:sorted(v) for k,v in groups.items() if len(v)>1}
    machine_pass = original_audit["status"] == "PASS" and paired_audit["status"] == "PASS" and not duplicates and not leaks
    audit = {
        "status": "PASS" if machine_pass else "FAIL", "scope": "full machine integrity/provenance audit",
        "images": len(records), "paired_samples": len(pairs), "counts": stat_rows,
        "exact_duplicate_image_groups": len(duplicates), "cross_split_duplicate_image_groups": len(cross_duplicates),
        "cross_split_source_groups": len(leaks), "regression_test_unchanged": original_audit["regression_test_unchanged"],
        "visual_review_status": "PENDING", "ready_for_formal_training": False,
        "issues": {"duplicate_images": duplicates[:100], "source_group_leaks": leaks, "paired_issues": paired_audit["issues"]},
    }
    (ROOT / "dataset_v3_audit.txt").write_text("dataset_v3 RGB + thermal audit\n" + json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k:v for k,v in audit.items() if k not in ("counts", "issues")}, indent=2))
    print("duplicates", duplicates[:3])


if __name__ == "__main__":
    main()
