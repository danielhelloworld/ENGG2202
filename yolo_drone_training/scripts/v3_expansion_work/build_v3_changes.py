"""Trace v2->new-v3 and previous-v3->new-v3 file-level changes."""

from __future__ import annotations

import csv
from pathlib import Path

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2")
WORK = Path(r"C:\Users\admin\Documents\Codex\2026-10-22\except-specified-u-yolo-v2-1\work")


def load(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as file:
        return list(csv.DictReader(file))


def main() -> None:
    previous = ROOT / "dataset_v3_previous"
    current = ROOT / "dataset_v3_new"
    rows = []
    for row in load(previous / "dataset_v3_changes.csv"):
        if row["action"] == "added" and "__v3_" in row["new_file"]:
            continue
        rows.append({"comparison": "v2_to_v3", **row})
    for row in load(WORK / "aod4_staged" / "staging_manifest.csv"):
        rows.append({
            "comparison": "v2_to_v3",
            "action": "added",
            "file": "",
            "old_split": "",
            "new_split": row["split"],
            "new_file": row["image"],
            "source": "AOD-4",
            "source_file": row["official_zip_path"],
            "source_group": row["source_group"],
            "reason": "official CC BY 4.0 real image; video-level split; temporal thinning; drone-only conversion or hard negative",
        })
    for row in load(WORK / "halmstad_staged" / "selected_manifest.csv"):
        rows.append({
            "comparison": "v2_to_v3",
            "action": "added",
            "file": "",
            "old_split": "",
            "new_split": row["split"],
            "new_file": row["image"],
            "source": "Halmstad-new-RGB",
            "source_file": row["video"] + ".mp4#frame=" + row["frame"],
            "source_group": row["source_group"],
            "reason": "official CC0 RGB sequence absent from dataset_v2; 2-second frame spacing",
        })
    for row in load(previous / "dataset_v3_changes.csv"):
        if row["action"] != "added" or "__v3_" not in row["new_file"]:
            continue
        rows.append({
            "comparison": "previous_v3_to_new_v3",
            "action": "removed",
            "file": row["new_file"],
            "old_split": "train",
            "new_split": "",
            "new_file": "",
            "source": row["source"],
            "source_file": row["source_file"],
            "source_group": row["source_group"],
            "reason": "remove pre-generated derived view; use Ultralytics online augmentation",
        })
    for row in load(WORK / "aod4_staged" / "staging_manifest.csv"):
        rows.append({
            "comparison": "previous_v3_to_new_v3",
            "action": "added",
            "file": "",
            "old_split": "",
            "new_split": row["split"],
            "new_file": row["image"],
            "source": "AOD-4",
            "source_file": row["official_zip_path"],
            "source_group": row["source_group"],
            "reason": "new official real source with sequence-level split and class remapping",
        })
    for row in load(WORK / "halmstad_staged" / "selected_manifest.csv"):
        rows.append({
            "comparison": "previous_v3_to_new_v3",
            "action": "added",
            "file": "",
            "old_split": "",
            "new_split": row["split"],
            "new_file": row["image"],
            "source": "Halmstad-new-RGB",
            "source_file": row["video"] + ".mp4#frame=" + row["frame"],
            "source_group": row["source_group"],
            "reason": "unused official RGB sequence; 2-second frame spacing",
        })
    with (current / "dataset_v3_changes.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    from collections import Counter
    print(Counter((r["comparison"], r["action"]) for r in rows))


if __name__ == "__main__":
    main()
