"""Keep all UAV-CB captures in train because original recording IDs are absent."""
import csv
from pathlib import Path

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work").resolve()
path = ROOT / "uavcb_pair_manifest.csv"
rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
kept, changes = [], []
for row in rows:
    if row["pair_id"] == "uavcb_cloud_1_000383":
        for key in ("rgb", "thermal", "rgb_label", "thermal_label"):
            item = (ROOT / row[key]).resolve()
            if not item.is_relative_to(ROOT):
                raise RuntimeError(item)
            item.unlink()
        changes.append({"pair_id": row["pair_id"], "action": "removed", "old_split": row["split"], "new_split": "",
                        "reason": "exact RGB+thermal duplicate of uavcb_building_1_000772; both annotations visually checked"})
        continue
    old_split = row["split"]
    if old_split != "train":
        for key in ("rgb", "thermal", "rgb_label", "thermal_label"):
            old = (ROOT / row[key]).resolve()
            new = ROOT / "rgbt/train" / old.name
            if not old.is_relative_to(ROOT) or not new.resolve().is_relative_to(ROOT):
                raise RuntimeError("path outside dataset")
            old.rename(new)
            row[key] = new.relative_to(ROOT).as_posix()
        changes.append({"pair_id": row["pair_id"], "action": "moved", "old_split": old_split, "new_split": "train",
                        "reason": "recording IDs not supplied; cross-background exact duplicate confirms series grouping cannot prove split independence"})
    row["split"] = "train"
    row["original_series"] = row["source_group"]
    row["source_group"] = "uavcb:all_public_capture"
    kept.append(row)
with path.open("w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=list(kept[0]))
    writer.writeheader()
    writer.writerows(kept)
with (ROOT / "uavcb_regrouping_changes.csv").open("w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=list(changes[0]))
    writer.writeheader()
    writer.writerows(changes)
print("UAV-CB train pairs", len(kept), "changed pairs", len(changes))
