"""Replace AOD-4's Anti-UAV repackaged timestamp frames with original paired frames."""
import csv
from pathlib import Path

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work")
with (ROOT / "dataset_v3_manifest.csv").open(newline="", encoding="utf-8-sig") as file:
    rows = list(csv.DictReader(file))
removed = []
for row in rows:
    if not row["source_group"].startswith("aod4:timestamp:"):
        continue
    if row["split"] == "regression_test":
        raise RuntimeError("Never modify regression_test")
    for field in ("image", "label"):
        path = (ROOT / row[field]).resolve()
        if not path.is_relative_to(ROOT.resolve()):
            raise RuntimeError(path)
        path.unlink(missing_ok=True)
    removed.append({"image": row["image"], "label": row["label"], "source_group": row["source_group"],
                    "old_split": row["split"], "reason": "AOD-4 frame is repackaged Anti-UAV300; replaced by original synchronized RGB/IR videos"})
with (ROOT / "aod_anti_overlap_replacements.csv").open("w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=list(removed[0]))
    writer.writeheader()
    writer.writerows(removed)
print("Removed repackaged AOD frames from candidate only:", len(removed))
