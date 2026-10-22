"""Ingest official DUT detection images; collapse its leaking splits into train."""

from __future__ import annotations

import csv
import hashlib
import io
import xml.etree.ElementTree as ET
import zipfile
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2")
RAW = ROOT / "raw_downloads/official_github/DUT-Anti-UAV"
OUT = ROOT / "dataset_v3_expansion_work"


def main() -> None:
    with (ROOT / "dataset_v3/dataset_v3_manifest.csv").open(newline="", encoding="utf-8-sig") as file:
        seen = {r["image_sha256"] for r in csv.DictReader(file)}
    output_images = OUT / "images/train"
    output_labels = OUT / "labels/train"
    records = []
    rejected = []
    stats = Counter()
    for official_split in ("train", "val", "test"):
        with zipfile.ZipFile(RAW / f"{official_split}.zip") as archive:
            image_names = sorted(n for n in archive.namelist() if n.endswith(".jpg"))
            for name in image_names:
                source_id = Path(name).stem
                raw_image = archive.read(name)
                digest = hashlib.sha256(raw_image).hexdigest()
                if digest in seen:
                    rejected.append((official_split, source_id, "exact SHA-256 duplicate"))
                    stats["exact_duplicates"] += 1
                    continue
                xml_name = name.replace("/img/", "/xml/").replace(".jpg", ".xml")
                try:
                    root = ET.fromstring(archive.read(xml_name))
                    decoded = cv2.imdecode(np.frombuffer(raw_image, np.uint8), cv2.IMREAD_COLOR)
                    if decoded is None:
                        raise ValueError("corrupt image")
                    height, width = decoded.shape[:2]
                    if (int(root.findtext("size/width")), int(root.findtext("size/height"))) != (width, height):
                        raise ValueError("XML/image dimensions differ")
                    lines = []
                    invalid_boxes = 0
                    for obj in root.findall("object"):
                        if obj.findtext("name", "").upper() != "UAV":
                            raise ValueError("non-UAV class")
                        bbox = obj.find("bndbox")
                        x1, y1, x2, y2 = [float(bbox.findtext(t)) for t in ("xmin", "ymin", "xmax", "ymax")]
                        if x2 <= x1 or y2 <= y1:
                            invalid_boxes += 1
                            continue
                        x1, x2 = max(0, x1), min(width, x2)
                        y1, y2 = max(0, y1), min(height, y2)
                        if x2 <= x1 or y2 <= y1:
                            raise ValueError("box entirely outside image")
                        line = f"0 {(x1+x2)/(2*width):.8f} {(y1+y2)/(2*height):.8f} {(x2-x1)/width:.8f} {(y2-y1)/height:.8f}\n"
                        if line not in lines:
                            lines.append(line)
                    if invalid_boxes and not lines:
                        raise ValueError("all annotated boxes are zero/inverted; do not infer a negative label")
                    target_stem = f"dut_{official_split}_{source_id}"
                    target_image = output_images / f"{target_stem}.jpg"
                    target_label = output_labels / f"{target_stem}.txt"
                    target_image.write_bytes(raw_image)
                    target_label.write_text("".join(lines), encoding="utf-8")
                    seen.add(digest)
                    records.append({
                        "image": target_image.relative_to(OUT).as_posix(),
                        "label": target_label.relative_to(OUT).as_posix(),
                        "source": "DUT-Anti-UAV", "source_file": name,
                        "source_group": "dut:all_unknown_origin", "official_split": official_split,
                        "split": "train", "sha256": digest, "width": width, "height": height,
                        "positive": int(bool(lines)), "objects": len(lines),
                        "correction_reason": "removed zero/inverted box" if invalid_boxes else "",
                    })
                    stats["kept"] += 1
                    stats["positive" if lines else "negative"] += 1
                    stats["objects"] += len(lines)
                    stats["corrected_images"] += int(bool(invalid_boxes))
                except Exception as error:
                    rejected.append((official_split, source_id, str(error)))
                    stats["rejected"] += 1
            print("DUT", official_split, "processed", len(image_names), dict(stats), flush=True)
    with (OUT / "dut_manifest.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    with (OUT / "dut_rejected.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("official_split", "source_id", "reason"))
        writer.writerows(rejected)
    print("DUT complete", dict(stats), "rejected", len(rejected))


if __name__ == "__main__":
    main()
