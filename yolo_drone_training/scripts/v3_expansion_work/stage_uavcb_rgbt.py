"""Convert the complete official UAV-CB RGB/thermal archive to paired YOLO labels."""

from __future__ import annotations

import csv
import hashlib
import json
import zipfile
from pathlib import Path

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2")
ZIP = ROOT / "raw_downloads/official_github/UAV-CB/UAV-CB_full.zip"
OUT = ROOT / "dataset_v3_expansion_work"

# Keep each continuous background+aircraft series in exactly one split.
VAL = {"cloud_visible_1", "building_visible_2"}
FINAL = {"vegetation_visible_2", "powerline_visible_2", "ground_visible_1"}


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def label_from_json(data: bytes) -> tuple[str, int, int]:
    obj = json.loads(data)
    width, height = int(obj["imageWidth"]), int(obj["imageHeight"])
    lines = []
    for shape in obj["shapes"]:
        if shape["label"].lower() != "drone":
            raise ValueError(f"unexpected class {shape['label']}")
        xs = [float(point[0]) for point in shape["points"]]
        ys = [float(point[1]) for point in shape["points"]]
        x1, x2, y1, y2 = min(xs), max(xs), min(ys), max(ys)
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError(f"invalid UAV-CB box: {x1,x2,y1,y2,width,height}")
        lines.append(f"0 {(x1+x2)/(2*width):.8f} {(y1+y2)/(2*height):.8f} {(x2-x1)/width:.8f} {(y2-y1)/height:.8f}\n")
    if len(lines) != 1:
        raise ValueError(f"expected one box, got {len(lines)}")
    return "".join(lines), width, height


def main() -> None:
    records = []
    with zipfile.ZipFile(ZIP) as archive:
        paths = set(archive.namelist())
        visible = sorted(x for x in paths if x.endswith(".jpg") and "/visible/" in x)
        if len(visible) != 3393:
            raise RuntimeError(f"expected 3393 pairs, found {len(visible)}")
        for index, rgb_path in enumerate(visible, 1):
            stem = Path(rgb_path).stem
            parts = stem.split("_")
            group = "_".join(parts[:3])
            split = "final_test" if group in FINAL else "val" if group in VAL else "train"
            thermal_path = rgb_path.replace("/visible/", "/thermal/").replace("_visible_", "_thermal_")
            rgb_json = str(Path(rgb_path).with_suffix(".json")).replace("\\", "/")
            thermal_json = str(Path(thermal_path).with_suffix(".json")).replace("\\", "/")
            for path in (thermal_path, rgb_json, thermal_json):
                if path not in paths:
                    raise FileNotFoundError(path)
            rgb_data, th_data = archive.read(rgb_path), archive.read(thermal_path)
            rgb_label, rw, rh = label_from_json(archive.read(rgb_json))
            th_label, tw, th = label_from_json(archive.read(thermal_json))
            pair_id = "uavcb_" + stem.replace("_visible_", "_")
            folder = OUT / "rgbt" / split
            folder.mkdir(parents=True, exist_ok=True)
            rgb_out, th_out = folder / f"{pair_id}_rgb.jpg", folder / f"{pair_id}_thermal.jpg"
            rl_out, tl_out = folder / f"{pair_id}_rgb.txt", folder / f"{pair_id}_thermal.txt"
            rgb_out.write_bytes(rgb_data)
            th_out.write_bytes(th_data)
            rl_out.write_text(rgb_label, encoding="utf-8")
            tl_out.write_text(th_label, encoding="utf-8")
            records.append({
                "pair_id": pair_id, "split": split, "source": "UAV-CB", "source_group": "uavcb:" + group,
                "rgb": rgb_out.relative_to(OUT).as_posix(), "thermal": th_out.relative_to(OUT).as_posix(),
                "rgb_label": rl_out.relative_to(OUT).as_posix(), "thermal_label": tl_out.relative_to(OUT).as_posix(),
                "rgb_source": rgb_path, "thermal_source": thermal_path,
                "rgb_sha256": digest(rgb_data), "thermal_sha256": digest(th_data),
                "rgb_width": rw, "rgb_height": rh, "thermal_width": tw, "thermal_height": th,
                "positive": 1,
            })
            if index % 500 == 0:
                print("UAV-CB staged", index, flush=True)
    output = OUT / "uavcb_pair_manifest.csv"
    with output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    print("UAV-CB complete", len(records), {s: sum(r["split"] == s for r in records) for s in ("train", "val", "final_test")})


if __name__ == "__main__":
    main()
