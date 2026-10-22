"""Stage real AOD-4 frames with video-level splits and temporal thinning."""

from __future__ import annotations

import csv
import hashlib
import json
import re
import zipfile
from collections import defaultdict
from pathlib import Path

WORK = Path(r"C:\Users\admin\Documents\Codex\2026-10-22\except-specified-u-yolo-v2-1\work")
ARCHIVE = Path(r"C:\Users\admin\Documents\drone_yolo_v2\raw_downloads\aod4_official\AOD 4.zip")
DEST = WORK / "aod4_staged"
FINAL_TIMES = {"20190926_141816", "20190925_205804"}
VAL_TIMES = {"20190925_131530", "20190926_183400"}
SKIP_YT_POS = {"aod4:yt:CwqG3k5FKFU", "aod4:yt:Ov1O5AzpMB0", "aod4:yt:false", "aod4:yt:Y4jvy5q_PXg"}


def frame_number(row: dict[str, str]) -> int:
    match = re.search(r"[_-](\d+)_jpg$", row["original_name"])
    return int(match.group(1)) if match else -1


def thin(rows: list[dict[str, str]], gap: int, cap: int | None = None) -> list[dict[str, str]]:
    selected = []
    last = -10**8
    for row in sorted(rows, key=lambda r: (frame_number(r), r["filename"])):
        n = frame_number(row)
        if n - last >= gap:
            selected.append(row)
            last = n
    if cap is not None and len(selected) > cap:
        selected = [selected[round(i * (len(selected) - 1) / (cap - 1))] for i in range(cap)]
    return selected


def split_for(row: dict[str, str]) -> str:
    group = row["source_group"]
    if row["source_subtype"] == "timestamp_video":
        time = group.rsplit(":", 1)[-1]
        if time in FINAL_TIMES:
            return "final_test"
        if time in VAL_TIMES:
            return "val"
        return "train"
    # Entire YouTube video in one split, irrespective of positive/negative frames.
    bucket = int(hashlib.sha256(group.encode()).hexdigest()[:8], 16) % 10
    return "final_test" if bucket == 0 else "val" if bucket == 1 else "train"


def choose(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    by_stream: dict[str, list[dict[str, str]]] = defaultdict(list)
    by_video: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        subtype = row["source_subtype"]
        if subtype == "timestamp_video" and row["positive"] == "1":
            stream = re.sub(r"_\d+_jpg$", "", row["original_name"])
            by_stream[stream].append(row)
        elif subtype == "youtube_video":
            if row["positive"] == "0" or row["source_group"] not in SKIP_YT_POS:
                by_video[row["source_group"]].append(row)
    selected = []
    for stream, items in sorted(by_stream.items()):
        selected.extend(thin(items, gap=20))
    for video, items in sorted(by_video.items()):
        positives = [r for r in items if r["positive"] == "1"]
        negatives = [r for r in items if r["positive"] == "0"]
        selected.extend(thin(positives, gap=10, cap=35))
        selected.extend(thin(negatives, gap=30, cap=8))
    return selected


def yolo_lines(row: dict[str, str]) -> str:
    width, height = int(row["width"]), int(row["height"])
    lines = []
    for x, y, bw, bh in json.loads(row["drone_boxes"]):
        x, y, bw, bh = map(float, (x, y, bw, bh))
        if bw <= 0 or bh <= 0 or x < 0 or y < 0 or x + bw > width or y + bh > height:
            raise ValueError(f"invalid box: {row['filename']} {(x, y, bw, bh)}")
        lines.append(f"0 {(x + bw / 2) / width:.8f} {(y + bh / 2) / height:.8f} {bw / width:.8f} {bh / height:.8f}\n")
    return "".join(lines)


def main() -> None:
    with (WORK / "aod4_inventory.csv").open(newline="", encoding="utf-8") as file:
        candidates = choose(list(csv.DictReader(file)))
    (DEST / "images").mkdir(parents=True, exist_ok=True)
    (DEST / "labels").mkdir(parents=True, exist_ok=True)
    records = []
    with zipfile.ZipFile(ARCHIVE) as archive:
        for row in candidates:
            output_name = "aod4_" + row["filename"]
            image = DEST / "images" / output_name
            label = DEST / "labels" / (Path(output_name).stem + ".txt")
            image.write_bytes(archive.read(row["zip_path"]))
            label.write_text(yolo_lines(row), encoding="utf-8")
            records.append({
                "image": image.name,
                "label": label.name,
                "official_zip_path": row["zip_path"],
                "original_name": row["original_name"],
                "source_subtype": row["source_subtype"],
                "source_group": row["source_group"],
                "split": split_for(row),
                "positive": row["positive"],
            })
    with (DEST / "staging_manifest.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    print("staged", len(records), "positive", sum(r["positive"] == "1" for r in records))
    for split in ("train", "val", "final_test"):
        r = [r for r in records if r["split"] == split]
        print(split, len(r), "positive", sum(x["positive"] == "1" for x in r))


if __name__ == "__main__":
    main()
