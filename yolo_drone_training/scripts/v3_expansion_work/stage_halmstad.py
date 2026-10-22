"""Stage 1 fps RGB frames from Halmstad sessions absent from dataset_v2."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import cv2

REPO = Path(r"C:\Users\admin\Documents\Codex\ENGG2202")
sys.path.insert(0, str(REPO / "yolo_drone_training" / "scripts"))
from mcos_decoder import load_groundtruth  # noqa: E402
from import_halmstad import align_boxes, yolo_line  # noqa: E402

WORK = Path(r"C:\Users\admin\Documents\Codex\2026-10-22\except-specified-u-yolo-v2-1\work")
SOURCE = WORK / "official_halmstad" / "Data" / "Video_V"
DEST = WORK / "halmstad_staged"


def main() -> None:
    images, labels = DEST / "images", DEST / "labels"
    images.mkdir(parents=True, exist_ok=True)
    labels.mkdir(parents=True, exist_ok=True)
    rows = []
    for video in sorted(SOURCE.glob("*.mp4")):
        cap = cv2.VideoCapture(str(video))
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if count <= 0 or fps <= 0:
            raise RuntimeError(f"bad video metadata: {video}")
        drone = video.stem.startswith("V_DRONE_")
        gt = align_boxes(
            load_groundtruth(video.with_name(video.stem + "_LABELS.mat")),
            count,
            drone,
            video,
        )
        indices = {min(count - 1, round(t * fps)) for t in range(int(count / fps) + 1)}
        for index in sorted(indices):
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f"cannot read {video.name} frame {index}")
            stem = f"halmstad_{video.stem.lower()}_f{index:06d}"
            image = images / f"{stem}.jpg"
            label = labels / f"{stem}.txt"
            if not cv2.imwrite(str(image), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError(f"cannot write {image}")
            line = yolo_line(gt[index], width, height)
            label.write_text(line or "", encoding="utf-8")
            area = 0.0
            if line:
                values = [float(x) for x in line.split()[1:]]
                area = values[2] * values[3]
            rows.append({
                "image": image.name,
                "video": video.stem,
                "frame": index,
                "time_s": round(index / fps, 3),
                "class_source": video.stem.split("_")[1],
                "positive": bool(line),
                "box_area_fraction": round(area, 8),
            })
        cap.release()
    with (DEST / "staging_manifest.csv").open("w", newline="", encoding="utf-8") as out:
        writer = csv.DictWriter(out, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"staged {len(rows)} RGB frames, {sum(r['positive'] for r in rows)} positives")


if __name__ == "__main__":
    main()
