"""Sample synchronized RGB/IR frames from official Anti-UAV300 videos."""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
import tempfile
import zipfile
from collections import Counter
from pathlib import Path

import cv2

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2")
ARCHIVE = ROOT / "raw_downloads/official_github/Anti-UAV-RGBT.zip"
DATASET = ROOT / "dataset_v3_expansion_work"
BASE_MANIFEST = ROOT / "dataset_v3/dataset_v3_manifest.csv"
SECONDS_GAP = 4.0
MAX_PER_SEQUENCE = 15


def split_map() -> dict[str, str]:
    with BASE_MANIFEST.open(newline="", encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))
    result = {}
    for row in rows:
        if row["source_group"].startswith("aod4:timestamp:"):
            prefix = row["source_group"].split(":")[-1]
            previous = result.setdefault(prefix, row["split"])
            if previous != row["split"]:
                raise ValueError(f"AOD prefix leak: {prefix}")
    return result


def choose_split(prefix: str, old: dict[str, str]) -> str:
    if prefix in old:
        return old[prefix]
    bucket = int(hashlib.sha256(prefix.encode()).hexdigest()[:8], 16) % 10
    return "final_test" if bucket == 0 else "val" if bucket == 1 else "train"


def box_line(annotation: dict, index: int, width: int, height: int) -> tuple[str, bool]:
    visible = bool(annotation["exist"][index])
    if not visible:
        return "", False
    rect = annotation["gt_rect"][index]
    if not isinstance(rect, list) or len(rect) != 4:
        raise ValueError(f"bad GT rectangle at frame {index}: {rect}")
    x, y, bw, bh = map(float, rect)
    if not (bw > 0 and bh > 0 and 0 <= x < width and 0 <= y < height):
        raise ValueError(f"nonpositive/outside GT at frame {index}: {rect} vs {width,height}")
    # Original tracking annotations can extend a few pixels past the frame edge.
    x2, y2 = min(width, x + bw), min(height, y + bh)
    if x2 <= x or y2 <= y:
        raise ValueError(f"zero clipped GT at frame {index}: {rect}")
    line = f"0 {(x+x2)/(2*width):.8f} {(y+y2)/(2*height):.8f} {(x2-x)/width:.8f} {(y2-y)/height:.8f}\n"
    return line, True


def extract_member(archive: zipfile.ZipFile, name: str, target: Path) -> None:
    with archive.open(name) as src, target.open("wb") as dest:
        shutil.copyfileobj(src, dest, length=1024 * 1024)


def frame_at(cap: cv2.VideoCapture, index: int):
    cap.set(cv2.CAP_PROP_POS_FRAMES, index)
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError(f"cannot decode frame {index}")
    return frame


def main() -> None:
    old_splits = split_map()
    rows = []
    errors = []
    counts = Counter()
    with zipfile.ZipFile(ARCHIVE) as archive:
        directories = sorted(x.rsplit("/", 1)[0] for x in archive.namelist() if x.endswith("/visible.mp4"))
        if len(directories) != 318:
            raise RuntimeError(f"expected 318 sequences, got {len(directories)}")
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent, prefix="anti_video_") as temp_name:
            temp = Path(temp_name)
            for seq_no, directory in enumerate(directories, 1):
                name = directory.split("/")[-1]
                prefix = "_".join(name.split("_")[:2])
                split = choose_split(prefix, old_splits)
                visible_gt = json.loads(archive.read(directory + "/visible.json"))
                thermal_gt = json.loads(archive.read(directory + "/infrared.json"))
                rgb_video, th_video = temp / "visible.mp4", temp / "infrared.mp4"
                extract_member(archive, directory + "/visible.mp4", rgb_video)
                extract_member(archive, directory + "/infrared.mp4", th_video)
                rgb_cap, th_cap = cv2.VideoCapture(str(rgb_video)), cv2.VideoCapture(str(th_video))
                try:
                    rfps, tfps = rgb_cap.get(cv2.CAP_PROP_FPS), th_cap.get(cv2.CAP_PROP_FPS)
                    rn, tn = int(rgb_cap.get(cv2.CAP_PROP_FRAME_COUNT)), int(th_cap.get(cv2.CAP_PROP_FRAME_COUNT))
                    rw, rh = int(rgb_cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(rgb_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    tw, th = int(th_cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(th_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    if min(rfps, tfps, rn, tn, rw, rh, tw, th) <= 0:
                        raise ValueError("bad video metadata")
                    duration = min((rn - 1) / rfps, (tn - 1) / tfps,
                                   (len(visible_gt["exist"]) - 1) / rfps,
                                   (len(thermal_gt["exist"]) - 1) / tfps)
                    times = [i * SECONDS_GAP for i in range(int(duration / SECONDS_GAP) + 1)]
                    if len(times) > MAX_PER_SEQUENCE:
                        times = [times[round(i * (len(times) - 1) / (MAX_PER_SEQUENCE - 1))] for i in range(MAX_PER_SEQUENCE)]
                    for sample_no, second in enumerate(times):
                        ri, ti = min(rn - 1, round(second * rfps)), min(tn - 1, round(second * tfps))
                        try:
                            rgb_line, rgb_positive = box_line(visible_gt, ri, rw, rh)
                            th_line, th_positive = box_line(thermal_gt, ti, tw, th)
                        except ValueError as error:
                            errors.append((directory + f"#t={second}", str(error)))
                            continue
                        rgb = frame_at(rgb_cap, ri)
                        thermal = frame_at(th_cap, ti)
                        if rgb.shape[:2] != (rh, rw) or thermal.shape[:2] != (th, tw):
                            raise ValueError("decoded image size mismatch")
                        pair_id = f"anti300_{name}_t{round(second*1000):06d}"
                        folder = DATASET / "rgbt" / split
                        folder.mkdir(parents=True, exist_ok=True)
                        rgb_path, th_path = folder / f"{pair_id}_rgb.jpg", folder / f"{pair_id}_thermal.jpg"
                        rl_path, tl_path = folder / f"{pair_id}_rgb.txt", folder / f"{pair_id}_thermal.txt"
                        if not cv2.imwrite(str(rgb_path), rgb, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                            raise IOError(rgb_path)
                        if not cv2.imwrite(str(th_path), thermal, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                            raise IOError(th_path)
                        rl_path.write_text(rgb_line, encoding="utf-8")
                        tl_path.write_text(th_line, encoding="utf-8")
                        rows.append({
                            "pair_id": pair_id, "split": split, "source": "Anti-UAV300", "source_group": "anti300:" + prefix,
                            "sequence": name, "official_split": directory.split("/")[0], "time_s": second,
                            "rgb_frame": ri, "thermal_frame": ti,
                            "rgb": rgb_path.relative_to(DATASET).as_posix(), "thermal": th_path.relative_to(DATASET).as_posix(),
                            "rgb_label": rl_path.relative_to(DATASET).as_posix(), "thermal_label": tl_path.relative_to(DATASET).as_posix(),
                            "rgb_width": rw, "rgb_height": rh, "thermal_width": tw, "thermal_height": th,
                            "rgb_positive": int(rgb_positive), "thermal_positive": int(th_positive),
                        })
                        counts[split] += 1
                except Exception as error:
                    errors.append((directory, str(error)))
                finally:
                    rgb_cap.release()
                    th_cap.release()
                    rgb_video.unlink(missing_ok=True)
                    th_video.unlink(missing_ok=True)
                if seq_no % 25 == 0:
                    print("Anti-UAV300 sequences", seq_no, "pairs", len(rows), "errors", len(errors), flush=True)
    if not rows:
        raise RuntimeError("no Anti-UAV frames staged")
    with (DATASET / "anti300_pair_manifest.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (DATASET / "anti300_staging_errors.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("sequence", "error"))
        writer.writerows(errors)
    print("Anti-UAV300 complete", len(rows), dict(counts), "errors", len(errors))
    if errors:
        print("first errors", errors[:5])


if __name__ == "__main__":
    main()
