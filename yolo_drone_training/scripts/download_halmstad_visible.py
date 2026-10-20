#!/usr/bin/env python3
"""Download only the CC0 Halmstad visible-light videos and annotations."""

from __future__ import annotations

import argparse
import json
import os
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from check_storage import directory_bytes


PROJECT_ROOT = Path(__file__).resolve().parents[1]
API_URL = (
    "https://api.github.com/repos/DroneDetectionThesis/"
    "Drone-detection-dataset/contents/Data/Video_V?per_page=1000"
)


def read_json(url: str):
    request = urllib.request.Request(url, headers={"User-Agent": "ENGG2202-dataset"})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def download_one(item: dict, output: Path) -> tuple[str, str]:
    target = output / item["name"]
    expected = int(item["size"])
    if target.is_file() and target.stat().st_size == expected:
        return item["name"], "existing"
    temporary = target.with_suffix(target.suffix + ".part")
    request = urllib.request.Request(
        item["download_url"], headers={"User-Agent": "ENGG2202-dataset"}
    )
    with urllib.request.urlopen(request, timeout=60) as response:
        with temporary.open("wb") as handle:
            while chunk := response.read(1024 * 1024):
                handle.write(chunk)
    actual = temporary.stat().st_size
    if actual != expected:
        temporary.unlink(missing_ok=True)
        raise OSError(
            f"{item['name']}: expected {expected} bytes, got {actual}"
        )
    os.replace(temporary, target)
    return item["name"], "downloaded"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=Path("raw/sources/halmstad_visible")
    )
    parser.add_argument("--max-download-mib", type=int, default=300)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    output = (PROJECT_ROOT / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)

    listing = read_json(API_URL)
    selected = sorted(
        (
            item
            for item in listing
            if item.get("type") == "file"
            and item["name"].startswith("V_")
            and (item["name"].endswith(".mp4") or item["name"].endswith("_LABELS.mat"))
        ),
        key=lambda item: item["name"],
    )
    videos = {item["name"][:-4] for item in selected if item["name"].endswith(".mp4")}
    labels = {
        item["name"][:-11]
        for item in selected
        if item["name"].endswith("_LABELS.mat")
    }
    if videos != labels:
        raise SystemExit(
            f"video/annotation mismatch: missing labels={sorted(videos-labels)}, "
            f"missing videos={sorted(labels-videos)}"
        )

    total_bytes = sum(int(item["size"]) for item in selected)
    max_bytes = args.max_download_mib * 1024**2
    if total_bytes > max_bytes:
        raise SystemExit(
            f"refusing {total_bytes / 1024**2:.1f} MiB download; "
            f"limit is {args.max_download_mib} MiB"
        )
    current_project_bytes = directory_bytes(PROJECT_ROOT)
    missing_bytes = sum(
        int(item["size"])
        for item in selected
        if not (output / item["name"]).is_file()
        or (output / item["name"]).stat().st_size != int(item["size"])
    )
    projected = current_project_bytes + missing_bytes
    print(
        f"approved: {len(videos)} videos + {len(labels)} annotations, "
        f"{total_bytes / 1024**2:.1f} MiB total, "
        f"projected project size {projected / 1024**3:.2f} GiB"
    )
    counts = {"downloaded": 0, "existing": 0}
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as executor:
        futures = [executor.submit(download_one, item, output) for item in selected]
        for number, future in enumerate(as_completed(futures), start=1):
            name, state = future.result()
            counts[state] += 1
            if number % 50 == 0 or number == len(futures):
                print(f"completed {number}/{len(futures)} files")

    manifest = {
        "source": API_URL,
        "license": "CC0-1.0",
        "videos": len(videos),
        "annotations": len(labels),
        "bytes": total_bytes,
        "files": [{"name": item["name"], "size": item["size"]} for item in selected],
    }
    (output / "download_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(counts, sort_keys=True))


if __name__ == "__main__":
    main()
