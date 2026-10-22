"""Resume the official AOD-4 file download and record its checksum."""

from __future__ import annotations

import hashlib
import json
import time
import urllib.request
from pathlib import Path

URL = (
    "https://data.mendeley.com/public-files/datasets/cd5z895tr2/files/"
    "fb36418a-ca4d-4628-8e42-c6e5b2575ac2/file_downloaded"
)
ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2\raw_downloads\aod4_official")
FILE = ROOT / "AOD 4.zip"
EXPECTED = 1_921_835_727


def main() -> None:
    ROOT.mkdir(parents=True, exist_ok=True)
    offset = FILE.stat().st_size if FILE.exists() else 0
    if offset > EXPECTED:
        raise ValueError(f"oversized partial download: {offset}")
    if offset < EXPECTED:
        request = urllib.request.Request(
            URL,
            headers={"User-Agent": "Mozilla/5.0", "Range": f"bytes={offset}-"},
        )
        with urllib.request.urlopen(request, timeout=120) as response:
            if offset and response.status != 206:
                raise ValueError(f"server did not honor Range: {response.status}")
            with FILE.open("ab" if offset else "wb") as out:
                last = time.monotonic()
                while chunk := response.read(1024 * 1024):
                    out.write(chunk)
                    if time.monotonic() - last >= 10:
                        print(f"downloaded {out.tell():,}/{EXPECTED:,}", flush=True)
                        last = time.monotonic()
    if FILE.stat().st_size != EXPECTED:
        raise ValueError(f"incomplete download: {FILE.stat().st_size}")
    sha = hashlib.sha256()
    with FILE.open("rb") as source:
        for chunk in iter(lambda: source.read(4 * 1024 * 1024), b""):
            sha.update(chunk)
    record = {
        "source": URL,
        "doi": "10.17632/cd5z895tr2.1",
        "license": "CC BY 4.0",
        "bytes": FILE.stat().st_size,
        "sha256": sha.hexdigest(),
    }
    (ROOT / "download_record.json").write_text(
        json.dumps(record, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(record), flush=True)


if __name__ == "__main__":
    main()
