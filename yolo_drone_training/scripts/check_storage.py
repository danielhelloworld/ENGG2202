#!/usr/bin/env python3
"""Report project disk use without enforcing a fixed storage limit."""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def directory_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(
        item.stat().st_size
        for item in path.rglob("*")
        if item.is_file() and not item.is_symlink()
    )


def gib(value: int) -> float:
    return value / 1024**3


def main() -> int:
    categories = [".venv", "raw", "dataset", "runs", "artifacts"]
    accounted = 0
    for name in categories:
        size = directory_bytes(PROJECT_ROOT / name)
        accounted += size
        print(f"{name:10s} {gib(size):6.2f} GiB")

    other = max(0, directory_bytes(PROJECT_ROOT) - accounted)
    total = accounted + other
    print(f"{'other':10s} {gib(other):6.2f} GiB")
    print(f"{'TOTAL':10s} {gib(total):6.2f} GiB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
