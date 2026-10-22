"""Contact sheets for all validation/final-test images and 20% of train."""

from __future__ import annotations

import csv
import math
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_new")
DEST = ROOT / "annotation_previews"


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as file:
        return list(csv.DictReader(file))


def draw_sheet(items: list[dict[str, str]], stem: str) -> None:
    for page in range(math.ceil(len(items) / 100)):
        subset = items[page * 100 : (page + 1) * 100]
        canvas = Image.new("RGB", (1400, 1250), "white")
        draw = ImageDraw.Draw(canvas)
        for local_index, row in enumerate(subset):
            index = page * 100 + local_index
            image_path = ROOT / row["image"]
            label_path = ROOT / row["label"]
            image = Image.open(image_path).convert("RGB")
            original_w, original_h = image.size
            image.thumbnail((135, 98), Image.Resampling.LANCZOS)
            x = (local_index % 10) * 140
            y = (local_index // 10) * 125
            canvas.paste(image, (x, y))
            for line in label_path.read_text(encoding="utf-8-sig").splitlines():
                _, cx, cy, bw, bh = map(float, line.split())
                x1 = x + (cx - bw / 2) * image.width
                y1 = y + (cy - bh / 2) * image.height
                x2 = x + (cx + bw / 2) * image.width
                y2 = y + (cy + bh / 2) * image.height
                draw.rectangle((x1, y1, x2, y2), outline="red", width=2)
            color = "darkgreen" if int(row["positive"]) == 0 else "black"
            draw.text((x, y + 100), f"{index}: {row['source'][:10]}", fill=color)
        canvas.save(DEST / f"{stem}_{page + 1:02d}.jpg", quality=88)


def main() -> None:
    DEST.mkdir(exist_ok=True)
    manifest = rows(ROOT / "dataset_v3_manifest.csv")
    val = [r for r in manifest if r["split"] == "val"]
    final = [r for r in manifest if r["split"] == "final_test"]
    train = [r for r in manifest if r["split"] == "train"]
    sample = sorted(random.Random(2202).sample(train, math.ceil(len(train) * 0.20)), key=lambda r: r["image"])
    review_images = {r["image"] for r in rows(ROOT / "review_required.csv")}
    review = [r for r in manifest if r["image"] in review_images]
    for name, items in (("val_all", val), ("final_test_all", final), ("train_random_20pct", sample), ("tiny_multi_all", review)):
        draw_sheet(items, name)
        with (DEST / f"{name}_index.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["index", "image", "label", "split", "source"])
            for index, row in enumerate(items):
                writer.writerow([index, row["image"], row["label"], row["split"], row["source"]])
        print(name, len(items), "sheets", math.ceil(len(items) / 100))


if __name__ == "__main__":
    main()
