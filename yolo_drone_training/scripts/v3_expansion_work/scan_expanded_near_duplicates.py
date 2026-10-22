"""Perceptual screening with target crops, to reduce sky-background false positives."""
from __future__ import annotations

import csv
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work")
WORK = Path(__file__).parent


def phash(gray):
    small = cv2.resize(gray, (32,32), interpolation=cv2.INTER_AREA)
    dct = cv2.dct(np.float32(small))[:8,:8]
    bits = (dct > np.median(dct.ravel()[1:])).ravel()
    return sum(int(bit) << i for i,bit in enumerate(bits))


def feature(row):
    image = cv2.imread(str(ROOT / row["image"]), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(row["image"])
    height,width = image.shape
    lines = (ROOT / row["label"]).read_text().splitlines()
    box = None
    if lines:
        values = [list(map(float, line.split()[1:])) for line in lines]
        box = max(values, key=lambda x:x[2]*x[3])
        x,y,w,h = box
        cw,ch = max(64,w*width*4),max(64,h*height*4)
        crop = image[max(0,int(y*height-ch/2)):min(height,int(y*height+ch/2)+1),
                     max(0,int(x*width-cw/2)):min(width,int(x*width+cw/2)+1)]
    else:
        crop = image
    return phash(image),phash(crop),cv2.resize(image,(32,32),interpolation=cv2.INTER_AREA),np.array(box or [-1,-1,-1,-1])


def main():
    rows = list(csv.DictReader((ROOT / "dataset_v3_manifest.csv").open(newline="",encoding="utf-8")))
    cache = WORK / "expanded_perceptual_features.npz"
    if cache.exists():
        features = np.load(cache)
        hashes,crop_hashes,thumbs,boxes = features["hashes"],features["crop_hashes"],features["thumbs"],features["boxes"]
        if list(features["paths"]) != [r["image"] for r in rows]:
            names={Path(p).name:i for i,p in enumerate(features["paths"])}
            indices=[names[Path(r["image"]).name] for r in rows]
            hashes,crop_hashes,thumbs,boxes=hashes[indices],crop_hashes[indices],thumbs[indices],boxes[indices]
    else:
        with ThreadPoolExecutor(max_workers=8) as pool:
            features = []
            for i,item in enumerate(pool.map(feature,rows),1):
                features.append(item)
                if i%2000==0:
                    print("perceptual features",i,flush=True)
        hashes = np.array([x[0] for x in features],dtype=np.uint64)
        crop_hashes = np.array([x[1] for x in features],dtype=np.uint64)
        thumbs = np.stack([x[2] for x in features])
        boxes = np.stack([x[3] for x in features])
        np.savez_compressed(cache,paths=np.array([r["image"] for r in rows]),hashes=hashes,crop_hashes=crop_hashes,thumbs=thumbs,boxes=boxes)
    buckets = defaultdict(list)
    candidates = []
    screened = 0
    for i,row in enumerate(rows):
        value = int(hashes[i])
        previous = set()
        for part in range(4):
            key = (row["modality"],part,(value>>(16*part))&65535)
            previous.update(buckets[key])
            buckets[key].append(i)
        for j in previous:
            other = rows[j]
            same_group = row["source_group"] == other["source_group"]
            if same_group and row["source"] != "DUT-Anti-UAV":
                continue
            if row["source"] not in ("DUT-Anti-UAV","Anti-UAV300","UAV-CB") and other["source"] not in ("DUT-Anti-UAV","Anti-UAV300","UAV-CB"):
                continue
            distance = (value ^ int(hashes[j])).bit_count()
            if distance>2:
                continue
            screened += 1
            mae = float(np.abs(thumbs[i].astype(np.int16)-thumbs[j].astype(np.int16)).mean())
            crop_distance = (int(crop_hashes[i]) ^ int(crop_hashes[j])).bit_count()
            if (boxes[i,0]<0) != (boxes[j,0]<0):
                continue
            if boxes[i,0]>=0:
                center_distance = float(np.linalg.norm(boxes[i,:2]-boxes[j,:2]))
                size_ratio = float(np.sqrt(np.prod(boxes[i,2:])/np.prod(boxes[j,2:])))
                if center_distance>0.04 or not 0.65<size_ratio<1.55 or crop_distance>10:
                    continue
            if mae>8 or (same_group and (distance>0 or crop_distance>4 or mae>2)):
                continue
            candidates.append({"priority":"regression" if "regression_test" in (row["split"],other["split"]) else "cross_split" if row["split"]!=other["split"] else "within_train",
                "phash_distance":distance,"crop_distance":crop_distance,"thumbnail_mae":round(mae,3),
                "image_a":other["image"],"image_b":row["image"],"source_a":other["source"],"source_b":row["source"],
                "split_a":other["split"],"split_b":row["split"],"group_a":other["source_group"],"group_b":row["source_group"]})
    target=ROOT/"expanded_near_duplicate_candidates.csv"
    with target.open("w",newline="",encoding="utf-8") as file:
        writer=csv.DictWriter(file,fieldnames=list(candidates[0]) if candidates else ["image_a","image_b"])
        writer.writeheader();writer.writerows(candidates)
    print("global pHash<=2 screened",screened,"high-priority candidates",len(candidates))
    print("regression",sum(x["priority"]=="regression" for x in candidates),"cross_split",sum(x["priority"]=="cross_split" for x in candidates))


if __name__=="__main__":
    main()
