"""Apply decisions from inspected contact sheets; never modify regression_test."""
import csv
from collections import defaultdict
from pathlib import Path

import cv2

ROOT=Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work").resolve()

def read(path):
    return list(csv.DictReader(path.open(newline="",encoding="utf-8")))

def write(path,rows):
    with path.open("w",newline="",encoding="utf-8") as file:
        writer=csv.DictWriter(file,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)

manifest={r["image"]:r for r in read(ROOT/"dataset_v3_manifest.csv")}
candidates=read(ROOT/"expanded_near_duplicate_candidates.csv")
parent={}
def find(x):
    parent.setdefault(x,x)
    if parent[x]!=x:parent[x]=find(parent[x])
    return parent[x]
for row in candidates:
    if row["source_a"]==row["source_b"]=="DUT-Anti-UAV":parent[find(row["image_a"])]=find(row["image_b"])
components=defaultdict(list)
for image in parent:components[find(image)].append(image)
remove={}
for group in components.values():
    # Every candidate edge was visually inspected in dut_near_01..14.
    keeper=max(group,key=lambda x:(int(manifest[x]["width"])*int(manifest[x]["height"]),x))
    for image in group:
        if image!=keeper:remove[image]="visually confirmed near duplicate; keep "+keeper
protected=read(ROOT/"protected_source_scene_candidates.csv")
for row in protected:
    # 00195/00196 are different factory scenes; all other sheet entries are the
    # same protected video15 sequence or the old regression photograph.
    if Path(row["image"]).stem not in ("dut_train_00195","dut_train_00196"):
        remove[row["image"]]="visual source overlap with protected "+row["reference_group"]
changes=[]
for image,reason in sorted(remove.items()):
    row=manifest[image]
    if row["split"]=="regression_test":raise RuntimeError("Protected split")
    for key in ("image","label"):
        path=(ROOT/row[key]).resolve()
        if not path.is_relative_to(ROOT):raise RuntimeError(path)
        path.unlink()
    changes.append({"action":"removed","file":image,"old_split":row["split"],"new_split":"","reason":reason})
anti=read(ROOT/"anti300_pair_manifest.csv")
for row in anti:
    prefix=row["source_group"].split(":")[-1]
    target=row["split"]
    group=row["source_group"]
    if prefix.startswith("20190925_") and prefix.split("_")[1]>="190000":
        target="train";group="anti300:20190925_evening_capture"
    if prefix in ("20190926_183400","20190926_183941"):
        target="val";group="anti300:20190926_evening_sunset_capture"
    row["original_source_group"]=row["source_group"]
    if target!=row["split"]:
        old_split=row["split"]
        for key in ("rgb","thermal","rgb_label","thermal_label"):
            old=(ROOT/row[key]).resolve();new=ROOT/"rgbt"/target/old.name
            if not old.is_relative_to(ROOT) or not new.resolve().is_relative_to(ROOT):raise RuntimeError("outside root")
            old.rename(new);row[key]=new.relative_to(ROOT).as_posix()
        row["split"]=target
        changes.append({"action":"moved","file":row["pair_id"],"old_split":old_split,"new_split":target,
                        "reason":"visual similarity across neighboring capture sessions; keep complete capture union in one split"})
    row["source_group"]=group
write(ROOT/"anti300_pair_manifest.csv",anti)
write(ROOT/"expansion_visual_curation_changes.csv",changes)
print("DUT removed",len(remove),"near components",len(components),"paired moves",sum(r["action"]=="moved" for r in changes))
