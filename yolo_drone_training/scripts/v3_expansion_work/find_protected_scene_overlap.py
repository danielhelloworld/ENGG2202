import csv
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT=Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work")
WORK=Path(__file__).parent
rows=list(csv.DictReader((ROOT/"dataset_v3_manifest.csv").open(newline="",encoding="utf-8")))
feat=np.load(WORK/"expanded_perceptual_features.npz")
lookup={p:i for i,p in enumerate(feat["paths"])}
hashes,thumbs=feat["hashes"],feat["thumbs"]
refs=[r for r in rows if r["source_group"] in ("mendeley:video15","mendeley:280.jpeg")]
found=[]
for row in rows:
    if row["source"]!="DUT-Anti-UAV":continue
    i=lookup[row["image"]]
    matches=[]
    for ref in refs:
        j=lookup[ref["image"]]
        d=(int(hashes[i])^int(hashes[j])).bit_count()
        if d>12:continue
        mae=float(np.abs(thumbs[i].astype(np.int16)-thumbs[j].astype(np.int16)).mean())
        if mae<20:matches.append((d,mae,ref))
    if matches:
        d,mae,ref=min(matches,key=lambda x:(x[0],x[1]))
        found.append({"image":row["image"],"reference":ref["image"],"reference_group":ref["source_group"],"phash_distance":d,"mae":round(mae,3)})
with (ROOT/"protected_source_scene_candidates.csv").open("w",newline="",encoding="utf-8") as file:
    writer=csv.DictWriter(file,fieldnames=list(found[0]));writer.writeheader();writer.writerows(found)
font=ImageFont.truetype(r"C:\Windows\Fonts\arial.ttf",14)
for start in range(0,len(found),20):
    batch=found[start:start+20]
    sheet=Image.new("RGB",(1200,((len(batch)+3)//4)*200),"white")
    draw=ImageDraw.Draw(sheet)
    for k,row in enumerate(batch):
        im=Image.open(ROOT/row["image"]).convert("RGB")
        x,y=(k%4)*300,(k//4)*200
        sheet.paste(ImageOps.contain(im,(295,165)),(x,y))
        draw.text((x,y+166),f"{start+k}: {Path(row['image']).stem}",fill="black",font=font)
        draw.text((x,y+183),f"{row['reference_group']} d={row['phash_distance']}",fill="black",font=font)
    sheet.save(ROOT/"curation_previews"/f"protected_scene_{start//20+1:02d}.jpg",quality=90)
print("protected source similar-scene candidates",len(found),"references",len(refs))
