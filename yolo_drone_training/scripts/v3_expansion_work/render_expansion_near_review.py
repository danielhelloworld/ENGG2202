import csv
import math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT=Path(r"C:\Users\admin\Documents\drone_yolo_v2\dataset_v3_expansion_work")
OUT=ROOT/"curation_previews"
OUT.mkdir(exist_ok=True)
font=ImageFont.truetype(r"C:\Windows\Fonts\arial.ttf",13)
rows=list(csv.DictReader((ROOT/"expanded_near_duplicate_candidates.csv").open(newline="",encoding="utf-8")))
manifest={r["image"]:r for r in csv.DictReader((ROOT/"dataset_v3_manifest.csv").open(newline="",encoding="utf-8"))}

def panels(path):
    row=manifest[path]
    image=Image.open(ROOT/path).convert("RGB")
    width,height=image.size
    boxes=[]
    for line in (ROOT/row["label"]).read_text().splitlines():
        _,x,y,w,h=map(float,line.split())
        box=((x-w/2)*width,(y-h/2)*height,(x+w/2)*width,(y+h/2)*height)
        boxes.append(box)
        ImageDraw.Draw(image).rectangle(box,outline="red",width=2)
    if boxes:
        b=max(boxes,key=lambda b:(b[2]-b[0])*(b[3]-b[1]))
        cx,cy=(b[0]+b[2])/2,(b[1]+b[3])/2
        cw,ch=max(70,(b[2]-b[0])*3),max(70,(b[3]-b[1])*3)
        crop=image.crop((max(0,cx-cw/2),max(0,cy-ch/2),min(width,cx+cw/2),min(height,cy+ch/2)))
    else:
        crop=image
    return image,crop

def render(items,prefix):
    for start in range(0,len(items),6):
        batch=items[start:start+6]
        sheet=Image.new("RGB",(1200,len(batch)*230),"white")
        draw=ImageDraw.Draw(sheet)
        for i,row in enumerate(batch):
            a,ac=panels(row["image_a"]); b,bc=panels(row["image_b"])
            for j,im in enumerate((a,b,ac,bc)):
                sheet.paste(ImageOps.contain(im,(295,190)),(j*300,i*230))
            text=f"{start+i}: {Path(row['image_a']).stem} / {Path(row['image_b']).stem}"
            draw.text((4,i*230+193),text,fill="black",font=font)
            draw.text((4,i*230+210),f"{row['split_a']} / {row['split_b']}  pHash {row['phash_distance']} crop {row['crop_distance']} MAE {row['thumbnail_mae']}",fill="black",font=font)
        sheet.save(OUT/f"{prefix}_{start//6+1:02d}.jpg",quality=88)
    with (OUT/f"{prefix}_index.csv").open("w",newline="",encoding="utf-8") as file:
        writer=csv.DictWriter(file,fieldnames=["review_index"]+list(items[0]))
        writer.writeheader();writer.writerows({"review_index":i,**r} for i,r in enumerate(items))
    print(prefix,len(items),math.ceil(len(items)/6))

render([r for r in rows if r["priority"]!="within_train"],"cross_split")
render([r for r in rows if r["source_a"]==r["source_b"]=="DUT-Anti-UAV"],"dut_near")
paired={}
for row in rows:
    a,b=manifest[row["image_a"]],manifest[row["image_b"]]
    if a["pair_id"] and b["pair_id"]:
        paired.setdefault(tuple(sorted((a["pair_id"],b["pair_id"]))),[]).append(row)
mutual=[]
for value in paired.values():
    if len({manifest[r["image_a"]]["modality"] for r in value})==2:
        mutual.extend(value)
render(mutual,"paired_both_modalities")
