"""One GPU epoch on a 64/32 image sample, separate from formal training."""
import csv, json, os, random, shutil, sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
os.environ.setdefault('YOLO_CONFIG_DIR',str(HERE/'.ultralytics'))
os.environ.setdefault('MPLCONFIGDIR',str(HERE/'.mplconfig'))
DATA=Path(os.environ.get('DRONE_DATASET_ROOT',r'C:\Users\admin\Documents\drone_yolo_v2\dataset_v3'))
SUBSET=HERE/'smoke_subset_v3new'
RUN=HERE/'runs'/'v2_smoke'/'gpu_1epoch_v3new'

def pick(rows,split,npositive,nnegative):
    positive=[];negative=[]
    for row in rows:
        if row['split']!=split:continue
        (positive if (DATA/row['label']).read_text().strip() else negative).append(row)
    random.Random(2202 if split=='train' else 2203).shuffle(positive)
    random.Random(2204 if split=='train' else 2205).shuffle(negative)
    # Include the newly acquired AOD-4 source in both positive and negative paths.
    new_positive=[r for r in positive if r['source']=='AOD-4']
    new_negative=[r for r in negative if r['source']=='AOD-4']
    quota=8 if split=='train' else 4
    selected=(new_positive[:quota]+[r for r in positive if r not in new_positive][:npositive-quota]
              +new_negative[:quota]+[r for r in negative if r not in new_negative][:nnegative-quota])
    if len(selected)!=npositive+nnegative:raise RuntimeError(f'Insufficient {split} sample')
    return selected

def main():
    import torch
    from ultralytics import YOLO
    if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable')
    if RUN.exists():raise FileExistsError(f'Smoke run already exists: {RUN}')
    rows=list(csv.DictReader((DATA/'dataset_v3_manifest.csv').open(encoding='utf-8-sig',newline='')))
    counts={}
    for split,np,nn in [('train',32,32),('val',16,16)]:
        selected=pick(rows,split,np,nn)
        (SUBSET/'images'/split).mkdir(parents=True,exist_ok=True)
        (SUBSET/'labels'/split).mkdir(parents=True,exist_ok=True)
        for row in selected:
            shutil.copy2(DATA/row['image'],SUBSET/'images'/split/Path(row['image']).name)
            shutil.copy2(DATA/row['label'],SUBSET/'labels'/split/Path(row['label']).name)
        counts[split]={'images':len(selected),'positive':np,'negative':nn}
    (SUBSET/'data.yaml').write_text(f'path: {SUBSET.as_posix()}\ntrain: images/train\nval: images/val\nnames:\n  0: drone\n',encoding='utf-8')
    weights=Path(r'C:\Users\admin\Documents\drone_yolo_v2\v1\training\best.pt')
    YOLO(str(weights)).train(data=str(SUBSET/'data.yaml'),epochs=1,patience=1,
        batch=8,imgsz=640,device=0,workers=0,optimizer='AdamW',lr0=0.0005,
        amp=True,cache=False,plots=False,seed=2202,deterministic=True,
        project=str(RUN.parent),name=RUN.name,exist_ok=False,resume=False)
    result={'status':'PASS' if (RUN/'results.csv').is_file() and (RUN/'weights/last.pt').is_file() else 'FAIL',
            'subset':counts,'python':sys.version.split()[0],'torch':torch.__version__,
            'cuda_runtime':torch.version.cuda,'gpu':torch.cuda.get_device_name(0),'run_dir':str(RUN)}
    (RUN.parent/'smoke_test_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('SMOKE_RESULT',json.dumps(result),flush=True)
    if result['status']!='PASS':raise RuntimeError('Smoke outputs missing')

if __name__=='__main__':main()
