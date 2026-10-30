"""Portable RGB drone dataset and YOLO11 training/evaluation delivery tools."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import os
import random
import shutil
import sys
import tempfile
import time
import zipfile
from collections import Counter,defaultdict
from pathlib import Path

ROOT=Path(__file__).resolve().parent
os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'outputs/runtime_cache/ultralytics'))
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'outputs/runtime_cache/matplotlib'))

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(8*1024**2),b''):h.update(chunk)
    return h.hexdigest()

def read_csv(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

def dump(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2,default=str)+'\n',encoding='utf-8')

def dataset(path):
    path=Path(path).resolve()
    if not (path/'manifest.csv').is_file():raise ValueError(f'Dataset manifest missing: {path}')
    return path

def restore(args):
    package=Path(args.package).resolve();dest=Path(args.dest).resolve()
    spec=json.loads((package/'PACKAGE_MANIFEST.json').read_text(encoding='utf-8'))
    dest.mkdir(parents=True,exist_ok=True)
    for item in spec['dataset_archives']:
        archive=package/item['file']
        if sha(archive)!=item['sha256']:raise ValueError(f'Archive checksum mismatch: {archive}')
        with zipfile.ZipFile(archive) as z:
            for member in z.infolist():
                target=(dest/member.filename).resolve()
                if not target.is_relative_to(dest):raise ValueError('Unsafe archive path')
                if member.is_dir():target.mkdir(parents=True,exist_ok=True);continue
                target.parent.mkdir(parents=True,exist_ok=True)
                temporary=target.with_name(target.name+'.restoring')
                with z.open(member) as source,temporary.open('wb') as out:shutil.copyfileobj(source,out,1024**2)
                temporary.replace(target)
        print('Restored',archive.name,flush=True)
    print('Dataset restored:',dest)

def check(args):
    from PIL import Image
    data=dataset(args.data);rows=read_csv(data/'manifest.csv');groups=defaultdict(set);counts=Counter();issues=[]
    images=set();hashes={};known_splits={'train','val','final_test','regression_test'}
    for i,r in enumerate(rows,1):
        p=data/r['image'];label=data/r['label'];split=r['split']
        try:
            if split not in known_splits or not p.resolve().is_relative_to(data) or not label.resolve().is_relative_to(data):raise ValueError('path/split')
            if r['image'] in images:raise ValueError('duplicate path')
            images.add(r['image'])
            if r['modality']!='RGB' or '_thermal.' in r['image']:raise ValueError('non-RGB entry')
            if split!='regression_test':groups[r['evaluation_group']].add(split)
            if args.full:
                if sha(p)!=r['image_sha256'] or sha(label)!=r['label_sha256']:raise ValueError('SHA-256 mismatch')
                with Image.open(p) as im:
                    im.load()
                    if im.size!=(int(r['width']),int(r['height'])):raise ValueError('dimensions')
                    im.convert('RGB')
            bs=[]
            for line in label.read_text().splitlines():
                c,x,y,w,h=map(float,line.split())
                if c!=0 or not all(math.isfinite(v) for v in [x,y,w,h]) or min(w,h)<=0 or min(x-w/2,y-h/2)<-1e-6 or max(x+w/2,y+h/2)>1+1e-6:raise ValueError('invalid annotation')
                bs.append((x,y,w,h))
            if len(bs)!=int(r['objects']) or bool(bs)!=bool(int(r['positive'])):raise ValueError('annotation count')
            if split!='regression_test':
                old=hashes.setdefault(r['image_sha256'],r['image'])
                if old!=r['image']:raise ValueError('duplicate file hash: '+old)
            counts[split]+=1
        except Exception as exc:issues.append(dict(image=r['image'],error=str(exc)))
        if i%3000==0:print('Checked',i,'/',len(rows),flush=True)
    for split in known_splits:
        expected={r['image'] for r in rows if r['split']==split}
        listed={line.removeprefix('./') for line in (data/f'{split}.txt').read_text().splitlines()}
        if expected!=listed:issues.append(dict(split=split,error='split list differs from manifest'))
    conflicts={g:sorted(s) for g,s in groups.items() if len(s)>1}
    report=dict(status='PASS' if not issues and not conflicts else 'FAIL',images=len(rows),counts=dict(counts),issues=issues,group_conflicts=conflicts,full_hash_and_decode=args.full)
    dump(data/'LAST_CHECK.json',report);print(json.dumps(report,ensure_ascii=False,indent=2))
    if report['status']!='PASS':raise SystemExit(1)

def doctor(args):
    import torch,ultralytics
    info=dict(python=sys.version,torch=torch.__version__,ultralytics=ultralytics.__version__,cuda=torch.cuda.is_available(),cuda_runtime=torch.version.cuda)
    if info['cuda']:
        info['gpu']=torch.cuda.get_device_name(0)
        x=torch.ones(3,device='cuda');info['cuda_tensor_test']=float((x+x).sum().cpu())
    print(json.dumps(info,indent=2))
    if args.require_cuda and not info['cuda']:raise SystemExit('CUDA unavailable: check NVIDIA driver and CUDA PyTorch installation before training.')

def train(args):
    from ultralytics import YOLO
    data=dataset(args.data)
    if not Path(args.model).is_file():raise FileNotFoundError(args.model)
    if (ROOT/'runs'/args.name).exists():raise FileExistsError('Run exists; choose a new --name to preserve results')
    if os.name=='nt':
        import ctypes
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)  # Keep system awake; display may turn off.
    YOLO(str(Path(args.model).resolve())).train(data=str(data/('data_balanced75.yaml' if args.balanced else 'data.yaml')),
      epochs=args.epochs,patience=args.patience,imgsz=640,batch=args.batch,device=args.device,
      workers=args.workers,optimizer='AdamW',lr0=args.lr,lrf=.01,weight_decay=.0005,warmup_epochs=3,
      close_mosaic=10,seed=2202,deterministic=True,cache=False,amp=True,resume=False,
      save=True,save_period=5,plots=True,project=str(ROOT/'runs'),name=args.name,exist_ok=False)

def smoke(args):
    from ultralytics import YOLO
    import yaml
    data=dataset(args.data);rng=random.Random(2202)
    # Exercise positive and negative examples from every source, using a small
    # temporary list. Does not alter formal training weights or test data.
    rows=read_csv(data/'manifest.csv')
    with tempfile.TemporaryDirectory(prefix='drone_smoke_') as td:
        work=Path(td)
        for split,budget in [('train',64),('val',32)]:
            buckets=defaultdict(list)
            for r in rows:
                if r['split']==split:buckets[(r['source'],r['positive'])].append(r)
            selected=[]
            for bucket in buckets.values():rng.shuffle(bucket)
            while len(selected)<budget:
                progress=False
                for key in sorted(buckets):
                    if buckets[key] and len(selected)<budget:selected.append(buckets[key].pop());progress=True
                if not progress:break
            (work/f'{split}.txt').write_text(''.join((data/r['image']).as_posix()+'\n' for r in selected))
        config=dict(path=str(work),train='train.txt',val='val.txt',names={0:'drone'},channels=3)
        (work/'data.yaml').write_text(yaml.safe_dump(config))
        YOLO(str(Path(args.model).resolve())).train(data=str(work/'data.yaml'),epochs=1,imgsz=320,batch=4,device=args.device,
          workers=0,cache=False,amp=False,plots=False,save=False,project=str(ROOT/'outputs'),name='smoke_'+time.strftime('%Y%m%d_%H%M%S'),exist_ok=False)

def evaluate(args):
    from ultralytics import YOLO
    data=dataset(args.data);out=Path(args.out).resolve()
    if out.exists():raise FileExistsError('Evaluation output exists: '+str(out))
    result=YOLO(str(Path(args.model).resolve())).val(data=str(data/('data_regression.yaml' if args.split=='regression_test' else 'data.yaml')),
      split='test' if args.split in ['final_test','regression_test'] else 'val',imgsz=640,batch=args.batch,device=args.device,
      workers=0,conf=.001,iou=.7,plots=True,save_json=True,project=str(out.parent),name=out.name,exist_ok=False)
    metrics=dict(result.results_dict)
    metrics.update(split=args.split,weights_sha256=sha(args.model),data_version=json.loads((data/'DATASET_CARD.json').read_text())['version'],
      confidence_for_AP=.001,nms_iou=.7,operating_threshold_note='AP uses a confidence sweep; deployment confidence is selected on validation.')
    dump(out/'metrics.json',metrics)
    print(json.dumps(metrics,indent=2))

def errors(args):
    from ultralytics import YOLO
    from PIL import Image,ImageDraw
    data=dataset(args.data);rows=[r for r in read_csv(data/'manifest.csv') if r['split']==args.split];by={str((data/r['image']).resolve()):r for r in rows}
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False);records=[]
    def box_iou(a,b):
        x1,y1,x2,y2=a;u1,v1,u2,v2=b;inter=max(0,min(x2,u2)-max(x1,u1))*max(0,min(y2,v2)-max(y1,v1))
        return inter/((x2-x1)*(y2-y1)+(u2-u1)*(v2-v1)-inter) if inter else 0
    source_list=out/'source_images.txt';source_list.write_text('\n'.join(by)+'\n')
    predictions=YOLO(str(Path(args.model).resolve())).predict(source=str(source_list),stream=True,imgsz=640,batch=1,device=args.device,conf=args.conf,iou=args.iou,verbose=False)
    for pred in predictions:
        r=by[str(Path(pred.path).resolve())];w,h=int(r['width']),int(r['height']);gt=[]
        for line in (data/r['label']).read_text().splitlines():
            _,x,y,bw,bh=map(float,line.split());gt.append(((x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h))
        boxes=pred.boxes.xyxy.cpu().tolist();scores=pred.boxes.conf.cpu().tolist();used=set();tp=set()
        for i in sorted(range(len(boxes)),key=lambda k:-scores[k]):
            choices=[(box_iou(boxes[i],b),j) for j,b in enumerate(gt) if j not in used]
            overlap,j=max(choices,default=(0,-1))
            if overlap>=.5:tp.add(i);used.add(j)
        fp=len(boxes)-len(tp);fn=len(gt)-len(used)
        records.append(dict(image=r['image'],source=r['source'],gt=len(gt),predictions=len(boxes),tp=len(tp),fp=fp,fn=fn))
        if fp or fn:
            with Image.open(data/r['image']) as im:im=im.convert('RGB')
            draw=ImageDraw.Draw(im)
            for j,b in enumerate(gt):draw.rectangle(b,outline='lime' if j in used else 'orange',width=2)
            for i,b in enumerate(boxes):
                draw.rectangle(b,outline='cyan' if i in tp else 'red',width=2);draw.text((b[0],max(0,b[1]-12)),f'{"TP" if i in tp else "FP"} {scores[i]:.3f}',fill='cyan' if i in tp else 'red')
            for folder,condition in [('false_positives',fp),('false_negatives',fn)]:
                if condition:(out/folder).mkdir(exist_ok=True);im.save(out/folder/Path(r['image']).name,quality=93)
    with (out/'all_images.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    dump(out/'summary.json',dict(images=len(records),confidence=args.conf,nms_iou=args.iou,match_iou=.5,
      tp=sum(r['tp'] for r in records),fp=sum(r['fp'] for r in records),fn=sum(r['fn'] for r in records)))

def calibration(args):
    data=dataset(args.data);rows=[r for r in read_csv(data/'manifest.csv') if r['split']=='train'];rng=random.Random(2202)
    buckets=defaultdict(list)
    for r in rows:buckets[(r['source'],r['positive'])].append(r)
    for b in buckets.values():rng.shuffle(b)
    selected=[]
    while len(selected)<100:
        progress=False
        for key in sorted(buckets):
            if buckets[key] and len(selected)<100:selected.append(buckets[key].pop());progress=True
        if not progress:break
    out=Path(args.out).resolve();out.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
        for i,r in enumerate(selected):z.write(data/r['image'],f'{i:03d}.jpg')
    if out.stat().st_size>100_000_000:raise ValueError('Calibration ZIP exceeds converter limit')
    dump(out.with_suffix('.json'),dict(images=len(selected),source_split='train',files=[r['image'] for r in selected],note='Replace with representative MaxiCAM2 captures when available; do not use final-test images.'))

def export(args):
    from ultralytics import YOLO
    import onnx
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False)
    shutil.copy2(args.model,out/'best.pt')
    model=YOLO(str(out/'best.pt'))
    if list(model.names.values())!=['drone']:raise ValueError('Expected single-class drone model')
    file=model.export(format='onnx',imgsz=(480,640),opset=17,simplify=True,dynamic=False,nms=False,batch=1,device='cpu')
    graph=onnx.load(file);onnx.checker.check_model(graph)
    shape=[d.dim_value for d in graph.graph.input[0].type.tensor_type.shape.dim]
    if shape!=[1,3,480,640]:raise ValueError(shape)
    dump(out/'conversion_parameters.json',dict(target='MaixCAM2',model='YOLO11n Detect',classes=['drone'],input_shape=shape,
      opset=17,dynamic=False,nms=False,onnx_sha256=sha(file),converter='https://maixhub.com/toolbox',
      expected_download=['.mud','one or two .axmodel files'],status='ONNX_READY_FOR_CONVERSION; board compilation/testing follows training'))
    print('Fixed 640x480 ONNX ready:',file)

def save(args):
    run=Path(args.run).resolve();package=Path(args.package).resolve();name=run.name
    if not (run/'weights/best.pt').is_file():raise FileNotFoundError('best.pt missing from run')
    dest=package/'training_results'/f'{name}_{time.strftime("%Y%m%d_%H%M%S")}';dest.mkdir(parents=True,exist_ok=False)
    def copy_verified(source,target):
        target.parent.mkdir(parents=True,exist_ok=True)
        for attempt in range(3):
            before=sha(source);shutil.copy2(source,target)
            if before==sha(source)==sha(target):return
        raise ValueError('Source still changing or copy verification failed: '+str(source))
    for p in run.rglob('*'):
        if p.is_file() and p.name not in ['.DS_Store']:
            relative=p.relative_to(run);target=dest/'run'/relative;copy_verified(p,target)
    if args.deliverable:
        source=Path(args.deliverable).resolve()
        for p in source.rglob('*'):
            rel=p.relative_to(source)
            if p.is_file() and not any(x in ['runtime_cache','__pycache__'] or x.startswith('smoke_') for x in rel.parts):copy_verified(p,dest/'deliverable'/rel)
    dump(dest/'FILE_HASHES.json',{p.relative_to(dest).as_posix():sha(p) for p in dest.rglob('*') if p.is_file()})
    print('Saved and hashed:',dest)

def main():
    p=argparse.ArgumentParser(description=__doc__);subs=p.add_subparsers(dest='command',required=True)
    a=subs.add_parser('restore');a.add_argument('--package',required=True);a.add_argument('--dest',required=True);a.set_defaults(func=restore)
    a=subs.add_parser('check');a.add_argument('--data',required=True);a.add_argument('--full',action='store_true');a.set_defaults(func=check)
    a=subs.add_parser('doctor');a.add_argument('--require-cuda',action='store_true');a.set_defaults(func=doctor)
    a=subs.add_parser('train');a.add_argument('--data',required=True);a.add_argument('--model',required=True);a.add_argument('--name',default='drone_v2_finetune');a.add_argument('--epochs',type=int,default=60);a.add_argument('--patience',type=int,default=15);a.add_argument('--batch',type=int,default=16);a.add_argument('--workers',type=int,default=0);a.add_argument('--device',default='0');a.add_argument('--lr',type=float,default=.001);a.add_argument('--balanced',action='store_true');a.set_defaults(func=train)
    a=subs.add_parser('smoke');a.add_argument('--data',required=True);a.add_argument('--model',required=True);a.add_argument('--device',default='0');a.set_defaults(func=smoke)
    a=subs.add_parser('eval');a.add_argument('--data',required=True);a.add_argument('--model',required=True);a.add_argument('--split',choices=['val','final_test','regression_test'],default='val');a.add_argument('--out',required=True);a.add_argument('--device',default='0');a.add_argument('--batch',type=int,default=1);a.set_defaults(func=evaluate)
    a=subs.add_parser('errors');a.add_argument('--data',required=True);a.add_argument('--model',required=True);a.add_argument('--split',choices=['val','final_test'],default='val');a.add_argument('--out',required=True);a.add_argument('--device',default='0');a.add_argument('--conf',type=float,default=.25);a.add_argument('--iou',type=float,default=.45);a.set_defaults(func=errors)
    a=subs.add_parser('calibration');a.add_argument('--data',required=True);a.add_argument('--out',required=True);a.set_defaults(func=calibration)
    a=subs.add_parser('export');a.add_argument('--model',required=True);a.add_argument('--out',required=True);a.set_defaults(func=export)
    a=subs.add_parser('save');a.add_argument('--run',required=True);a.add_argument('--package',required=True);a.add_argument('--deliverable');a.set_defaults(func=save)
    args=p.parse_args();args.func(args)

if __name__=='__main__':main()
