"""Render exact STL geometry into part thumbnails and an offline HTML index."""
import base64
import hashlib
import html
import json
from pathlib import Path
import struct
import sys
from urllib.parse import quote

import numpy as np
from PIL import Image, ImageDraw, ImageFont

DEST = Path(__file__).resolve().parent
ROOT = DEST.parent
PACK = ROOT / 'STL_单件打印_2026-10-03'
FONT = 'C:/Windows/Fonts/msyh.ttc'
VERSIONS = ['V3 USB 优化版', 'V2 原位置加强版', 'V2 原版', 'V1 历史试作（未采用）']

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def old_hashes():
    return {p.relative_to(ROOT).as_posix(): sha(p)
            for p in ROOT.rglob('*') if p.is_file() and DEST not in p.parents}

def read_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3,3)), ('attribute','<u2')])
    assert len(data) == 84 + 50 * count
    return np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].astype(np.float64)

def render(triangles, direction, size, color):
    w, h = size
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    right = np.cross([0,0,1], direction)
    right /= np.linalg.norm(right)
    up = np.cross(direction, right)
    center = (triangles.min(axis=(0,1)) + triangles.max(axis=(0,1))) / 2
    geometry = triangles - center
    projected = np.stack([geometry @ right, -(geometry @ up), geometry @ direction], axis=-1)
    lo, hi = projected[:,:,:2].min(axis=(0,1)), projected[:,:,:2].max(axis=(0,1))
    scale = min((w-64)/(hi[0]-lo[0]), (h-48)/(hi[1]-lo[1]))
    projected[:,:,:2] = (projected[:,:,:2] - (lo+hi)/2) * scale + [w/2,h/2]
    normals = np.cross(geometry[:,1]-geometry[:,0], geometry[:,2]-geometry[:,0])
    normals /= np.linalg.norm(normals, axis=1)[:,None]
    light = np.array([.2,-.65,.85])
    light /= np.linalg.norm(light)
    fill = np.array([.8,.4,.3])
    fill /= np.linalg.norm(fill)
    brightness = .40 + .49*np.maximum(0,normals@light) + .14*np.maximum(0,normals@fill)
    rgb = np.clip(np.asarray(color)[None,:]*brightness[:,None], 0,255).astype(np.uint8)
    buffer = np.full((h,w,3), [245,248,252], dtype=np.uint8)
    depth = np.full((h,w), -np.inf)
    visible = normals @ direction > 1e-8
    for index in np.flatnonzero(visible):
        t = projected[index]
        xmin,ymin = np.maximum([0,0], np.floor(t[:,:2].min(axis=0)).astype(int))
        xmax,ymax = np.minimum([w-1,h-1], np.ceil(t[:,:2].max(axis=0)).astype(int))
        if xmin>xmax or ymin>ymax:
            continue
        x, y = np.meshgrid(np.arange(xmin,xmax+1)+.5,np.arange(ymin,ymax+1)+.5)
        a,b,c = t
        denom = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(denom)<1e-10:
            continue
        u = ((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/denom
        v = ((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/denom
        z = u*a[2]+v*b[2]+(1-u-v)*c[2]
        area = depth[ymin:ymax+1,xmin:xmax+1]
        mask = (u>=-1e-8)&(v>=-1e-8)&(u+v<=1+1e-8)&(z>area)
        area[mask] = z[mask]
        buffer[ymin:ymax+1,xmin:xmax+1][mask] = rgb[index]
    mask = np.isfinite(depth)
    assert np.count_nonzero(mask)>100
    inside = mask.copy()
    inside[1:,:] &= mask[:-1,:]
    inside[:-1,:] &= mask[1:,:]
    inside[:,1:] &= mask[:,:-1]
    inside[:,:-1] &= mask[:,1:]
    buffer[mask & ~inside] = np.asarray(color)*.48
    return Image.fromarray(buffer)

def thumb(triangles, item, index):
    canvas = Image.new('RGB', (800,600), '#f5f8fc')
    color = {'底座':(59,150,211), '相机板':(121,104,206), '铆销':(231,154,51), '试装件':(48,170,157)}[item['category']]
    canvas.paste(render(triangles,[1.7,-2.2,1.7],(780,386),color),(10,86))
    # A second, independent lower view reveals the integrated stakes and recesses.
    inset = render(triangles,[1.7,-2.2,-1.5],(250,170),color)
    canvas.paste(inset,(536,414))
    draw = ImageDraw.Draw(canvas)
    regular = lambda size: ImageFont.truetype(FONT,size)
    draw.text((24,16),f'P{index:02d}  {item["title"]}',font=regular(24),fill='#172c43')
    draw.text((24,52),item['version'],font=regular(18),fill='#667b91')
    draw.rounded_rectangle((536,414,786,584),radius=10,outline='#d3deea',width=2)
    draw.text((548,421),'底部视角',font=regular(16),fill='#667b91')
    dims = ' × '.join(f'{x:.2f}' for x in item['dimensions'])
    draw.text((24,493),dims+' mm',font=regular(22),fill='#172c43')
    draw.text((24,533),f'建议数量：{item["quantity"]}  |  独立视图缩放',font=regular(18),fill='#667b91')
    return canvas

def main():
    if (DEST/'零件索引.html').exists() and '--refresh' not in sys.argv:
        raise RuntimeError('Preview index already exists; refusing to overwrite.')
    baseline = old_hashes()
    source_report = json.loads((PACK/'单件网格验证.json').read_text(encoding='utf-8'))
    cards = []
    preview_dir = DEST/'缩略图'
    preview_dir.mkdir(exist_ok=True)
    ordered_parts = sorted(source_report['parts'], key=lambda p: (
        p['file'].split('/')[0], '试装件/' in p['file'], Path(p['file']).name))
    for index, part in enumerate(ordered_parts,1):
        source = PACK/part['file']
        assert sha(source) == part['target_sha256']
        folder = part['file'].split('/')[0]
        version_index = int(folder[:2])-1
        title = source.stem[3:].replace('1-4', '1/4').replace('_', ' · ')
        if '试装件/' in part['file']:
            category = '试装件'
        elif '底座' in title:
            category = '底座'
        elif '铆销' in title:
            category = '铆销'
        else:
            category = '相机板'
        item = {'id':f'P{index:02d}', 'title':title, 'version':VERSIONS[version_index],
                'version_index':version_index, 'category':category, 'quantity':part['quantity'],
                'dimensions':part['mesh']['dimensions_mm'], 'use':part['use'],
                'stl':'../'+quote(PACK.name+'/'+part['file'],safe='/'),
                'source':part['file'], 'sha256':sha(source)}
        target = preview_dir/f'P{index:02d}.png'
        thumb(read_stl(source),item,index).save(target)
        item['thumbnail']='缩略图/'+target.name
        item['data_image']='data:image/png;base64,'+base64.b64encode(target.read_bytes()).decode('ascii')
        cards.append(item)
    # One full contact sheet per version gives an image-only overview.
    for vi, version in enumerate(VERSIONS):
        subset = [c for c in cards if c['version_index']==vi]
        sheet = Image.new('RGB',(2400,100+600*((len(subset)+2)//3)),'#f5f8fc')
        draw = ImageDraw.Draw(sheet)
        draw.text((30,24),version+' — 单件打印索引',font=ImageFont.truetype(FONT,38),fill='#172c43')
        for n, item in enumerate(subset):
            with Image.open(DEST/item['thumbnail']) as img:
                sheet.paste(img,(800*(n%3),100+600*(n//3)))
        sheet.save(DEST/f'{vi+1:02d}_零件总览.png')
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>MaixCAM2 · 单件打印索引</title>
<style>
:root{color-scheme:light;--ink:#172c43;--muted:#63778b;--line:#dce5ed;--blue:#1766a9}*{box-sizing:border-box}body{margin:0;background:#f3f6fa;color:var(--ink);font-family:"Microsoft YaHei",system-ui,sans-serif}main{max-width:1480px;margin:auto;padding:34px 28px}.eyebrow{font-size:12px;letter-spacing:2px;color:var(--blue);font-weight:700}h1{font-size:32px;margin:10px 0 12px}p{line-height:1.7;color:var(--muted);margin:8px 0}.stats{display:flex;gap:20px;margin:22px 0;font-size:14px}.stats strong{font-size:24px;margin-right:6px;color:var(--blue)}.controls{position:sticky;top:0;z-index:5;display:flex;gap:12px;flex-wrap:wrap;background:#f3f6faee;padding:14px 0;backdrop-filter:blur(12px)}select,input{font:inherit;border:1px solid var(--line);padding:12px 14px;border-radius:9px;background:white;color:var(--ink)}input{flex:1;min-width:220px}.status{font-size:14px;margin:10px 0 20px;color:var(--muted)}.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:20px}.card{border:1px solid var(--line);background:white;border-radius:14px;overflow:hidden;box-shadow:0 5px 18px #152b4010}.card img{display:block;width:100%;height:auto}.body{padding:16px 18px}.tags{display:flex;gap:6px;align-items:center;font-size:12px;color:var(--muted)}.badge{background:#edf4fb;padding:4px 8px;border-radius:5px;color:var(--blue)}h2{font-size:17px;margin:12px 0 8px}.use{min-height:42px;font-size:13px;line-height:1.6;color:var(--muted)}.filename{font-size:11px;line-height:1.65;color:#728497;word-break:break-all;margin:10px 0}.actions{display:flex;gap:12px}.actions a{text-decoration:none;font-size:13px;padding:9px 12px;border:1px solid var(--line);border-radius:7px;color:var(--blue)}.actions a.primary{background:var(--blue);color:white;border-color:var(--blue)}.note{background:#eaf1f8;border-left:3px solid var(--blue);padding:12px 16px;font-size:13px;border-radius:4px}.empty{padding:40px;text-align:center;color:var(--muted)}footer{font-size:12px;color:var(--muted);margin-top:28px;line-height:1.8}@media(max-width:1000px){.grid{grid-template-columns:repeat(2,minmax(0,1fr))}}@media(max-width:650px){main{padding:24px 16px}.grid{grid-template-columns:1fr}h1{font-size:25px}.controls>*{width:100%}.stats{gap:12px;font-size:12px}}
</style><main><div class="eyebrow">MAIXCAM2 / GIMBAL CONNECTOR</div><h1>找到你要打印的零件</h1><p>按版本、零件类型或名称筛选，查看真实 STL 外形，再打开对应打印文件。</p><div class="stats"><span><strong>34</strong>单件缩略图</span><span><strong>4</strong>版本目录</span><span><strong>2</strong>每件视角</span></div><div class="note">每套选择一个底座和一种相机板。加强版与 USB 版另需 2 颗侧向热铆销。底部一体热铆柱随底座打印；V1 是未采用的历史方向。</div><div class="controls"><select id="version" aria-label="版本"><option value="0">V3 USB 优化版 · 当前版</option><option value="1">V2 原位置加强版</option><option value="2">V2 原版</option><option value="3">V1 历史方向试作 · 未采用</option><option value="all">全部版本</option></select><select id="category" aria-label="零件类型"><option value="all">全部零件类型</option><option>底座</option><option>相机板</option><option>铆销</option><option>试装件</option></select><input id="search" type="search" placeholder="搜索：热铆、M2、四角、0.25…" aria-label="搜索零件"></div><div class="status" id="status"></div><div class="grid" id="grid"></div><footer>图像由对应 STL 网格直接渲染，含上方斜视图与底部视图；每件独立缩放，缩略图大小不能用于比较实物尺寸。模型尺寸单位为 mm，视图不代表推荐打印方向。<br>本索引可离线打开；请保留本预览目录与原 STL 打印目录的相对位置。原模型和 STL 文件没有修改。</footer></main><script>
const parts=__DATA__;
const escapeText=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function update(){const v=document.getElementById('version').value,c=document.getElementById('category').value,q=document.getElementById('search').value.trim().toLowerCase();const found=parts.filter(p=>(v==='all'||p.version_index===Number(v))&&(c==='all'||p.category===c)&&(!q||(p.title+' '+p.source+' '+p.use).toLowerCase().includes(q)));document.getElementById('status').textContent=`当前显示 ${found.length} / 34 个零件 · 试装件为可选打印`;document.getElementById('grid').innerHTML=found.length?found.map(p=>`<article class="card"><a href="${p.thumbnail}" target="_blank"><img src="${p.data_image}" alt="${escapeText(p.title)}：上方与底部视图"></a><div class="body"><div class="tags"><span class="badge">${p.id} · ${escapeText(p.category)}</span><span>${escapeText(p.version)}</span></div><h2>${escapeText(p.title)}</h2><div class="use">${escapeText(p.use)} · 建议 ${p.quantity} 件</div><div class="filename">${escapeText(p.source)}</div><div class="actions"><a class="primary" href="${p.stl}" download>获取 STL</a><a href="${p.thumbnail}" target="_blank">查看大图</a></div></div></article>`).join(''):'<div class="empty">没有匹配零件，请调整筛选条件。</div>';}['version','category','search'].forEach(id=>document.getElementById(id).addEventListener('input',update));update();
</script></html>'''
    (DEST/'零件索引.html').write_text(page.replace('__DATA__',json.dumps(cards,ensure_ascii=False)),encoding='utf-8')
    if old_hashes()!=baseline:
        raise RuntimeError('An existing project file changed during preview generation.')
    report = {'thumbnail_count':len(cards),'overview_count':4,'existing_files_unchanged':len(baseline),
              'rendering':'STL triangles, orthographic projection, z-buffer visibility, two directions',
              'parts':[{k:v for k,v in c.items() if k!='data_image'} for c in cards]}
    (DEST/'缩略图清单.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (DEST/'README.md').write_text('# 单件打印缩略图\n\n用浏览器打开 `零件索引.html`，可按版本、类型和名称筛选，点击“获取 STL”访问对应文件。默认显示当前 V3 USB 版。\n\n`缩略图/` 包含每个单件的 PNG，`01_零件总览.png` 至 `04_零件总览.png` 为各版总览。图片来自真实 STL 网格，每件有上方和底部两个视角。\n\n原有项目文件均未改动。请保持本目录与 `STL_单件打印_2026-10-03/` 并列，以便索引内的 STL 链接正常工作。\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='parts'},ensure_ascii=True))

if __name__=='__main__':
    main()
