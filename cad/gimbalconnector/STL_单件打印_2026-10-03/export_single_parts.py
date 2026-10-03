"""Create a separate slicer package without modifying existing project files."""
import ast
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import zipfile

DEST = Path(__file__).resolve().parent
ROOT = DEST.parent
VERSIONS = {
    'output_v3_usb': '01_V3_USB优化版',
    'output_v2_reinforced': '02_V2_原位置加强版',
    'output_v2': '03_V2_原版',
    'output_v1': '04_V1_历史方向试作_未采用',
}
NAMES = {
    'receiver_heat_stake.stl': ('01_摩擦快拆底座_一体热铆柱.stl', 1, '底座二选一'),
    'receiver_M2_bolt.stl': ('02_摩擦快拆底座_M2螺丝固定.stl', 1, '底座二选一'),
    'camera_plate.stl': ('03_相机快装板_中心1-4英寸螺丝.stl', 1, '相机板二选一'),
    'camera_plate_four_screw.stl': ('04_相机快装板_中心螺丝及四角固定件.stl', 1, '相机板二选一；立边与板一体成型'),
    'side_heat_rivet_print_2.stl': ('05_独立侧向热铆销_单颗_打印2颗.stl', 2, '加强版和USB版侧向两孔固定'),
    'fit_coupon_male.stl': ('试装件/01_燕尾公件.stl', 1, '可选间隙试装件'),
    'fit_coupon_gap_0.15.stl': ('试装件/02_燕尾母件_单侧间隙0.15mm.stl', 1, '可选间隙试装件'),
    'fit_coupon_gap_0.25.stl': ('试装件/03_燕尾母件_单侧间隙0.25mm.stl', 1, '可选间隙试装件'),
    'fit_coupon_gap_0.35.stl': ('试装件/04_燕尾母件_单侧间隙0.35mm.stl', 1, '可选间隙试装件'),
}

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def original_hashes():
    return {p.relative_to(ROOT).as_posix(): sha(p)
            for p in ROOT.rglob('*') if p.is_file() and DEST not in p.parents}

def main():
    if any((DEST / name).exists() for name in VERSIONS.values()):
        raise RuntimeError('Output already exists; refusing to overwrite.')
    baseline = original_hashes()
    # Reuse only the read-only topology checker, with no module startup or cache.
    tree = ast.parse((ROOT / 'quick_release_tool/validate_optimized.py').read_text(encoding='utf-8-sig'))
    checker = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'check')
    namespace = {'struct': struct, 'collections': collections, 'math': math}
    exec(compile(ast.Module(body=[checker], type_ignores=[]), '<topology_checker>', 'exec'), namespace)
    check = namespace['check']
    entries = []
    for source_folder, target_folder in VERSIONS.items():
        sources = sorted((ROOT / source_folder).glob('*.stl'))
        expected = 9 if source_folder in ('output_v2_reinforced', 'output_v3_usb') else 8
        if len(sources) != expected:
            raise ValueError(f'Unexpected source count: {source_folder}')
        for source in sources:
            target_name, quantity, use = NAMES[source.name]
            target = DEST / target_folder / target_name
            before = check(source)
            lo, hi = before['bbox_mm']
            translation = [-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]]
            data = bytearray(source.read_bytes())
            count = struct.unpack_from('<I', data, 80)[0]
            for triangle in range(count):
                for vertex in range(3):
                    offset = 84 + 50 * triangle + 12 + 12 * vertex
                    values = struct.unpack_from('<3f', data, offset)
                    struct.pack_into('<3f', data, offset, *(values[i] + translation[i] for i in range(3)))
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            after = check(target)
            dimension_error = max(abs(a-b) for a,b in zip(before['dimensions_mm'], after['dimensions_mm']))
            volume_error = abs(after['volume_mm3'] / before['volume_mm3'] - 1)
            after_lo, after_hi = after['bbox_mm']
            centered_error = max(abs(after_lo[i] + after_hi[i]) for i in (0,1))
            original_data = source.read_bytes()
            unchanged_records = all(
                data[84+50*i:96+50*i] == original_data[84+50*i:96+50*i]
                and data[132+50*i:134+50*i] == original_data[132+50*i:134+50*i]
                for i in range(count)
            )
            passed = (after['connected_meshes'] == 1 and after['boundary_edges'] == 0
                      and after['nonmanifold_edges'] == 0 and after['inconsistent_edges'] == 0
                      and after['degenerate_triangles'] == 0 and after['volume_mm3'] > 0
                      and dimension_error < .0001 and volume_error < .00001
                      and abs(after_lo[2]) < .00001 and centered_error < .00001
                      and unchanged_records and before['triangles'] == after['triangles'])
            if not passed:
                raise ValueError(f'Validation failed: {target}')
            entries.append({
                'source': source.relative_to(ROOT).as_posix(),
                'file': target.relative_to(DEST).as_posix(), 'quantity': quantity, 'use': use,
                'units': 'mm', 'translation_mm': translation,
                'source_sha256': sha(source), 'target_sha256': sha(target),
                'dimension_error_mm': dimension_error, 'relative_volume_error': volume_error,
                'normals_and_attributes_preserved': unchanged_records, 'mesh': after, 'passed': passed,
            })
    current = original_hashes()
    if current != baseline:
        raise RuntimeError('Existing project files changed during export.')
    report = {'stl_count': len(entries), 'existing_files_verified_unchanged': len(baseline),
              'existing_file_hashes': baseline, 'all_passed': all(e['passed'] for e in entries),
              'operation': 'translation only; XY centered and minimum Z zero; no rotation or rescaling',
              'parts': entries}
    (DEST / '单件网格验证.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    with (DEST / '打印清单.csv').open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['STL文件', '建议数量', '用途', 'X尺寸mm', 'Y尺寸mm', 'Z尺寸mm', '来源'])
        for entry in entries:
            writer.writerow([entry['file'], entry['quantity'], entry['use'],
                             *[round(x, 4) for x in entry['mesh']['dimensions_mm']], entry['source']])
    archive = DEST / '单件STL打印包.zip'
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as handle:
        for path in sorted(DEST.rglob('*')):
            if path.is_file() and path != archive and path != Path(__file__).resolve():
                handle.write(path, path.relative_to(DEST).as_posix())
    print(json.dumps({'directory': str(DEST), 'STL_files': len(entries),
                      'all_meshes_passed': report['all_passed'],
                      'existing_files_unchanged': len(baseline),
                      'zip_bytes': archive.stat().st_size}, ensure_ascii=True))

if __name__ == '__main__':
    main()
