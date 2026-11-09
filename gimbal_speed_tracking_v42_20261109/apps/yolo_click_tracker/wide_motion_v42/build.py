"""Build a new standalone entry; never write the previous main.py or src files."""
import ast
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
APP=ROOT.parent
OUTPUT=APP/'main_wide_motion_v42.py'


def verify_previous():
    expected=json.loads((ROOT/'previous_files_sha256.json').read_text(encoding='utf-8'))
    changed=[name for name,digest in expected.items()
             if not (APP/name).is_file() or hashlib.sha256((APP/name).read_bytes()).hexdigest()!=digest]
    if changed:raise RuntimeError('Previous files differ; refusing to silently change baseline: '+str(changed))
    return len(expected)


def build():
    count=verify_previous()
    baseline=(APP/'main.py').read_text(encoding='utf-8')
    marker='\nif __name__ == "__main__":\n    tracker_main()'
    if baseline.count(marker)!=1:raise RuntimeError('Baseline entry guard changed')
    prefix,suffix=baseline.rsplit(marker,1)
    extension=(ROOT/'overlay.py').read_text(encoding='utf-8')
    result=('# Standalone wide-motion-v42; previous versions are preserved.\n'
            '# New profile and loss policy are under WIDE_CONFIG near the end.\n'+prefix+
            '\n# Independent v42 extension\n'+extension+marker+suffix)
    ast.parse(result)
    OUTPUT.write_text(result,encoding='utf-8')
    report={'baseline_sha256':hashlib.sha256((APP/'main.py').read_bytes()).hexdigest(),
            'overlay_sha256':hashlib.sha256((ROOT/'overlay.py').read_bytes()).hexdigest(),
            'output_sha256':hashlib.sha256(OUTPUT.read_bytes()).hexdigest(),
            'protected_previous_files':count}
    (ROOT/'build_hashes.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('Built',OUTPUT,'; verified',count,'previous files unchanged')


if __name__=='__main__':build()
