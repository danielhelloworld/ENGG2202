"""Bundle this module with the existing telemetry30 reader for MaixVision.

Run after checking out feature/maixcam2-micarray-direction locally. Copies the
upstream Sipeed attribution and license next to generated board_main.py.
"""
import argparse
from pathlib import Path
import ast

HERE = Path(__file__).resolve().parent


def build(repo):
    source = Path(repo) / 'apps' / 'thermal_yolo_fusion'
    if not (source / 'src' / 'thermal.py').exists():
        raise SystemExit('Expected apps/thermal_yolo_fusion/src/thermal.py in --repo')
    thermal = '\n'.join((source / 'src' / (name + '.py')).read_text() for name in ('config', 'thermal'))
    detector = (HERE / 'detector.py').read_text()
    runner = '''\nimport time\ndef run():\n    from maix import app\n    reader = thermal_api["ThermalReader"](dict(thermal_api["CONFIG"]))\n    engine = Detector()\n    reader.start()\n    try:\n        while not app.need_exit():\n            frame, source_status, count = reader.snapshot()\n            if frame is not None:\n                frame = dict(frame, reader_count=count)\n            result = engine.update(frame, time.monotonic())\n            print("THERMAL_UAV", result["status"], len(result["candidates"]),\n                  [(t["track_id"], t["center"], t["track_state"]) for t in result["tracks"]])\n            time.sleep(.1)\n    finally:\n        reader.close()\nif __name__ == "__main__":\n    run()\n'''
    bundle = '# Generated, thermal source from existing ENGG2202 app. No motor control.\n'
    bundle += 'thermal_api = {"__name__": "thermal_adapter"}\nexec(' + repr(thermal) + ', thermal_api)\n'
    bundle += detector + runner
    ast.parse(bundle)
    (HERE / 'board_main.py').write_text(bundle)
    for name in ('THIRD_PARTY.md', 'LICENSE_SIPEED.txt'):
        (HERE / name).write_bytes((source / name).read_bytes())
    print('Built board_main.py')

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--repo', required=True)
    build(p.parse_args().repo)
