"""PC JSONL replay and explicitly synthetic sample generation."""
import argparse
import json
import time
from pathlib import Path
from detector import Detector


def synthetic():
    for i in range(12):
        pixels = [[35 + (i // 5) * 8 for _ in range(160)] for _ in range(120)]
        if i not in (5, 6):
            pixels[45][30 + min(i, 5)] = 115  # point target
        if i >= 3:
            for y in range(70, 75):
                for x in range(90, 96):
                    pixels[y][x] = 105  # larger hot patch
        pixels[20][120] = 255  # fixed bad pixel
        if i >= 9:
            for y in range(20, 100):
                for x in range(20, 140):
                    pixels[y][x] = min(255, pixels[y][x] + 25)  # synthetic scene motion / background change
        yield dict(frame_id=i, received_at=i*.1, lo=100, hi=400,
                   valid_temperature=True, pixels=pixels)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', help='JSONL: one decoded frame per line, pixels as 120 lists of 160 uint8')
    p.add_argument('--synthetic', action='store_true')
    p.add_argument('--output', default='replay_results.jsonl')
    a = p.parse_args()
    if not a.synthetic and not a.input:
        p.error('supply --synthetic or --input')
    frames = synthetic() if a.synthetic else (json.loads(line) for line in open(a.input) if line.strip())
    detector = Detector({'bad_pixels': [[120, 20]]} if a.synthetic else None)
    durations = []
    with open(a.output, 'w') as f:
        for frame in frames:
            begin = time.perf_counter()
            result = detector.update(frame, frame['received_at'])
            durations.append((time.perf_counter()-begin)*1000)
            f.write(json.dumps(dict(dataset='SYNTHETIC' if a.synthetic else 'REPLAY', **result))+'\n')
    print('%s frames=%d median_ms=%.2f output=%s' %
          ('SYNTHETIC' if a.synthetic else 'REPLAY', len(durations), sorted(durations)[len(durations)//2], a.output))

if __name__ == '__main__':
    main()
