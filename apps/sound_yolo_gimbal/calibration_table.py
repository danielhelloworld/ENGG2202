"""PC helper: aggregate measured CSV rows into a sound calibration table.

CSV columns: vx,vy,yaw_deg,pitch_deg. At least 3 distinct poses, >=5 readings each.
Does not command hardware or set sound_calibrated=True automatically.
"""
import argparse
import csv
import json
import math
import statistics
from collections import defaultdict


def make_table(rows):
    groups = defaultdict(list)
    for row in rows:
        vx, vy, yaw, pitch = (float(row[k]) for k in ("vx", "vy", "yaw_deg", "pitch_deg"))
        if not all(math.isfinite(v) for v in (vx, vy, yaw, pitch)) or abs(vx) > 1 or abs(vy) > 1:
            raise ValueError("Nonfinite or out-of-range heatmap coordinates")
        groups[(yaw, pitch)].append((vx, vy))
    if len(groups) < 3:
        raise ValueError("Measure at least 3 distinct directions")
    table = []
    for (yaw, pitch), vectors in sorted(groups.items()):
        if len(vectors) < 5:
            raise ValueError("At least 5 readings required per direction")
        center = tuple(statistics.median(p[i] for p in vectors) for i in range(2))
        if max(math.hypot(p[0]-center[0], p[1]-center[1]) for p in vectors) > .12:
            raise ValueError("Unstable sound samples at yaw=%s pitch=%s" % (yaw, pitch))
        table.append([*center, yaw, pitch])
    for index, first in enumerate(table):
        for second in table[index+1:]:
            if math.hypot(first[0]-second[0], first[1]-second[1]) < .05:
                raise ValueError("Directions have indistinguishable heatmap vectors; repeat calibration")
    return {"sound_samples": table, "sound_calibrated": False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    args = parser.parse_args()
    with open(args.csv_path, encoding="utf-8-sig", newline="") as file:
        result = make_table(csv.DictReader(file))
    print(json.dumps(result, indent=2))
