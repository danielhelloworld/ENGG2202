"""Fixed-camera Thermal160 candidate detector and thermal-coordinate tracker.

Consumes decoded telemetry30 frames from the existing ThermalReader. No UART ownership.
Pure Python so the PC replay needs no third-party packages.
"""
import math
import statistics
import time

W, H = 160, 120
DEFAULT = dict(max_age=0.60, future_tolerance=0.02, confirm_hits=3,
               max_misses=2, hold_seconds=0.8, gate_px=9.0,
               min_contrast=7.0, max_candidates=24, bad_pixels=[])


def _mean_image(values, radius):
    # Integral image, clipped windows, preserving single-pixel detail.
    integral = [[0] * (W + 1) for _ in range(H + 1)]
    for y in range(H):
        row_sum = 0
        for x in range(W):
            row_sum += values[y][x]
            integral[y + 1][x + 1] = integral[y][x + 1] + row_sum
    out = [[0.0] * W for _ in range(H)]
    for y in range(H):
        y0, y1 = max(0, y - radius), min(H, y + radius + 1)
        for x in range(W):
            x0, x1 = max(0, x - radius), min(W, x + radius + 1)
            total = integral[y1][x1] - integral[y0][x1] - integral[y1][x0] + integral[y0][x0]
            out[y][x] = total / ((x1 - x0) * (y1 - y0))
    return out


def _pixels(frame):
    pixels = frame['pixels']
    if hasattr(pixels, 'tolist'):
        pixels = pixels.tolist()
    if len(pixels) != H or any(len(row) != W for row in pixels):
        raise ValueError('pixels must be 120x160')
    if any(not isinstance(v, int) or v < 0 or v > 255 for row in pixels for v in row):
        raise ValueError('pixels must be uint8 values')
    return pixels


def detect(pixels, config=None):
    c = dict(DEFAULT, **(config or {}))
    # One scale keeps point targets; larger scales retain broader hot patches.
    means = [_mean_image(pixels, r) for r in (2, 5, 11)]
    response = [[max(pixels[y][x] - means[i][y][x] for i in range(3))
                 for x in range(W)] for y in range(H)]
    sample = [response[y][x] for y in range(2, H - 2, 3) for x in range(2, W - 2, 3)]
    median = statistics.median(sample)
    mad = statistics.median(abs(v - median) for v in sample)
    threshold = max(c['min_contrast'], median + 4 * max(mad, 1.0))
    bad = {tuple(p) for p in c['bad_pixels']}
    active = [[response[y][x] >= threshold and (x, y) not in bad
               and 1 < x < W - 2 and 1 < y < H - 2 for x in range(W)] for y in range(H)]
    seen = set()
    output = []
    for y in range(2, H - 2):
        for x in range(2, W - 2):
            if not active[y][x] or (x, y) in seen:
                continue
            todo, component = [(x, y)], []
            seen.add((x, y))
            while todo:
                px, py = todo.pop()
                component.append((px, py))
                for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)):
                    q = (px + dx, py + dy)
                    if 0 <= q[0] < W and 0 <= q[1] < H and active[q[1]][q[0]] and q not in seen:
                        seen.add(q)
                        todo.append(q)
            xs, ys = [p[0] for p in component], [p[1] for p in component]
            bx, by = min(xs), min(ys)
            bw, bh = max(xs) - bx + 1, max(ys) - by + 1
            area = len(component)
            if area > 500 or max(bw, bh) / max(1, min(bw, bh)) > 8:
                continue
            contrast = max(response[py][px] for px, py in component)
            # Edge-rich structures are less useful evidence; this is a heuristic score.
            shape = min(1.0, 4 * area / max(1, bw * bh))
            quality = min(1.0, contrast / max(2 * threshold, 1)) * shape
            output.append(dict(bbox=[bx, by, bw, bh], center=[sum(xs)/area, sum(ys)/area],
                               area=area, local_contrast=round(contrast, 3),
                               quality_score=round(quality, 3)))
    output.sort(key=lambda z: (-z['quality_score'], -z['local_contrast']))
    return output[:c['max_candidates']]


class Detector:
    def __init__(self, config=None):
        self.config = dict(DEFAULT, **(config or {}))
        self.tracks = []
        self.serial = 0
        self.last_stamp = None
        self.last_frame_id = None
        self.previous_pixels = None
        self.motion_cooldown = 0

    def _snapshot(self, valid, status, reason, stamp=None, candidates=None, frame_id=None, motion_quality=1.0):
        return dict(frame_id=frame_id, received_at=stamp, valid=valid, status=status, reason=reason,
                    candidates=candidates or [], tracks=[dict(t, detected_this_frame=False, predicted=True, track_state='HOLD', quality_score=0.0) if not valid else dict(t) for t in self.tracks],
                    motion_quality=motion_quality)

    def update(self, frame, now=None):
        now = time.monotonic() if now is None else float(now)
        c = self.config
        self.tracks = [t for t in self.tracks if now - t['last_seen_at'] <= c['hold_seconds']]
        if frame is None:
            return self._snapshot(False, 'NO_FRAME', 'reader has no frame')
        stamp = frame.get('received_at')
        fid = frame.get('frame_id', frame.get('reader_count'))
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return self._snapshot(False, 'INVALID', 'missing received_at', stamp, frame_id=fid)
        if stamp > now + c['future_tolerance']:
            return self._snapshot(False, 'FUTURE', 'received_at is in future', stamp, frame_id=fid)
        if now - stamp > c['max_age']:
            return self._snapshot(False, 'STALE', 'thermal frame timed out', stamp, frame_id=fid)
        if not frame.get('valid_temperature', False):
            return self._snapshot(False, 'INVALID_TEMPERATURE', 'telemetry lo/hi invalid', stamp, frame_id=fid)
        lo, hi = frame.get('lo'), frame.get('hi')
        if not isinstance(lo, int) or not isinstance(hi, int) or not (-1000 <= lo < hi <= 3200):
            return self._snapshot(False, 'INVALID_TEMPERATURE', 'telemetry range invalid', stamp, frame_id=fid)
        if (self.last_stamp is not None and stamp <= self.last_stamp) or (fid is not None and fid == self.last_frame_id):
            return self._snapshot(False, 'DUPLICATE', 'no new received frame', stamp, frame_id=fid)
        try:
            pixels = _pixels(frame)
        except (KeyError, TypeError, ValueError) as exc:
            return self._snapshot(False, 'INVALID_PIXELS', str(exc), stamp, frame_id=fid)
        self.last_stamp, self.last_frame_id = stamp, fid
        candidates = detect(pixels, c)
        # Global gain changes are removed from the motion check by median offset.
        motion_quality = 1.0
        if self.previous_pixels is not None:
            diffs = [pixels[y][x] - self.previous_pixels[y][x] for y in range(0, H, 4) for x in range(0, W, 4)]
            offset = statistics.median(diffs)
            changed = sum(abs(d-offset) > 12 for d in diffs) / len(diffs)
            if changed > .35:
                self.motion_cooldown = 3
        if self.motion_cooldown:
            motion_quality = .25  # fixed-camera assumption likely broken
            self.motion_cooldown -= 1
        self.previous_pixels = pixels
        for i, item in enumerate(candidates):
            item['observation_id'] = '%s:%d' % (str(fid if fid is not None else stamp), i)
            item['track_id'] = None
            item['detected_this_frame'] = True
            item['predicted'] = False
            item['quality_score'] = round(item['quality_score'] * motion_quality, 3)
        possible = []
        for ti, track in enumerate(self.tracks):
            dt = max(0, stamp - track['last_seen_at'])
            px = track['center'][0] + track['velocity'][0] * min(dt, .5)
            py = track['center'][1] + track['velocity'][1] * min(dt, .5)
            for ci, item in enumerate(candidates):
                distance = math.hypot(item['center'][0]-px, item['center'][1]-py)
                ratio = item['area']/max(1, track['area'])
                if distance <= c['gate_px'] + 8*min(dt, .5) and .2 <= ratio <= 5:
                    possible.append((distance + 2*abs(math.log(ratio)), ti, ci))
        possible.sort()
        used_t, used_c = set(), set()
        for cost, ti, ci in possible:
            if ti in used_t or ci in used_c:
                continue
            # Reject close alternatives on either side, including crossing targets.
            if any((tj == ti or cj == ci) and abs(other-cost) < 2 for other, tj, cj in possible
                   if (other, tj, cj) != (cost, ti, ci)):
                candidates[ci]['quality_score'] *= .5
                continue
            t, item = self.tracks[ti], candidates[ci]
            dt = stamp - t['last_seen_at']
            if dt > 0:
                t['velocity'] = [(item['center'][j]-t['center'][j])/dt for j in (0, 1)]
            t.update(center=item['center'], bbox=item['bbox'], area=item['area'],
                     local_contrast=item['local_contrast'], quality_score=item['quality_score'],
                     hit_count=t['hit_count']+1, miss_count=0, last_seen_at=stamp,
                     detected_this_frame=True, predicted=False)
            t['track_state'] = 'CONFIRMED' if t['hit_count'] >= c['confirm_hits'] and motion_quality == 1 else 'TENTATIVE'
            item['track_id'] = t['track_id']
            used_t.add(ti); used_c.add(ci)
        for ti, t in enumerate(self.tracks):
            if ti not in used_t:
                t['miss_count'] += 1
                t['detected_this_frame'] = False
                t['predicted'] = True
                t['track_state'] = 'HOLD'
                dt = min(max(0, stamp - t['last_seen_at']), .5)
                t['predicted_center'] = [t['center'][j] + t['velocity'][j]*dt for j in (0, 1)]
                t['quality_score'] = 0.0
        self.tracks = [t for t in self.tracks if t['miss_count'] <= c['max_misses']]
        for ci, item in enumerate(candidates):
            if ci not in used_c:
                self.serial += 1
                item['track_id'] = self.serial
                self.tracks.append(dict(track_id=self.serial, bbox=item['bbox'], center=item['center'],
                    area=item['area'], local_contrast=item['local_contrast'],
                    quality_score=item['quality_score'], hit_count=1, miss_count=0,
                    last_seen_at=stamp, velocity=[0.0, 0.0], detected_this_frame=True,
                    predicted=False, track_state='TENTATIVE'))
        for item in candidates:
            t = next((t for t in self.tracks if t['track_id'] == item['track_id']), None)
            if t:
                item.update(hit_count=t['hit_count'], miss_count=t['miss_count'],
                            last_seen_at=t['last_seen_at'], track_state=t['track_state'])
        status = 'MOTION_UNCERTAIN' if motion_quality < 1 else 'OK'
        return self._snapshot(True, status, 'fixed-camera motion check' if motion_quality < 1 else '',
                              stamp, candidates, fid, motion_quality)
