# Generated, thermal source from existing ENGG2202 app. No motor control.
thermal_api = {"__name__": "thermal_adapter"}
exec('"""Configuration included at the top of the generated standalone main.py."""\nimport os\nimport time\nimport logging\nimport numpy as np\nimport cv2\n\nCONFIG = {\n    # Leave empty to select an installed YOLO11n/s or YOLOv8n model.\n    "model_path": "",\n    "model_type": "YOLO11",  # Used only when model_path is explicitly set.\n    "confidence": 0.50,\n    "iou": 0.45,\n    "max_objects": 12,\n    "width": 640,\n    "height": 480,\n    "camera_fps": 60,  # Requested sensor rate; actual rate depends on sensor/mode.\n    "loop_fps": 0,  # 0 removes the application display/composition FPS cap.\n    "detector_mode": "inline",  # Compatibility default; "process" is experimental on board.\n    "inference_fps": 10,  # Skip inference between scheduled frames; 0 means every opportunity.\n    "camera_read_timeout_ms": 200,\n    "camera_stall_seconds": 10,\n    "diagnostic_trace": True,  # First three loop attempts + stalled Python stacks.\n    "max_detection_age": 0.35,\n    "max_detection_thermal_delta": 0.15,\n    "alpha": 0.40,\n    "thermal_flip_x": True,\n    "thermal_flip_y": True,\n    # Smaller INITIAL footprint; not measured lens FOV. ALIGN tunes each distance.\n    "thermal_rect": [0.25, 0.25, 0.5, 0.5],\n    "manual_adjustments": {},  # Per-distance scale/shift; exported on SAVE.\n    # Installation metadata only; centimetres cannot be converted to pixels without optics/depth.\n    "thermal_offset_m": [0.05, 0.05, 0.0],  # right, down, forward relative to RGB\n    "calibration_distances_m": [0.5, 1.0, 2.0, 3.0, 5.0],\n    "calibration_distance_m": 1.0,\n    "calibration_profiles": {},  # Paste exported profiles here for MaixVision single-file reuse.\n    "alignment_path": "/root/thermal_yolo_fusion/alignment.json",  # Survives MaixVision temp uploads.\n    "calibration_ransac_px": 4.0,\n    "calibration_max_error_px": 6.0,  # RGB model-input pixels, not screen pixels\n    "max_thermal_age": 0.60,\n    "max_pair_delta": 0.25,\n    "hot_min_c": 25.0,  # Candidate must also exceed the frame median by hot_delta_c.\n    "hot_delta_c": 3.0,\n    "hot_min_area_px": 5,  # Original 160x120 thermal pixels, not enlarged display pixels.\n    "hot_max_candidates": 12,\n    "hot_track_gate_px": 14.0,\n    "hot_track_misses": 2,\n    "hot_track_timeout_s": 0.8,\n    "uart_port": "/dev/ttyS2",\n    "configure_uart_pins": True,\n    "reset_pin": "A9",\n    "reset_gpio": "GPIOA9",\n    "reset_active": 0,\n    "skip_frames": 10,\n    "thermal_retry_seconds": 5.0,\n    "rtsp_enabled": True,\n    "rtsp_host": "0.0.0.0",\n    "rtsp_port": 8554,\n    "stream_fps": 30,  # JPEG encode/output ceiling, independent of screen refresh.\n    "jpeg_quality": 70,\n    "max_clients": 2,\n}\n\nLOG = logging.getLogger("thermal_yolo")\nTHERMAL_W, THERMAL_H = 160, 120\nPIXELS = THERMAL_W * THERMAL_H\nBODY_SIZE = PIXELS + 30\nWIRE_SIZE = BODY_SIZE + 1\n\n\n"""TN160 telemetry30 UART input. Protocol derived from Sipeed MaixPy (Apache-2.0).\n\nSee THIRD_PARTY.md. This is not the legacy 19201-byte or USB/UVC protocol.\n"""\nimport struct\nimport threading\n\n\ndef valid_body(body):\n    if len(body) != BODY_SIZE:\n        return False\n    vtemp, lo, hi = struct.unpack_from(">Hhh", body, PIXELS)\n    if vtemp & 0xC000:\n        return False\n    if lo == hi == 32767:\n        return True\n    return -1000 <= lo <= hi <= 3200\n\n\ndef decode_body(body, received_at):\n    vtemp, lo, hi = struct.unpack_from(">Hhh", body, PIXELS)\n    return {\n        "pixels": np.frombuffer(body[:PIXELS], np.uint8).reshape(THERMAL_H, THERMAL_W),\n        "lo": lo, "hi": hi, "vtemp": vtemp & 0x3FFF,\n        "telemetry": body[PIXELS:], "received_at": received_at,\n        "valid_temperature": lo != 32767 and hi != 32767 and hi > lo,\n    }\n\n\nclass ThermalParser:\n    """Bounded incremental parser; acquire sync using two consecutive plausible frames.\n\n    Once locked, accepts a complete frame without throwing it away just because\n    noise follows it. There is no firmware CRC; false sync is still possible.\n    """\n    def __init__(self, skip=10):\n        self.buffer = bytearray()\n        self.locked = False\n        self.skip = skip\n        self.frames = 0\n        self.output_frames = 0\n        self.discarded_bytes = 0\n\n    def feed(self, chunk, received_at=None):\n        now = time.monotonic() if received_at is None else received_at\n        self.buffer.extend(chunk)\n        latest = None\n        while True:\n            idx = self.buffer.find(b"\\xff")\n            if idx < 0:\n                self.discarded_bytes += len(self.buffer)\n                self.buffer.clear()\n                self.locked = False\n                break\n            if idx:\n                self.discarded_bytes += idx\n                del self.buffer[:idx]\n                self.locked = False\n            if len(self.buffer) < WIRE_SIZE:\n                break\n            body = bytes(self.buffer[1:WIRE_SIZE])\n            if not valid_body(body):\n                del self.buffer[0]\n                self.discarded_bytes += 1\n                self.locked = False\n                continue\n            if not self.locked:\n                if len(self.buffer) < WIRE_SIZE * 2:\n                    break\n                next_body = bytes(self.buffer[WIRE_SIZE + 1:WIRE_SIZE * 2])\n                if self.buffer[WIRE_SIZE] != 255 or not valid_body(next_body):\n                    del self.buffer[0]\n                    self.discarded_bytes += 1\n                    continue\n                self.locked = True\n            del self.buffer[:WIRE_SIZE]\n            self.frames += 1\n            if self.skip > 0:\n                self.skip -= 1\n            else:\n                self.output_frames += 1\n                latest = decode_body(body, now)\n            # A frame may end exactly at the read boundary; retain the sync state.\n            if not self.buffer:\n                break\n        return latest\n\n\nclass ThermalReader:\n    """One UART owner thread, latest-frame mailbox, finite reads and reconnection."""\n    def __init__(self, config):\n        self.config = config\n        self.stop_event = threading.Event()\n        self.lock = threading.Lock()\n        self.latest = None\n        self.status = "THERMAL STARTING"\n        self.total_frames = 0\n        self.reset_gpio = None\n        self.thread = threading.Thread(target=self._run, name="thermal-uart", daemon=True)\n\n    def start(self):\n        self.thread.start()\n\n    def snapshot(self):\n        with self.lock:\n            return self.latest, self.status, self.total_frames\n\n    def _status(self, message, clear=False):\n        with self.lock:\n            self.status = message\n            if clear:\n                self.latest = None\n\n    def _open(self):\n        from maix import gpio, pinmap, err\n        from maix.peripheral import uart\n        c = self.config\n        if c["configure_uart_pins"]:\n            for pin, function in (("B0", "UART2_TX"), ("B1", "UART2_RX")):\n                err.check_raise(pinmap.set_pin_function(pin, function), "UART2 pinmap: " + pin)\n        err.check_raise(pinmap.set_pin_function(c["reset_pin"], c["reset_gpio"]), "Thermal reset pinmap")\n        self.reset_gpio = gpio.GPIO(c["reset_gpio"], gpio.Mode.OUT)\n        idle = 1 - c["reset_active"]\n        self.reset_gpio.value(idle)\n        time.sleep(0.02)\n        self.reset_gpio.value(c["reset_active"])\n        time.sleep(0.12)\n        self.reset_gpio.value(idle)\n        time.sleep(0.40)\n        serial = uart.UART(port=c["uart_port"], baudrate=2000000)\n        try:\n            serial.write(b"\\x44")\n        finally:\n            serial.close()\n        time.sleep(0.10)\n        return uart.UART(port=c["uart_port"], baudrate=4000000)\n\n    def _run(self):\n        while not self.stop_event.is_set():\n            serial = None\n            try:\n                self._status("THERMAL CONNECTING", clear=True)\n                LOG.info("THERMAL opening %s / telemetry30", self.config["uart_port"])\n                serial = self._open()\n                LOG.info("THERMAL UART ready; waiting for valid frames")\n                parser = ThermalParser(self.config["skip_frames"])\n                last_valid = time.monotonic()\n                while not self.stop_event.is_set():\n                    try:\n                        chunk = serial.read(32768, timeout=2)\n                    except TypeError:\n                        chunk = serial.read(32768, 2)\n                    if chunk:\n                        before = parser.output_frames\n                        frame = parser.feed(bytes(chunk))\n                        if frame is not None:\n                            if before == 0:\n                                LOG.info("THERMAL first frame received; temperature_valid=%s", frame["valid_temperature"])\n                            last_valid = time.monotonic()\n                            with self.lock:\n                                self.latest = frame\n                                self.status = "THERMAL OK" if frame["valid_temperature"] else "UNCALIBRATED"\n                                self.total_frames += parser.output_frames - before\n                    else:\n                        self.stop_event.wait(0.001)\n                    if time.monotonic() - last_valid > self.config["thermal_retry_seconds"]:\n                        raise RuntimeError("No telemetry30 frames; check module firmware/wiring")\n            except Exception as exc:\n                LOG.warning("Thermal input: %s", exc)\n                self._status("THERMAL OFFLINE / CHECK FIRMWARE", clear=True)\n            finally:\n                if serial is not None:\n                    try:\n                        serial.close()\n                    except Exception:\n                        LOG.exception("UART close failed")\n            self.stop_event.wait(1.0)\n\n    def close(self):\n        self.stop_event.set()\n        if self.thread.is_alive():\n            self.thread.join(timeout=3.0)\n        if self.thread.is_alive():\n            LOG.warning("UART thread did not exit within 3 seconds")\n\n', thermal_api)
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

import time
def run():
    from maix import app
    reader = thermal_api["ThermalReader"](dict(thermal_api["CONFIG"]))
    engine = Detector()
    reader.start()
    try:
        while not app.need_exit():
            frame, source_status, count = reader.snapshot()
            if frame is not None:
                frame = dict(frame, reader_count=count)
            result = engine.update(frame, time.monotonic())
            print("THERMAL_UAV", result["status"], len(result["candidates"]),
                  [(t["track_id"], t["center"], t["track_state"]) for t in result["tracks"]])
            time.sleep(.1)
    finally:
        reader.close()
if __name__ == "__main__":
    run()
