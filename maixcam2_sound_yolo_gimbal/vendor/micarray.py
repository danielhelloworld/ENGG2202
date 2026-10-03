"""MaixCAM2 + MA-USB8 UART sound-direction display.

Open this single file in MaixVision and run it. The core classes are importable
without MaixPy, so another application can call SoundDirectionSensor.feed_bytes()
or poll() and read snapshot(). No camera, YOLO, thermal module or audio input.
"""

import math
import statistics
import time


# Standalone wiring: MA-USB8 TX0 -> MaixCAM2 A31 (UART1 RX),
# MA-USB8 RX0 -> A30 (UART1 TX), common GND. Power MA-USB8 by its USB-C.
# UART1 leaves Thermal160 UART2 and MaixVision communication UART4 alone.
UART_PORT = "/dev/ttyS1"
UART_BAUD = 2_000_000
ROTATION_DEG = 0  # Clockwise screen correction: 0/90/180/270.
MIRROR_X = False
DISPLAY_INTERVAL_S = 0.08
STALE_AFTER_S = 1.0
CENTER_DEAD_ZONE = 0.14  # About one 16x16 cell; do not invent a bearing near center.

HEADER = b"\xff" * 16
FRAME_SIZE = 272
MAX_BUFFER = FRAME_SIZE * 64
PIN_MAP = {
    "/dev/ttyS1": (("A30", "UART1_TX"), ("A31", "UART1_RX")),
    "/dev/ttyS2": (("B0", "UART2_TX"), ("B1", "UART2_RX")),
    "/dev/ttyS4": (("A21", "UART4_TX"), ("A22", "UART4_RX")),
}


class HotmapFrameParser:
    """Incremental 16xFF + 256-byte parser; confirms the following header."""

    def __init__(self):
        self.buffer = bytearray()
        self.frames = 0
        self.discarded_bytes = 0

    def feed(self, chunk):
        if not chunk:
            return []
        self.buffer.extend(chunk)
        if len(self.buffer) > MAX_BUFFER:
            drop = len(self.buffer) - MAX_BUFFER
            del self.buffer[:drop]
            self.discarded_bytes += drop
        output = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                drop = max(0, len(self.buffer) - len(HEADER) + 1)
                del self.buffer[:drop]
                self.discarded_bytes += drop
                break
            if start:
                del self.buffer[:start]
                self.discarded_bytes += start
            if len(self.buffer) < FRAME_SIZE + len(HEADER):
                break
            if self.buffer[FRAME_SIZE:FRAME_SIZE + len(HEADER)] != HEADER:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            output.append(bytes(self.buffer[16:FRAME_SIZE]))
            del self.buffer[:FRAME_SIZE]
            self.frames += 1
        return output


def orient_xy(x, y, rotation=0, mirror=False):
    """Transform x-right/y-up heatmap coordinates to mounted orientation."""
    if rotation not in (0, 90, 180, 270):
        raise ValueError("rotation must be 0/90/180/270")
    if rotation == 90:
        x, y = y, -x
    elif rotation == 180:
        x, y = -x, -y
    elif rotation == 270:
        x, y = -y, x
    return (-x if mirror else x), y


class SoundDirectionEstimator:
    """Estimate the dominant region in a hotmap; angle is NOT calibrated DOA."""

    def __init__(self, smoothing_s=0.18, stale_after_s=STALE_AFTER_S):
        self.smoothing_s = smoothing_s
        self.stale_after_s = stale_after_s
        self.last_time = None
        self.center = None
        self.last = {"valid": False, "reason": "no_data", "quality": 0.0}

    def update(self, payload, now=None):
        if len(payload) != 256:
            raise ValueError("hotmap payload must be 256 bytes")
        now = time.monotonic() if now is None else float(now)
        if self.last_time is not None and now < self.last_time:
            raise ValueError("timestamp moved backward")
        gap = 2.0 if self.last_time is None else now - self.last_time
        self.last_time = now
        values = list(payload)
        background = statistics.median(values)
        sigma = 1.4826 * statistics.median(abs(v - background) for v in values)
        contrast = max(values) - background
        threshold = max(6.0, 3.0 * sigma, 0.25 * contrast)
        hot = {i for i, value in enumerate(values) if value > background + threshold}
        components = []
        while hot:
            seed = hot.pop()
            stack = [seed]
            cells = [seed]
            while stack:
                index = stack.pop()
                row, col = divmod(index, 16)
                for rr in range(max(0, row - 1), min(16, row + 2)):
                    for cc in range(max(0, col - 1), min(16, col + 2)):
                        neighbour = rr * 16 + cc
                        if neighbour in hot:
                            hot.remove(neighbour)
                            stack.append(neighbour)
                            cells.append(neighbour)
            if len(cells) >= 3:
                weights = [(values[i] - background - threshold) ** 1.5 for i in cells]
                components.append((sum(weights), cells, weights))
        if not components:
            self.center = None
            self.last = {"valid": False, "reason": "low_contrast_or_noise", "quality": 0.0,
                         "background": background, "contrast": contrast, "component_count": 0}
            return self.snapshot(now)
        components.sort(key=lambda item: item[0], reverse=True)
        mass, cells, weights = components[0]
        dominance = mass / sum(item[0] for item in components)
        if dominance < 0.60:
            self.center = None
            self.last = {"valid": False, "reason": "ambiguous_multiple_sources", "quality": 0.0,
                         "background": background, "contrast": contrast,
                         "component_count": len(components), "dominance": dominance}
            return self.snapshot(now)
        x = sum(((i % 16) - 7.5) / 7.5 * weight for i, weight in zip(cells, weights)) / mass
        y = sum((7.5 - (i // 16)) / 7.5 * weight for i, weight in zip(cells, weights)) / mass
        if (self.center is None or gap >= 2 or self.smoothing_s <= 0
                or math.hypot(x - self.center[0], y - self.center[1]) > 0.5):
            self.center = (x, y)
        else:
            factor = -math.expm1(-gap / self.smoothing_s)
            self.center = (self.center[0] + (x - self.center[0]) * factor,
                           self.center[1] + (y - self.center[1]) * factor)
        rows = [i // 16 for i in cells]
        cols = [i % 16 for i in cells]
        self.last = {
            "valid": True, "reason": "ok", "vector_xy": self.center,
            "centroid_vector_xy": (x, y),
            "raw_bbox_rc": (min(rows), min(cols), max(rows) + 1, max(cols) + 1),
            "quality": dominance * min(1.0, contrast / max(12.0, 6.0 * sigma)),
            "dominance": dominance, "region_cells": len(cells),
            "background": background, "contrast": contrast, "component_count": len(components),
        }
        return self.snapshot(now)

    def snapshot(self, now=None, rotation=0, mirror=False):
        now = time.monotonic() if now is None else float(now)
        result = dict(self.last)
        age = None if self.last_time is None else max(0.0, now - self.last_time)
        result["age_seconds"] = age
        result["coordinate_system"] = "normalized_heatmap_x_right_y_up"
        result["rotation"] = rotation
        result["mirror"] = bool(mirror)
        if age is not None and age >= self.stale_after_s:
            result["valid"] = False
            result["reason"] = "stale"
        if not result["valid"]:
            for key in ("centroid_vector_xy", "raw_bbox_rc", "region_cells"):
                result.pop(key, None)
            result["vector_xy"] = None
            result["unit_direction_xy"] = None
            result["image_bearing_deg"] = None
            result["quality"] = 0.0
            return result
        x, y = orient_xy(*result["vector_xy"], rotation, mirror)
        cx, cy = orient_xy(*result["centroid_vector_xy"], rotation, mirror)
        result["vector_xy"] = [x, y]
        result["centroid_vector_xy"] = [cx, cy]
        length = math.hypot(x, y)
        result["unit_direction_xy"] = [x / length, y / length] if length >= CENTER_DEAD_ZONE else None
        result["image_bearing_deg"] = (math.degrees(math.atan2(x, y)) % 360
                                        if length >= CENTER_DEAD_ZONE else None)
        return result


class SoundDirectionSensor:
    """Reusable stream API: feed_bytes(), poll(), snapshot(), close().

    Instantiate without a serial port for integration with an external reader.
    open_uart() is intentionally explicit, so importing never claims a port.
    """

    def __init__(self, rotation=0, mirror=False, stale_after_s=STALE_AFTER_S):
        orient_xy(0, 0, rotation, mirror)
        self.rotation = rotation
        self.mirror = mirror
        self.parser = HotmapFrameParser()
        self.estimator = SoundDirectionEstimator(stale_after_s=stale_after_s)
        self.serial = None
        self.last_payload = None
        self.total_frames = 0

    def feed_bytes(self, data, now=None):
        now = time.monotonic() if now is None else float(now)
        frames = self.parser.feed(data)
        for payload in frames:
            self.last_payload = payload
            self.estimator.update(payload, now)
            self.total_frames += 1
        return len(frames)

    def open_uart(self, port=UART_PORT, baudrate=UART_BAUD, configure_pins=True):
        if self.serial is not None:
            raise RuntimeError("UART already open")
        from maix import err, pinmap
        from maix.peripheral import uart
        if configure_pins:
            if port not in PIN_MAP:
                raise ValueError("Unknown pin mapping; set configure_pins=False only after checking hardware")
            for pin, function in PIN_MAP[port]:
                err.check_raise(pinmap.set_pin_function(pin, function), "pinmap " + pin)
        self.serial = uart.UART(port=port, baudrate=baudrate)
        return self.serial

    def poll(self, read_size=4096, timeout_ms=2):
        if self.serial is None:
            raise RuntimeError("UART not open")
        try:
            data = self.serial.read(read_size, timeout=timeout_ms)
        except TypeError:
            data = self.serial.read(read_size, timeout_ms)
        return self.feed_bytes(bytes(data)) if data else 0

    def snapshot(self, now=None):
        result = self.estimator.snapshot(now, self.rotation, self.mirror)
        result["frames"] = self.total_frames
        result["discarded_bytes"] = self.parser.discarded_bytes
        return result

    def close(self):
        serial, self.serial = self.serial, None
        if serial is not None:
            serial.close()
