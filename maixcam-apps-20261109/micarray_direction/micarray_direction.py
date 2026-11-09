"""MaixCAM2 + MA-USB8 UART sound-direction display.

Open this single file in MaixVision and run it. The core classes are importable
without MaixPy, so another application can call SoundDirectionSensor.feed_bytes()
or poll() and read snapshot(). No camera, YOLO, thermal module or audio input.
"""

import json
import math
import os
import statistics
import time


# Standalone wiring: MA-USB8 TX0 -> MaixCAM2 A31 (UART1 RX),
# Configuration control also needs MaixCAM2 A30 (UART1 TX) -> MA-USB8 RX0.
# Common GND and independent power are required. No motor commands are sent.
# UART1 leaves Thermal160 UART2 and MaixVision communication UART4 alone.
UART_PORT = "/dev/ttyS1"
UART_BAUD = 2_000_000
ROTATION_DEG = 0  # Clockwise screen correction: 0/90/180/270.
MIRROR_X = False
DISPLAY_INTERVAL_S = 0.08
LOG_INTERVAL_S = 1.0
ENABLE_DISPLAY = True  # False: UART diagnostics in MaixVision console only.
CONFIG_ENABLED = True  # False: retain the original two-wire, read-only test.
CONFIG_PATH = "/root/micarray_direction/config.json"
CONFIG_CHECK_INTERVAL_S = 1.0
COMMAND_INTERVAL_S = 0.03
DEFAULT_CONFIG = {"detection_threshold": 650, "led_enabled": True}
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


def validate_board_config(value):
    if not isinstance(value, dict) or set(value) != set(DEFAULT_CONFIG):
        raise ValueError("Config needs exactly detection_threshold and led_enabled")
    threshold = value["detection_threshold"]
    if type(threshold) is not int or not 0 <= threshold <= 2000 or threshold % 50:
        raise ValueError("detection_threshold must be an integer 0..2000 in steps of 50")
    if type(value["led_enabled"]) is not bool:
        raise ValueError("led_enabled must be JSON true or false")
    return dict(value)


def save_board_config(path, value):
    """Atomically save on the CAMERA filesystem, not in MA-USB8 flash."""
    value = validate_board_config(value)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    temp = path + ".tmp"
    try:
        with open(temp, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(value, indent=2) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.remove(temp)


def load_board_config(path, create=False):
    try:
        with open(path, "r", encoding="utf-8-sig") as stream:
            text = stream.read(4097)
    except FileNotFoundError:
        if not create:
            raise
        save_board_config(path, DEFAULT_CONFIG)
        return dict(DEFAULT_CONFIG)
    if len(text) > 4096:
        raise ValueError("Config is larger than 4096 characters")
    return validate_board_config(json.loads(text))


class BoardConfigController:
    """Paced documented t/T/e/E requests; no ACK, readback or flash command.

    An unknown threshold is driven to the documented lower bound by 40 t's,
    then raised in steps of 50. Correct execution depends on firmware clamping
    and every command arriving; the requested value is never called readback.
    """

    def __init__(self, serial):
        self.serial = serial
        self.desired = None
        self.pending = []
        self.last_send = None
        self.sent_commands = 0
        self.status = "NOT_REQUESTED"
        self.error = None

    def submit(self, value):
        value = validate_board_config(value)
        if value == self.desired:
            return False
        led_only = (self.status == "SENT_UNCONFIRMED" and self.desired is not None
                    and value["detection_threshold"] == self.desired["detection_threshold"])
        self.pending = ([] if led_only else [b"t"] * 40
                        + [b"T"] * (value["detection_threshold"] // 50))
        self.pending.append(b"E" if value["led_enabled"] else b"e")
        self.desired = value
        self.status = "PENDING"
        self.error = None
        print("MICARRAY_CONFIG_REQUEST " + json.dumps(self.snapshot()), flush=True)
        return True

    def cancel(self, exc):
        self.pending = []
        self.desired = None
        self.status = "CONFIG_ERROR"
        self.error = str(exc)

    def tick(self, now):
        if not self.pending or (self.last_send is not None
                                and now - self.last_send < COMMAND_INTERVAL_S):
            return
        command = self.pending[0]
        try:
            written = self.serial.write(command)
            if written != len(command):
                raise IOError("UART short write: %r" % written)
        except Exception as exc:
            self.pending = []
            self.status = "WRITE_FAILED"
            self.error = str(exc)
            print("MICARRAY_CONFIG_WRITE_FAILED", repr(exc), flush=True)
            return
        del self.pending[0]
        self.last_send = now
        self.sent_commands += 1
        if not self.pending:
            self.status = "SENT_UNCONFIRMED"
            print("MICARRAY_CONFIG_SENT_UNCONFIRMED " + json.dumps(self.snapshot()), flush=True)

    def snapshot(self):
        return {"requested": self.desired, "status": self.status,
                "pending_commands": len(self.pending), "sent_commands": self.sent_commands,
                "error": self.error, "execution_confirmed": False,
                "driver_flash_save_supported": False}


class ConfigFileWatcher:
    """Reload valid JSON changes without repeatedly shifting the threshold."""

    def __init__(self, path, controller):
        self.path = path
        self.controller = controller
        self.last_check = None
        self.last_error = None

    def poll(self, now, create=False):
        if self.last_check is not None and now - self.last_check < CONFIG_CHECK_INTERVAL_S:
            return
        self.last_check = now
        try:
            value = load_board_config(self.path, create=create)
        except Exception as exc:
            self.controller.cancel(exc)
            message = repr(exc)
            if message != self.last_error:
                print("MICARRAY_CONFIG_FILE_ERROR path=%s %s" % (self.path, message), flush=True)
            self.last_error = message
            return
        self.last_error = None
        self.controller.submit(value)


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
        if (self.center is None or gap >= self.stale_after_s or self.smoothing_s <= 0
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
        self.received_bytes = 0
        self.last_byte_time = None
        self.first_rx_hex = ""
        self.last_rx_hex = ""
        self.byte_rate = 0.0
        self.frame_rate = 0.0
        self._rate_time = None
        self._rate_bytes = 0
        self._rate_frames = 0
        self.board_config = None

    def feed_bytes(self, data, now=None):
        now = time.monotonic() if now is None else float(now)
        if data:
            self.received_bytes += len(data)
            self.last_byte_time = now
            self.last_rx_hex = bytes(data[:32]).hex(" ")
            if not self.first_rx_hex:
                self.first_rx_hex = self.last_rx_hex
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
        now = time.monotonic() if now is None else float(now)
        result = self.estimator.snapshot(now, self.rotation, self.mirror)
        result["frames"] = self.total_frames
        result["discarded_bytes"] = self.parser.discarded_bytes
        result["received_bytes"] = self.received_bytes
        result["buffered_bytes"] = len(self.parser.buffer)
        result["byte_age_seconds"] = (None if self.last_byte_time is None
                                       else max(0.0, now - self.last_byte_time))
        result["bytes_per_second"] = self.byte_rate
        result["frames_per_second"] = self.frame_rate
        if self.last_byte_time is None:
            status = "WAIT_BYTES"
        elif result["byte_age_seconds"] >= self.estimator.stale_after_s:
            status = "UART_SILENT"
        elif result["age_seconds"] is None or result["reason"] == "stale":
            status = "BYTES_NO_FRAME"
        elif not result["valid"]:
            status = "FRAME_NO_DIRECTION"
        elif result["unit_direction_xy"] is None:
            status = "SOURCE_CENTER"
        else:
            status = "DIRECTION_OK"
        result["status"] = status
        result["map_direction"] = map_direction(result["image_bearing_deg"])
        return result

    def sample_rates(self, now=None):
        """Update rates over actual elapsed time, not over display refreshes."""
        now = time.monotonic() if now is None else float(now)
        if self._rate_time is not None and now > self._rate_time:
            elapsed = now - self._rate_time
            self.byte_rate = (self.received_bytes - self._rate_bytes) / elapsed
            self.frame_rate = (self.total_frames - self._rate_frames) / elapsed
        self._rate_time = now
        self._rate_bytes = self.received_bytes
        self._rate_frames = self.total_frames

    def diagnostic_record(self, now=None):
        result = self.snapshot(now)
        result["first_rx_hex"] = self.first_rx_hex
        result["last_rx_hex"] = self.last_rx_hex
        result["hint"] = STATUS_HINTS[result["status"]]
        if self.board_config is not None:
            result["board_control"] = self.board_config.snapshot()
        return result

    def close(self):
        serial, self.serial = self.serial, None
        if serial is not None:
            serial.close()


STATUS_HINTS = {
    "WAIT_BYTES": "Check MA-USB8 power, TX0 -> A31, common GND and UART1 ownership",
    "UART_SILENT": "UART bytes stopped; check power, cable and slip-ring contact",
    "BYTES_NO_FRAME": "Waiting for binary sync; if persistent, check 2M baud, output mode and HEX",
    "FRAME_NO_DIRECTION": "Fresh heatmaps received; try one steady sound source and inspect reason",
    "SOURCE_CENTER": "Source near map center; no unique map bearing",
    "DIRECTION_OK": "Fresh heatmap with dominant source; map bearing is not calibrated physical DOA",
}


def map_direction(angle):
    if angle is None:
        return "--"
    names = ("UP", "UP-RIGHT", "RIGHT", "DOWN-RIGHT", "DOWN", "DOWN-LEFT", "LEFT", "UP-LEFT")
    return names[int((angle + 22.5) // 45) % 8]


def display_hotmap(img, payload, left, top, size, image, rotation=0, mirror=False):
    """Draw a raw grid; display enhancement never affects the direction estimate."""
    cell = max(1, size // 16)
    background = statistics.median(payload) if payload else 0
    peak = max(payload) if payload else 1
    span = max(24, peak - background)
    oriented = [0] * 256
    if payload:
        for row in range(16):
            for col in range(16):
                x, y = orient_xy((col - 7.5) / 7.5, (7.5 - row) / 7.5, rotation, mirror)
                new_col = round(7.5 + 7.5 * x)
                new_row = round(7.5 - 7.5 * y)
                oriented[new_row * 16 + new_col] = payload[row * 16 + col]
    for row in range(16):
        for col in range(16):
            value = oriented[row * 16 + col]
            t = min(1.0, max(0.0, (value - background) / span))
            color = image.Color.from_rgb(int(15 + 230 * t), int(30 + 150 * t), int(50 + 25 * (1 - t)))
            img.draw_rect(left + col * cell, top + row * cell, cell - 1, cell - 1, color, -1)
    img.draw_rect(left - 1, top - 1, cell * 16 + 2, cell * 16 + 2,
                  image.Color.from_rgb(110, 125, 145), 2)
    return cell * 16


def draw_screen(disp, sensor):
    from maix import image
    width, height = disp.width(), disp.height()
    img = image.Image(width, height, image.Format.FMT_RGB888,
                      bg=image.Color.from_rgb(8, 14, 24))
    white = image.Color.from_rgb(235, 242, 250)
    cyan = image.Color.from_rgb(50, 225, 220)
    yellow = image.Color.from_rgb(255, 204, 75)
    muted = image.Color.from_rgb(145, 160, 180)
    img.draw_string(12, 10, "MICARRAY MODULE TEST", white, scale=1.5)
    mode = "TX=A30 RX=A31" if CONFIG_ENABLED else "RX=A31 READ ONLY"
    img.draw_string(12, 42, "UART1 %s | 2000000 8N1" % mode, muted)
    result = sensor.snapshot()
    img.draw_string(12, 68, result["status"], cyan if result["status"] == "DIRECTION_OK" else yellow)
    size = max(64, min(width - 72, height - 280, 256))
    size = size // 16 * 16
    left, top = (width - size) // 2, 114
    fresh = (result["age_seconds"] is not None
             and result["age_seconds"] < sensor.estimator.stale_after_s)
    payload = sensor.last_payload if fresh else None
    drawn = display_hotmap(img, payload, left, top, size, image,
                           sensor.rotation, sensor.mirror)
    cx, cy = left + drawn // 2, top + drawn // 2
    img.draw_string(cx - 8, top - 20, "UP", muted)
    img.draw_string(left - 20, cy - 8, "L", muted)
    img.draw_string(left + drawn + 8, cy - 8, "R", muted)
    img.draw_circle(cx, cy, 5, white, -1)
    if result["valid"] and result["unit_direction_xy"] is not None:
        x, y = result["unit_direction_xy"]
        length = int(drawn * 0.40)
        end_x, end_y = int(cx + x * length), int(cy - y * length)
        img.draw_line(cx, cy, end_x, end_y, cyan, 5)
        wing = max(10, drawn // 20)
        img.draw_line(end_x, end_y, int(end_x - x * wing + y * wing * .6),
                      int(end_y + y * wing + x * wing * .6), cyan, 4)
        img.draw_line(end_x, end_y, int(end_x - x * wing - y * wing * .6),
                      int(end_y + y * wing - x * wing * .6), cyan, 4)
        line = "MAP %03.0f DEG | %s" % (result["image_bearing_deg"], result["map_direction"])
        quality = "QUALITY %.2f | PEAK AREA %d" % (result["quality"], result["region_cells"])
    elif result["valid"]:
        line, quality = "SOURCE NEAR CENTER", "NO UNIQUE BEARING"
    else:
        line = "NO DIRECTION"
        quality = "REASON: " + result["reason"].upper()
    y0 = top + drawn + 12
    img.draw_string(12, y0, line, yellow, scale=1.2)
    img.draw_string(12, y0 + 26, quality, muted)
    age = "--" if result["age_seconds"] is None else "%.1f" % result["age_seconds"]
    img.draw_string(12, y0 + 52, "RX %d B | %.0f B/s | %.1f FPS" %
                    (result["received_bytes"], result["bytes_per_second"], result["frames_per_second"]), muted)
    img.draw_string(12, y0 + 76, "FRAMES %d | AGE %ss | DISCARD %d B" %
                    (result["frames"], age, result["discarded_bytes"]), muted)
    if sensor.board_config is not None:
        config = sensor.board_config.snapshot()
        requested = config["requested"]
        settings = ("THR %d LED %s" % (requested["detection_threshold"],
                                      "ON" if requested["led_enabled"] else "OFF")
                    if requested else "CONFIG INVALID")
        img.draw_string(12, y0 + 100, settings + " | " + config["status"], muted)
    else:
        img.draw_string(12, y0 + 100, "Map direction only; physical angle NOT calibrated", muted)
    disp.show(img)


def main():
    from maix import app, display, sys
    if str(sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART pin configuration is for MaixCAM2")
    sensor = SoundDirectionSensor(rotation=ROTATION_DEG, mirror=MIRROR_X)
    disp = None
    try:
        print("MICARRAY_START port=%s baud=%d read_only=%s" % (UART_PORT, UART_BAUD, not CONFIG_ENABLED), flush=True)
        print("WIRING: MA-USB8 TX0 -> A31; GND -> GND; independent power", flush=True)
        if CONFIG_ENABLED:
            print("CONFIG WIRING: MaixCAM2 A30 -> MA-USB8 RX0 is REQUIRED", flush=True)
        sensor.open_uart(UART_PORT, UART_BAUD)
        print("MICARRAY_UART_READY (port opened; no received signal confirmed yet)", flush=True)
        watcher = None
        if CONFIG_ENABLED:
            print("MICARRAY_CONFIG_PATH", CONFIG_PATH, flush=True)
            print("CONFIG saved on camera; requests have no ACK; driver flash persistence unsupported", flush=True)
            sensor.board_config = BoardConfigController(sensor.serial)
            watcher = ConfigFileWatcher(CONFIG_PATH, sensor.board_config)
            watcher.poll(time.monotonic(), create=True)
        if ENABLE_DISPLAY:
            try:
                disp = display.Display()
            except Exception as exc:
                print("MICARRAY_DISPLAY_ERROR console diagnostics continue:", repr(exc), flush=True)
        last_draw = 0.0
        last_log = time.monotonic()
        sensor.sample_rates(last_log)
        previous_status = None
        while not app.need_exit():
            sensor.poll()
            now = time.monotonic()
            if watcher is not None:
                watcher.poll(now)
                sensor.board_config.tick(now)
            status = sensor.snapshot(now)["status"]
            if now - last_log >= LOG_INTERVAL_S:
                sensor.sample_rates(now)
                print("MICARRAY_DIAG " + json.dumps(sensor.diagnostic_record(now)), flush=True)
                last_log = now
            if status != previous_status:
                print("MICARRAY_STATE %s: %s" % (status, STATUS_HINTS[status]), flush=True)
                previous_status = status
            if disp is not None and now - last_draw >= DISPLAY_INTERVAL_S:
                try:
                    draw_screen(disp, sensor)
                except Exception as exc:
                    print("MICARRAY_DRAW_ERROR console diagnostics continue:", repr(exc), flush=True)
                    try:
                        if hasattr(disp, "close"):
                            disp.close()
                    except Exception as close_exc:
                        print("MICARRAY_CLOSE_ERROR", repr(close_exc), flush=True)
                    finally:
                        disp = None
                last_draw = now
    except Exception as exc:
        print("MICARRAY_ERROR", repr(exc), flush=True)
        raise
    finally:
        for resource in (sensor, disp):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    print("MICARRAY_CLOSE_ERROR", repr(exc), flush=True)
        print("MICARRAY_STOP", flush=True)


if __name__ == "__main__":
    main()
