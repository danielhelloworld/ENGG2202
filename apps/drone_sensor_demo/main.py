# Generated MaixVision entry. Edit core.py/board_runtime.py then run build.py.
thermal_api = {"__name__": "thermal_adapter"}
exec('"""Configuration included at the top of the generated standalone main.py."""\nimport os\nimport time\nimport logging\nimport numpy as np\nimport cv2\n\nCONFIG = {\n    # Leave empty to select an installed YOLO11n/s or YOLOv8n model.\n    "model_path": "",\n    "model_type": "YOLO11",  # Used only when model_path is explicitly set.\n    "confidence": 0.50,\n    "iou": 0.45,\n    "max_objects": 12,\n    "width": 640,\n    "height": 480,\n    "camera_fps": 60,  # Requested sensor rate; actual rate depends on sensor/mode.\n    "loop_fps": 0,  # 0 removes the application display/composition FPS cap.\n    "detector_mode": "inline",  # Compatibility default; "process" is experimental on board.\n    "inference_fps": 10,  # Skip inference between scheduled frames; 0 means every opportunity.\n    "camera_read_timeout_ms": 200,\n    "camera_stall_seconds": 10,\n    "diagnostic_trace": True,  # First three loop attempts + stalled Python stacks.\n    "max_detection_age": 0.35,\n    "max_detection_thermal_delta": 0.15,\n    "alpha": 0.40,\n    "thermal_flip_x": True,\n    "thermal_flip_y": True,\n    # Smaller INITIAL footprint; not measured lens FOV. ALIGN tunes each distance.\n    "thermal_rect": [0.25, 0.25, 0.5, 0.5],\n    "manual_adjustments": {},  # Per-distance scale/shift; exported on SAVE.\n    # Installation metadata only; centimetres cannot be converted to pixels without optics/depth.\n    "thermal_offset_m": [0.05, 0.05, 0.0],  # right, down, forward relative to RGB\n    "calibration_distances_m": [0.5, 1.0, 2.0, 3.0, 5.0],\n    "calibration_distance_m": 1.0,\n    "calibration_profiles": {},  # Paste exported profiles here for MaixVision single-file reuse.\n    "alignment_path": "/root/thermal_yolo_fusion/alignment.json",  # Survives MaixVision temp uploads.\n    "calibration_ransac_px": 4.0,\n    "calibration_max_error_px": 6.0,  # RGB model-input pixels, not screen pixels\n    "max_thermal_age": 0.60,\n    "max_pair_delta": 0.25,\n    "hot_min_c": 25.0,  # Candidate must also exceed the frame median by hot_delta_c.\n    "hot_delta_c": 3.0,\n    "hot_min_area_px": 5,  # Original 160x120 thermal pixels, not enlarged display pixels.\n    "hot_max_candidates": 12,\n    "hot_track_gate_px": 14.0,\n    "hot_track_misses": 2,\n    "hot_track_timeout_s": 0.8,\n    "uart_port": "/dev/ttyS2",\n    "configure_uart_pins": True,\n    "reset_pin": "A9",\n    "reset_gpio": "GPIOA9",\n    "reset_active": 0,\n    "skip_frames": 10,\n    "thermal_retry_seconds": 5.0,\n    "rtsp_enabled": True,\n    "rtsp_host": "0.0.0.0",\n    "rtsp_port": 8554,\n    "stream_fps": 30,  # JPEG encode/output ceiling, independent of screen refresh.\n    "jpeg_quality": 70,\n    "max_clients": 2,\n}\n\nLOG = logging.getLogger("thermal_yolo")\nTHERMAL_W, THERMAL_H = 160, 120\nPIXELS = THERMAL_W * THERMAL_H\nBODY_SIZE = PIXELS + 30\nWIRE_SIZE = BODY_SIZE + 1\n\n"""TN160 telemetry30 UART input. Protocol derived from Sipeed MaixPy (Apache-2.0).\n\nSee THIRD_PARTY.md. This is not the legacy 19201-byte or USB/UVC protocol.\n"""\nimport struct\nimport threading\n\n\ndef valid_body(body):\n    if len(body) != BODY_SIZE:\n        return False\n    vtemp, lo, hi = struct.unpack_from(">Hhh", body, PIXELS)\n    if vtemp & 0xC000:\n        return False\n    if lo == hi == 32767:\n        return True\n    return -1000 <= lo <= hi <= 3200\n\n\ndef decode_body(body, received_at):\n    vtemp, lo, hi = struct.unpack_from(">Hhh", body, PIXELS)\n    return {\n        "pixels": np.frombuffer(body[:PIXELS], np.uint8).reshape(THERMAL_H, THERMAL_W),\n        "lo": lo, "hi": hi, "vtemp": vtemp & 0x3FFF,\n        "telemetry": body[PIXELS:], "received_at": received_at,\n        "valid_temperature": lo != 32767 and hi != 32767 and hi > lo,\n    }\n\n\nclass ThermalParser:\n    """Bounded incremental parser; acquire sync using two consecutive plausible frames.\n\n    Once locked, accepts a complete frame without throwing it away just because\n    noise follows it. There is no firmware CRC; false sync is still possible.\n    """\n    def __init__(self, skip=10):\n        self.buffer = bytearray()\n        self.locked = False\n        self.skip = skip\n        self.frames = 0\n        self.output_frames = 0\n        self.discarded_bytes = 0\n\n    def feed(self, chunk, received_at=None):\n        now = time.monotonic() if received_at is None else received_at\n        self.buffer.extend(chunk)\n        latest = None\n        while True:\n            idx = self.buffer.find(b"\\xff")\n            if idx < 0:\n                self.discarded_bytes += len(self.buffer)\n                self.buffer.clear()\n                self.locked = False\n                break\n            if idx:\n                self.discarded_bytes += idx\n                del self.buffer[:idx]\n                self.locked = False\n            if len(self.buffer) < WIRE_SIZE:\n                break\n            body = bytes(self.buffer[1:WIRE_SIZE])\n            if not valid_body(body):\n                del self.buffer[0]\n                self.discarded_bytes += 1\n                self.locked = False\n                continue\n            if not self.locked:\n                if len(self.buffer) < WIRE_SIZE * 2:\n                    break\n                next_body = bytes(self.buffer[WIRE_SIZE + 1:WIRE_SIZE * 2])\n                if self.buffer[WIRE_SIZE] != 255 or not valid_body(next_body):\n                    del self.buffer[0]\n                    self.discarded_bytes += 1\n                    continue\n                self.locked = True\n            del self.buffer[:WIRE_SIZE]\n            self.frames += 1\n            if self.skip > 0:\n                self.skip -= 1\n            else:\n                self.output_frames += 1\n                latest = decode_body(body, now)\n            # A frame may end exactly at the read boundary; retain the sync state.\n            if not self.buffer:\n                break\n        return latest\n\n\nclass ThermalReader:\n    """One UART owner thread, latest-frame mailbox, finite reads and reconnection."""\n    def __init__(self, config):\n        self.config = config\n        self.stop_event = threading.Event()\n        self.lock = threading.Lock()\n        self.latest = None\n        self.status = "THERMAL STARTING"\n        self.total_frames = 0\n        self.reset_gpio = None\n        self.thread = threading.Thread(target=self._run, name="thermal-uart", daemon=True)\n\n    def start(self):\n        self.thread.start()\n\n    def snapshot(self):\n        with self.lock:\n            return self.latest, self.status, self.total_frames\n\n    def _status(self, message, clear=False):\n        with self.lock:\n            self.status = message\n            if clear:\n                self.latest = None\n\n    def _open(self):\n        from maix import gpio, pinmap, err\n        from maix.peripheral import uart\n        c = self.config\n        if c["configure_uart_pins"]:\n            for pin, function in (("B0", "UART2_TX"), ("B1", "UART2_RX")):\n                err.check_raise(pinmap.set_pin_function(pin, function), "UART2 pinmap: " + pin)\n        err.check_raise(pinmap.set_pin_function(c["reset_pin"], c["reset_gpio"]), "Thermal reset pinmap")\n        self.reset_gpio = gpio.GPIO(c["reset_gpio"], gpio.Mode.OUT)\n        idle = 1 - c["reset_active"]\n        self.reset_gpio.value(idle)\n        time.sleep(0.02)\n        self.reset_gpio.value(c["reset_active"])\n        time.sleep(0.12)\n        self.reset_gpio.value(idle)\n        time.sleep(0.40)\n        serial = uart.UART(port=c["uart_port"], baudrate=2000000)\n        try:\n            serial.write(b"\\x44")\n        finally:\n            serial.close()\n        time.sleep(0.10)\n        return uart.UART(port=c["uart_port"], baudrate=4000000)\n\n    def _run(self):\n        while not self.stop_event.is_set():\n            serial = None\n            try:\n                self._status("THERMAL CONNECTING", clear=True)\n                LOG.info("THERMAL opening %s / telemetry30", self.config["uart_port"])\n                serial = self._open()\n                LOG.info("THERMAL UART ready; waiting for valid frames")\n                parser = ThermalParser(self.config["skip_frames"])\n                last_valid = time.monotonic()\n                while not self.stop_event.is_set():\n                    try:\n                        chunk = serial.read(32768, timeout=2)\n                    except TypeError:\n                        chunk = serial.read(32768, 2)\n                    if chunk:\n                        before = parser.output_frames\n                        frame = parser.feed(bytes(chunk))\n                        if frame is not None:\n                            if before == 0:\n                                LOG.info("THERMAL first frame received; temperature_valid=%s", frame["valid_temperature"])\n                            last_valid = time.monotonic()\n                            with self.lock:\n                                self.latest = frame\n                                self.status = "THERMAL OK" if frame["valid_temperature"] else "UNCALIBRATED"\n                                self.total_frames += parser.output_frames - before\n                    else:\n                        self.stop_event.wait(0.001)\n                    if time.monotonic() - last_valid > self.config["thermal_retry_seconds"]:\n                        raise RuntimeError("No telemetry30 frames; check module firmware/wiring")\n            except Exception as exc:\n                LOG.warning("Thermal input: %s", exc)\n                self._status("THERMAL OFFLINE / CHECK FIRMWARE", clear=True)\n            finally:\n                if serial is not None:\n                    try:\n                        serial.close()\n                    except Exception:\n                        LOG.exception("UART close failed")\n            self.stop_event.wait(1.0)\n\n    def close(self):\n        self.stop_event.set()\n        if self.thread.is_alive():\n            self.thread.join(timeout=3.0)\n        if self.thread.is_alive():\n            LOG.warning("UART thread did not exit within 3 seconds")\n', thermal_api)
sound_api = {"__name__": "sound_adapter"}
exec('"""MaixCAM2 + MA-USB8 UART sound-direction display.\n\nOpen this single file in MaixVision and run it. The core classes are importable\nwithout MaixPy, so another application can call SoundDirectionSensor.feed_bytes()\nor poll() and read snapshot(). No camera, YOLO, thermal module or audio input.\n"""\n\nimport json\nimport math\nimport os\nimport statistics\nimport time\n\n\n# Standalone wiring: MA-USB8 TX0 -> MaixCAM2 A31 (UART1 RX),\n# Configuration control also needs MaixCAM2 A30 (UART1 TX) -> MA-USB8 RX0.\n# Common GND and independent power are required. No motor commands are sent.\n# UART1 leaves Thermal160 UART2 and MaixVision communication UART4 alone.\nUART_PORT = "/dev/ttyS1"\nUART_BAUD = 2_000_000\nROTATION_DEG = 0  # Clockwise screen correction: 0/90/180/270.\nMIRROR_X = False\nDISPLAY_INTERVAL_S = 0.08\nLOG_INTERVAL_S = 1.0\nENABLE_DISPLAY = True  # False: UART diagnostics in MaixVision console only.\nCONFIG_ENABLED = True  # False: retain the original two-wire, read-only test.\nCONFIG_PATH = "/root/micarray_direction/config.json"\nCONFIG_CHECK_INTERVAL_S = 1.0\nCOMMAND_INTERVAL_S = 0.03\nDEFAULT_CONFIG = {"detection_threshold": 650, "led_enabled": True}\nSTALE_AFTER_S = 1.0\nCENTER_DEAD_ZONE = 0.14  # About one 16x16 cell; do not invent a bearing near center.\n\nHEADER = b"\\xff" * 16\nFRAME_SIZE = 272\nMAX_BUFFER = FRAME_SIZE * 64\nPIN_MAP = {\n    "/dev/ttyS1": (("A30", "UART1_TX"), ("A31", "UART1_RX")),\n    "/dev/ttyS2": (("B0", "UART2_TX"), ("B1", "UART2_RX")),\n    "/dev/ttyS4": (("A21", "UART4_TX"), ("A22", "UART4_RX")),\n}\n\n\ndef validate_board_config(value):\n    if not isinstance(value, dict) or set(value) != set(DEFAULT_CONFIG):\n        raise ValueError("Config needs exactly detection_threshold and led_enabled")\n    threshold = value["detection_threshold"]\n    if type(threshold) is not int or not 0 <= threshold <= 2000 or threshold % 50:\n        raise ValueError("detection_threshold must be an integer 0..2000 in steps of 50")\n    if type(value["led_enabled"]) is not bool:\n        raise ValueError("led_enabled must be JSON true or false")\n    return dict(value)\n\n\ndef save_board_config(path, value):\n    """Atomically save on the CAMERA filesystem, not in MA-USB8 flash."""\n    value = validate_board_config(value)\n    parent = os.path.dirname(path)\n    if parent:\n        os.makedirs(parent, exist_ok=True)\n    temp = path + ".tmp"\n    try:\n        with open(temp, "w", encoding="utf-8") as stream:\n            stream.write(json.dumps(value, indent=2) + "\\n")\n            stream.flush()\n            os.fsync(stream.fileno())\n        os.replace(temp, path)\n    finally:\n        if os.path.exists(temp):\n            os.remove(temp)\n\n\ndef load_board_config(path, create=False):\n    try:\n        with open(path, "r", encoding="utf-8-sig") as stream:\n            text = stream.read(4097)\n    except FileNotFoundError:\n        if not create:\n            raise\n        save_board_config(path, DEFAULT_CONFIG)\n        return dict(DEFAULT_CONFIG)\n    if len(text) > 4096:\n        raise ValueError("Config is larger than 4096 characters")\n    return validate_board_config(json.loads(text))\n\n\nclass BoardConfigController:\n    """Paced documented t/T/e/E requests; no ACK, readback or flash command.\n\n    An unknown threshold is driven to the documented lower bound by 40 t\'s,\n    then raised in steps of 50. Correct execution depends on firmware clamping\n    and every command arriving; the requested value is never called readback.\n    """\n\n    def __init__(self, serial):\n        self.serial = serial\n        self.desired = None\n        self.pending = []\n        self.last_send = None\n        self.sent_commands = 0\n        self.status = "NOT_REQUESTED"\n        self.error = None\n\n    def submit(self, value):\n        value = validate_board_config(value)\n        if value == self.desired:\n            return False\n        led_only = (self.status == "SENT_UNCONFIRMED" and self.desired is not None\n                    and value["detection_threshold"] == self.desired["detection_threshold"])\n        self.pending = ([] if led_only else [b"t"] * 40\n                        + [b"T"] * (value["detection_threshold"] // 50))\n        self.pending.append(b"E" if value["led_enabled"] else b"e")\n        self.desired = value\n        self.status = "PENDING"\n        self.error = None\n        print("MICARRAY_CONFIG_REQUEST " + json.dumps(self.snapshot()), flush=True)\n        return True\n\n    def cancel(self, exc):\n        self.pending = []\n        self.desired = None\n        self.status = "CONFIG_ERROR"\n        self.error = str(exc)\n\n    def tick(self, now):\n        if not self.pending or (self.last_send is not None\n                                and now - self.last_send < COMMAND_INTERVAL_S):\n            return\n        command = self.pending[0]\n        try:\n            written = self.serial.write(command)\n            if written != len(command):\n                raise IOError("UART short write: %r" % written)\n        except Exception as exc:\n            self.pending = []\n            self.status = "WRITE_FAILED"\n            self.error = str(exc)\n            print("MICARRAY_CONFIG_WRITE_FAILED", repr(exc), flush=True)\n            return\n        del self.pending[0]\n        self.last_send = now\n        self.sent_commands += 1\n        if not self.pending:\n            self.status = "SENT_UNCONFIRMED"\n            print("MICARRAY_CONFIG_SENT_UNCONFIRMED " + json.dumps(self.snapshot()), flush=True)\n\n    def snapshot(self):\n        return {"requested": self.desired, "status": self.status,\n                "pending_commands": len(self.pending), "sent_commands": self.sent_commands,\n                "error": self.error, "execution_confirmed": False,\n                "driver_flash_save_supported": False}\n\n\nclass ConfigFileWatcher:\n    """Reload valid JSON changes without repeatedly shifting the threshold."""\n\n    def __init__(self, path, controller):\n        self.path = path\n        self.controller = controller\n        self.last_check = None\n        self.last_error = None\n\n    def poll(self, now, create=False):\n        if self.last_check is not None and now - self.last_check < CONFIG_CHECK_INTERVAL_S:\n            return\n        self.last_check = now\n        try:\n            value = load_board_config(self.path, create=create)\n        except Exception as exc:\n            self.controller.cancel(exc)\n            message = repr(exc)\n            if message != self.last_error:\n                print("MICARRAY_CONFIG_FILE_ERROR path=%s %s" % (self.path, message), flush=True)\n            self.last_error = message\n            return\n        self.last_error = None\n        self.controller.submit(value)\n\n\nclass HotmapFrameParser:\n    """Incremental 16xFF + 256-byte parser; confirms the following header."""\n\n    def __init__(self):\n        self.buffer = bytearray()\n        self.frames = 0\n        self.discarded_bytes = 0\n\n    def feed(self, chunk):\n        if not chunk:\n            return []\n        self.buffer.extend(chunk)\n        if len(self.buffer) > MAX_BUFFER:\n            drop = len(self.buffer) - MAX_BUFFER\n            del self.buffer[:drop]\n            self.discarded_bytes += drop\n        output = []\n        while True:\n            start = self.buffer.find(HEADER)\n            if start < 0:\n                drop = max(0, len(self.buffer) - len(HEADER) + 1)\n                del self.buffer[:drop]\n                self.discarded_bytes += drop\n                break\n            if start:\n                del self.buffer[:start]\n                self.discarded_bytes += start\n            if len(self.buffer) < FRAME_SIZE + len(HEADER):\n                break\n            if self.buffer[FRAME_SIZE:FRAME_SIZE + len(HEADER)] != HEADER:\n                del self.buffer[0]\n                self.discarded_bytes += 1\n                continue\n            output.append(bytes(self.buffer[16:FRAME_SIZE]))\n            del self.buffer[:FRAME_SIZE]\n            self.frames += 1\n        return output\n\n\ndef orient_xy(x, y, rotation=0, mirror=False):\n    """Transform x-right/y-up heatmap coordinates to mounted orientation."""\n    if rotation not in (0, 90, 180, 270):\n        raise ValueError("rotation must be 0/90/180/270")\n    if rotation == 90:\n        x, y = y, -x\n    elif rotation == 180:\n        x, y = -x, -y\n    elif rotation == 270:\n        x, y = -y, x\n    return (-x if mirror else x), y\n\n\nclass SoundDirectionEstimator:\n    """Estimate the dominant region in a hotmap; angle is NOT calibrated DOA."""\n\n    def __init__(self, smoothing_s=0.18, stale_after_s=STALE_AFTER_S):\n        self.smoothing_s = smoothing_s\n        self.stale_after_s = stale_after_s\n        self.last_time = None\n        self.center = None\n        self.last = {"valid": False, "reason": "no_data", "quality": 0.0}\n\n    def update(self, payload, now=None):\n        if len(payload) != 256:\n            raise ValueError("hotmap payload must be 256 bytes")\n        now = time.monotonic() if now is None else float(now)\n        if self.last_time is not None and now < self.last_time:\n            raise ValueError("timestamp moved backward")\n        gap = 2.0 if self.last_time is None else now - self.last_time\n        self.last_time = now\n        values = list(payload)\n        background = statistics.median(values)\n        sigma = 1.4826 * statistics.median(abs(v - background) for v in values)\n        contrast = max(values) - background\n        threshold = max(6.0, 3.0 * sigma, 0.25 * contrast)\n        hot = {i for i, value in enumerate(values) if value > background + threshold}\n        components = []\n        while hot:\n            seed = hot.pop()\n            stack = [seed]\n            cells = [seed]\n            while stack:\n                index = stack.pop()\n                row, col = divmod(index, 16)\n                for rr in range(max(0, row - 1), min(16, row + 2)):\n                    for cc in range(max(0, col - 1), min(16, col + 2)):\n                        neighbour = rr * 16 + cc\n                        if neighbour in hot:\n                            hot.remove(neighbour)\n                            stack.append(neighbour)\n                            cells.append(neighbour)\n            if len(cells) >= 3:\n                weights = [(values[i] - background - threshold) ** 1.5 for i in cells]\n                components.append((sum(weights), cells, weights))\n        if not components:\n            self.center = None\n            self.last = {"valid": False, "reason": "low_contrast_or_noise", "quality": 0.0,\n                         "background": background, "contrast": contrast, "component_count": 0}\n            return self.snapshot(now)\n        components.sort(key=lambda item: item[0], reverse=True)\n        mass, cells, weights = components[0]\n        dominance = mass / sum(item[0] for item in components)\n        if dominance < 0.60:\n            self.center = None\n            self.last = {"valid": False, "reason": "ambiguous_multiple_sources", "quality": 0.0,\n                         "background": background, "contrast": contrast,\n                         "component_count": len(components), "dominance": dominance}\n            return self.snapshot(now)\n        x = sum(((i % 16) - 7.5) / 7.5 * weight for i, weight in zip(cells, weights)) / mass\n        y = sum((7.5 - (i // 16)) / 7.5 * weight for i, weight in zip(cells, weights)) / mass\n        if (self.center is None or gap >= self.stale_after_s or self.smoothing_s <= 0\n                or math.hypot(x - self.center[0], y - self.center[1]) > 0.5):\n            self.center = (x, y)\n        else:\n            factor = -math.expm1(-gap / self.smoothing_s)\n            self.center = (self.center[0] + (x - self.center[0]) * factor,\n                           self.center[1] + (y - self.center[1]) * factor)\n        rows = [i // 16 for i in cells]\n        cols = [i % 16 for i in cells]\n        self.last = {\n            "valid": True, "reason": "ok", "vector_xy": self.center,\n            "centroid_vector_xy": (x, y),\n            "raw_bbox_rc": (min(rows), min(cols), max(rows) + 1, max(cols) + 1),\n            "quality": dominance * min(1.0, contrast / max(12.0, 6.0 * sigma)),\n            "dominance": dominance, "region_cells": len(cells),\n            "background": background, "contrast": contrast, "component_count": len(components),\n        }\n        return self.snapshot(now)\n\n    def snapshot(self, now=None, rotation=0, mirror=False):\n        now = time.monotonic() if now is None else float(now)\n        result = dict(self.last)\n        age = None if self.last_time is None else max(0.0, now - self.last_time)\n        result["age_seconds"] = age\n        result["coordinate_system"] = "normalized_heatmap_x_right_y_up"\n        result["rotation"] = rotation\n        result["mirror"] = bool(mirror)\n        if age is not None and age >= self.stale_after_s:\n            result["valid"] = False\n            result["reason"] = "stale"\n        if not result["valid"]:\n            for key in ("centroid_vector_xy", "raw_bbox_rc", "region_cells"):\n                result.pop(key, None)\n            result["vector_xy"] = None\n            result["unit_direction_xy"] = None\n            result["image_bearing_deg"] = None\n            result["quality"] = 0.0\n            return result\n        x, y = orient_xy(*result["vector_xy"], rotation, mirror)\n        cx, cy = orient_xy(*result["centroid_vector_xy"], rotation, mirror)\n        result["vector_xy"] = [x, y]\n        result["centroid_vector_xy"] = [cx, cy]\n        length = math.hypot(x, y)\n        result["unit_direction_xy"] = [x / length, y / length] if length >= CENTER_DEAD_ZONE else None\n        result["image_bearing_deg"] = (math.degrees(math.atan2(x, y)) % 360\n                                        if length >= CENTER_DEAD_ZONE else None)\n        return result\n\n\nclass SoundDirectionSensor:\n    """Reusable stream API: feed_bytes(), poll(), snapshot(), close().\n\n    Instantiate without a serial port for integration with an external reader.\n    open_uart() is intentionally explicit, so importing never claims a port.\n    """\n\n    def __init__(self, rotation=0, mirror=False, stale_after_s=STALE_AFTER_S):\n        orient_xy(0, 0, rotation, mirror)\n        self.rotation = rotation\n        self.mirror = mirror\n        self.parser = HotmapFrameParser()\n        self.estimator = SoundDirectionEstimator(stale_after_s=stale_after_s)\n        self.serial = None\n        self.last_payload = None\n        self.total_frames = 0\n        self.received_bytes = 0\n        self.last_byte_time = None\n        self.first_rx_hex = ""\n        self.last_rx_hex = ""\n        self.byte_rate = 0.0\n        self.frame_rate = 0.0\n        self._rate_time = None\n        self._rate_bytes = 0\n        self._rate_frames = 0\n        self.board_config = None\n\n    def feed_bytes(self, data, now=None):\n        now = time.monotonic() if now is None else float(now)\n        if data:\n            self.received_bytes += len(data)\n            self.last_byte_time = now\n            self.last_rx_hex = bytes(data[:32]).hex(" ")\n            if not self.first_rx_hex:\n                self.first_rx_hex = self.last_rx_hex\n        frames = self.parser.feed(data)\n        for payload in frames:\n            self.last_payload = payload\n            self.estimator.update(payload, now)\n            self.total_frames += 1\n        return len(frames)\n\n    def open_uart(self, port=UART_PORT, baudrate=UART_BAUD, configure_pins=True):\n        if self.serial is not None:\n            raise RuntimeError("UART already open")\n        from maix import err, pinmap\n        from maix.peripheral import uart\n        if configure_pins:\n            if port not in PIN_MAP:\n                raise ValueError("Unknown pin mapping; set configure_pins=False only after checking hardware")\n            for pin, function in PIN_MAP[port]:\n                err.check_raise(pinmap.set_pin_function(pin, function), "pinmap " + pin)\n        self.serial = uart.UART(port=port, baudrate=baudrate)\n        return self.serial\n\n    def poll(self, read_size=4096, timeout_ms=2):\n        if self.serial is None:\n            raise RuntimeError("UART not open")\n        try:\n            data = self.serial.read(read_size, timeout=timeout_ms)\n        except TypeError:\n            data = self.serial.read(read_size, timeout_ms)\n        return self.feed_bytes(bytes(data)) if data else 0\n\n    def snapshot(self, now=None):\n        now = time.monotonic() if now is None else float(now)\n        result = self.estimator.snapshot(now, self.rotation, self.mirror)\n        result["frames"] = self.total_frames\n        result["discarded_bytes"] = self.parser.discarded_bytes\n        result["received_bytes"] = self.received_bytes\n        result["buffered_bytes"] = len(self.parser.buffer)\n        result["byte_age_seconds"] = (None if self.last_byte_time is None\n                                       else max(0.0, now - self.last_byte_time))\n        result["bytes_per_second"] = self.byte_rate\n        result["frames_per_second"] = self.frame_rate\n        if self.last_byte_time is None:\n            status = "WAIT_BYTES"\n        elif result["byte_age_seconds"] >= self.estimator.stale_after_s:\n            status = "UART_SILENT"\n        elif result["age_seconds"] is None or result["reason"] == "stale":\n            status = "BYTES_NO_FRAME"\n        elif not result["valid"]:\n            status = "FRAME_NO_DIRECTION"\n        elif result["unit_direction_xy"] is None:\n            status = "SOURCE_CENTER"\n        else:\n            status = "DIRECTION_OK"\n        result["status"] = status\n        result["map_direction"] = map_direction(result["image_bearing_deg"])\n        return result\n\n    def sample_rates(self, now=None):\n        """Update rates over actual elapsed time, not over display refreshes."""\n        now = time.monotonic() if now is None else float(now)\n        if self._rate_time is not None and now > self._rate_time:\n            elapsed = now - self._rate_time\n            self.byte_rate = (self.received_bytes - self._rate_bytes) / elapsed\n            self.frame_rate = (self.total_frames - self._rate_frames) / elapsed\n        self._rate_time = now\n        self._rate_bytes = self.received_bytes\n        self._rate_frames = self.total_frames\n\n    def diagnostic_record(self, now=None):\n        result = self.snapshot(now)\n        result["first_rx_hex"] = self.first_rx_hex\n        result["last_rx_hex"] = self.last_rx_hex\n        result["hint"] = STATUS_HINTS[result["status"]]\n        if self.board_config is not None:\n            result["board_control"] = self.board_config.snapshot()\n        return result\n\n    def close(self):\n        serial, self.serial = self.serial, None\n        if serial is not None:\n            serial.close()\n\n\nSTATUS_HINTS = {\n    "WAIT_BYTES": "Check MA-USB8 power, TX0 -> A31, common GND and UART1 ownership",\n    "UART_SILENT": "UART bytes stopped; check power, cable and slip-ring contact",\n    "BYTES_NO_FRAME": "Waiting for binary sync; if persistent, check 2M baud, output mode and HEX",\n    "FRAME_NO_DIRECTION": "Fresh heatmaps received; try one steady sound source and inspect reason",\n    "SOURCE_CENTER": "Source near map center; no unique map bearing",\n    "DIRECTION_OK": "Fresh heatmap with dominant source; map bearing is not calibrated physical DOA",\n}\n\n\ndef map_direction(angle):\n    if angle is None:\n        return "--"\n    names = ("UP", "UP-RIGHT", "RIGHT", "DOWN-RIGHT", "DOWN", "DOWN-LEFT", "LEFT", "UP-LEFT")\n    return names[int((angle + 22.5) // 45) % 8]\n\n\ndef display_hotmap(img, payload, left, top, size, image, rotation=0, mirror=False):\n    """Draw a raw grid; display enhancement never affects the direction estimate."""\n    cell = max(1, size // 16)\n    background = statistics.median(payload) if payload else 0\n    peak = max(payload) if payload else 1\n    span = max(24, peak - background)\n    oriented = [0] * 256\n    if payload:\n        for row in range(16):\n            for col in range(16):\n                x, y = orient_xy((col - 7.5) / 7.5, (7.5 - row) / 7.5, rotation, mirror)\n                new_col = round(7.5 + 7.5 * x)\n                new_row = round(7.5 - 7.5 * y)\n                oriented[new_row * 16 + new_col] = payload[row * 16 + col]\n    for row in range(16):\n        for col in range(16):\n            value = oriented[row * 16 + col]\n            t = min(1.0, max(0.0, (value - background) / span))\n            color = image.Color.from_rgb(int(15 + 230 * t), int(30 + 150 * t), int(50 + 25 * (1 - t)))\n            img.draw_rect(left + col * cell, top + row * cell, cell - 1, cell - 1, color, -1)\n    img.draw_rect(left - 1, top - 1, cell * 16 + 2, cell * 16 + 2,\n                  image.Color.from_rgb(110, 125, 145), 2)\n    return cell * 16\n\n\ndef draw_screen(disp, sensor):\n    from maix import image\n    width, height = disp.width(), disp.height()\n    img = image.Image(width, height, image.Format.FMT_RGB888,\n                      bg=image.Color.from_rgb(8, 14, 24))\n    white = image.Color.from_rgb(235, 242, 250)\n    cyan = image.Color.from_rgb(50, 225, 220)\n    yellow = image.Color.from_rgb(255, 204, 75)\n    muted = image.Color.from_rgb(145, 160, 180)\n    img.draw_string(12, 10, "MICARRAY MODULE TEST", white, scale=1.5)\n    mode = "TX=A30 RX=A31" if CONFIG_ENABLED else "RX=A31 READ ONLY"\n    img.draw_string(12, 42, "UART1 %s | 2000000 8N1" % mode, muted)\n    result = sensor.snapshot()\n    img.draw_string(12, 68, result["status"], cyan if result["status"] == "DIRECTION_OK" else yellow)\n    size = max(64, min(width - 72, height - 280, 256))\n    size = size // 16 * 16\n    left, top = (width - size) // 2, 114\n    fresh = (result["age_seconds"] is not None\n             and result["age_seconds"] < sensor.estimator.stale_after_s)\n    payload = sensor.last_payload if fresh else None\n    drawn = display_hotmap(img, payload, left, top, size, image,\n                           sensor.rotation, sensor.mirror)\n    cx, cy = left + drawn // 2, top + drawn // 2\n    img.draw_string(cx - 8, top - 20, "UP", muted)\n    img.draw_string(left - 20, cy - 8, "L", muted)\n    img.draw_string(left + drawn + 8, cy - 8, "R", muted)\n    img.draw_circle(cx, cy, 5, white, -1)\n    if result["valid"] and result["unit_direction_xy"] is not None:\n        x, y = result["unit_direction_xy"]\n        length = int(drawn * 0.40)\n        end_x, end_y = int(cx + x * length), int(cy - y * length)\n        img.draw_line(cx, cy, end_x, end_y, cyan, 5)\n        wing = max(10, drawn // 20)\n        img.draw_line(end_x, end_y, int(end_x - x * wing + y * wing * .6),\n                      int(end_y + y * wing + x * wing * .6), cyan, 4)\n        img.draw_line(end_x, end_y, int(end_x - x * wing - y * wing * .6),\n                      int(end_y + y * wing - x * wing * .6), cyan, 4)\n        line = "MAP %03.0f DEG | %s" % (result["image_bearing_deg"], result["map_direction"])\n        quality = "QUALITY %.2f | PEAK AREA %d" % (result["quality"], result["region_cells"])\n    elif result["valid"]:\n        line, quality = "SOURCE NEAR CENTER", "NO UNIQUE BEARING"\n    else:\n        line = "NO DIRECTION"\n        quality = "REASON: " + result["reason"].upper()\n    y0 = top + drawn + 12\n    img.draw_string(12, y0, line, yellow, scale=1.2)\n    img.draw_string(12, y0 + 26, quality, muted)\n    age = "--" if result["age_seconds"] is None else "%.1f" % result["age_seconds"]\n    img.draw_string(12, y0 + 52, "RX %d B | %.0f B/s | %.1f FPS" %\n                    (result["received_bytes"], result["bytes_per_second"], result["frames_per_second"]), muted)\n    img.draw_string(12, y0 + 76, "FRAMES %d | AGE %ss | DISCARD %d B" %\n                    (result["frames"], age, result["discarded_bytes"]), muted)\n    if sensor.board_config is not None:\n        config = sensor.board_config.snapshot()\n        requested = config["requested"]\n        settings = ("THR %d LED %s" % (requested["detection_threshold"],\n                                      "ON" if requested["led_enabled"] else "OFF")\n                    if requested else "CONFIG INVALID")\n        img.draw_string(12, y0 + 100, settings + " | " + config["status"], muted)\n    else:\n        img.draw_string(12, y0 + 100, "Map direction only; physical angle NOT calibrated", muted)\n    disp.show(img)\n\n\ndef main():\n    from maix import app, display, sys\n    if str(sys.device_id()).lower() != "maixcam2":\n        raise RuntimeError("This UART pin configuration is for MaixCAM2")\n    sensor = SoundDirectionSensor(rotation=ROTATION_DEG, mirror=MIRROR_X)\n    disp = None\n    try:\n        print("MICARRAY_START port=%s baud=%d read_only=%s" % (UART_PORT, UART_BAUD, not CONFIG_ENABLED), flush=True)\n        print("WIRING: MA-USB8 TX0 -> A31; GND -> GND; independent power", flush=True)\n        if CONFIG_ENABLED:\n            print("CONFIG WIRING: MaixCAM2 A30 -> MA-USB8 RX0 is REQUIRED", flush=True)\n        sensor.open_uart(UART_PORT, UART_BAUD)\n        print("MICARRAY_UART_READY (port opened; no received signal confirmed yet)", flush=True)\n        watcher = None\n        if CONFIG_ENABLED:\n            print("MICARRAY_CONFIG_PATH", CONFIG_PATH, flush=True)\n            print("CONFIG saved on camera; requests have no ACK; driver flash persistence unsupported", flush=True)\n            sensor.board_config = BoardConfigController(sensor.serial)\n            watcher = ConfigFileWatcher(CONFIG_PATH, sensor.board_config)\n            watcher.poll(time.monotonic(), create=True)\n        if ENABLE_DISPLAY:\n            try:\n                disp = display.Display()\n            except Exception as exc:\n                print("MICARRAY_DISPLAY_ERROR console diagnostics continue:", repr(exc), flush=True)\n        last_draw = 0.0\n        last_log = time.monotonic()\n        sensor.sample_rates(last_log)\n        previous_status = None\n        while not app.need_exit():\n            sensor.poll()\n            now = time.monotonic()\n            if watcher is not None:\n                watcher.poll(now)\n                sensor.board_config.tick(now)\n            status = sensor.snapshot(now)["status"]\n            if now - last_log >= LOG_INTERVAL_S:\n                sensor.sample_rates(now)\n                print("MICARRAY_DIAG " + json.dumps(sensor.diagnostic_record(now)), flush=True)\n                last_log = now\n            if status != previous_status:\n                print("MICARRAY_STATE %s: %s" % (status, STATUS_HINTS[status]), flush=True)\n                previous_status = status\n            if disp is not None and now - last_draw >= DISPLAY_INTERVAL_S:\n                try:\n                    draw_screen(disp, sensor)\n                except Exception as exc:\n                    print("MICARRAY_DRAW_ERROR console diagnostics continue:", repr(exc), flush=True)\n                    try:\n                        if hasattr(disp, "close"):\n                            disp.close()\n                    except Exception as close_exc:\n                        print("MICARRAY_CLOSE_ERROR", repr(close_exc), flush=True)\n                    finally:\n                        disp = None\n                last_draw = now\n    except Exception as exc:\n        print("MICARRAY_ERROR", repr(exc), flush=True)\n        raise\n    finally:\n        for resource in (sensor, disp):\n            if resource is not None and hasattr(resource, "close"):\n                try:\n                    resource.close()\n                except Exception as exc:\n                    print("MICARRAY_CLOSE_ERROR", repr(exc), flush=True)\n        print("MICARRAY_STOP", flush=True)\n\n\n', sound_api)
"""Evidence fusion, not a calibrated probability or a drone classifier."""
import json
import math
import os
from collections import deque

DEFAULT = {
    "weights": {"vision": 0.70, "sound": 0.15, "thermal": 0.15},
    "max_age": {"vision": 0.35, "sound": 0.60, "thermal": 0.60},
    "pair_delta": 0.25, "association_radius": 0.12,
    "visual_confirm": 0.80, "confirm_hits": 3, "hold_seconds": 0.8,
    "model_path": "/root/models/yolo11n.mud", "model_type": "YOLO11",
    "drone_labels": ["drone", "uav", "quadcopter"],
    "thermal_to_rgb": None, "sound_to_rgb": None,
    "calibration_resolution": None, "sound_calibration_bounds": None,
    "camera_fps": 30,
}


def validate(config):
    w = config["weights"]
    if set(w) != {"vision", "sound", "thermal"}:
        raise ValueError("Exactly three sensor weights required")
    if not all(isinstance(v, (float, int)) and math.isfinite(v) and 0 <= v <= 1 for v in w.values()):
        raise ValueError("Weights must be finite in [0,1]")
    if abs(sum(w.values()) - 1) > 1e-6 or w["vision"] < .60:
        raise ValueError("Weights must sum to 1; vision must be >= 0.60")
    for v in config["max_age"].values():
        if not math.isfinite(v) or v <= 0:
            raise ValueError("Invalid sensor timeout")
    if not .5 <= config["visual_confirm"] <= 1 or not 2 <= config["confirm_hits"] <= 10:
        raise ValueError("Invalid confirmation settings")
    for name in ("association_radius", "pair_delta", "hold_seconds"):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError("Invalid " + name)
    return config


def load_config(path):
    config = json.loads(json.dumps(DEFAULT))
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            config.update(json.load(f))
    return validate(config)


def save_config(path, config):
    validate(config)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    temp = path + ".tmp"
    with open(temp, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)


def observation(kind, x, y, score, timestamp, token, label="", calibrated=True):
    return dict(kind=kind, x=float(x), y=float(y), score=float(score),
                timestamp=float(timestamp), token=str(token), label=label, calibrated=calibrated)


class Fusion:
    """Bounded tracks, one-to-one association; every hit requires a new source frame."""
    def __init__(self, config):
        self.config = validate(config)
        self.tracks = []
        self.serial = 0
        self.seen = set()
        self.seen_order = deque()

    def update(self, observations, now):
        c = self.config
        self.tracks = [t for t in self.tracks if now - t["last"] <= c["hold_seconds"]]
        used = set()
        rejected = []
        for o in sorted(observations, key=lambda a: a.get("kind") != "vision"):
            kind = o.get("kind")
            if kind not in c["weights"]:
                continue
            if (not all(math.isfinite(o[k]) for k in ("x", "y", "score", "timestamp"))
                    or not 0 <= o["score"] <= 1 or not 0 <= o["x"] <= 1 or not 0 <= o["y"] <= 1
                    or not 0 <= now - o["timestamp"] <= c["max_age"][kind]
                    or not o.get("calibrated", False)):
                rejected.append(kind + ": stale/invalid/uncalibrated")
                continue
            key = (kind, o["token"])
            if key in self.seen:
                continue
            self.seen.add(key)
            self.seen_order.append(key)
            while len(self.seen_order) > 4096:
                self.seen.discard(self.seen_order.popleft())
            choices = []
            for t in self.tracks:
                if (t["id"], kind) in used:
                    continue
                distance = math.hypot(t["x"] - o["x"], t["y"] - o["y"])
                time_gate = c["hold_seconds"] if kind in t["evidence"] else c["pair_delta"]
                if distance <= c["association_radius"] and abs(t["last"] - o["timestamp"]) <= time_gate:
                    choices.append((distance, t["id"], t))
            choices.sort(key=lambda item: item[0])
            # Ambiguous candidates never add auxiliary evidence to either target.
            if len(choices) > 1 and choices[1][0] - choices[0][0] < .025:
                rejected.append(kind + ": ambiguous association")
                continue
            if choices:
                t = choices[0][2]
            else:
                self.serial += 1
                t = dict(id=self.serial, x=o["x"], y=o["y"], last=o["timestamp"],
                         evidence={}, visual_hits=deque(maxlen=5), confirmed_at=None)
                self.tracks.append(t)
            used.add((t["id"], kind))
            previous = t["evidence"].get(kind)
            if previous and o["timestamp"] <= previous["timestamp"]:
                continue
            t["evidence"][kind] = o
            t["last"] = max(t["last"], o["timestamp"])
            # Auxiliary directions cannot drag a fresh visual position away.
            v = t["evidence"].get("vision")
            if kind == "vision" or not v or now - v["timestamp"] > c["max_age"]["vision"]:
                t["x"], t["y"] = o["x"], o["y"]
            if kind == "vision":
                good = o["label"].lower() in c["drone_labels"] and o["score"] >= c["visual_confirm"]
                t["visual_hits"].append((o["timestamp"], good))
        result = []
        for t in self.tracks:
            values = {}
            for k in c["weights"]:
                o = t["evidence"].get(k)
                valid = o and 0 <= now - o["timestamp"] <= c["max_age"][k]
                # An auxiliary observation must also be synchronized to the latest evidence.
                valid = valid and abs(t["last"] - o["timestamp"]) <= c["pair_delta"]
                if k == "vision" and o and o["label"].lower() not in c["drone_labels"]:
                    valid = False
                values[k] = o["score"] if valid else 0.0
            hits = sum(good for ts, good in t["visual_hits"] if 0 <= now - ts <= .7)
            confirmed = values["vision"] >= c["visual_confirm"] and hits >= c["confirm_hits"]
            if confirmed:
                t["confirmed_at"] = now
            held = t["confirmed_at"] is not None and now - t["confirmed_at"] <= c["hold_seconds"]
            state = "VISUAL_CONFIRMED" if confirmed else "HOLD_UNCONFIRMED" if held else "CANDIDATE"
            contributions = {k: values[k] * c["weights"][k] for k in values}
            result.append(dict(id=t["id"], x=t["x"], y=t["y"], state=state,
                               evidence=values, contributions=contributions,
                               score=sum(contributions.values()), hits=hits))
        return sorted(result, key=lambda t: (t["state"] == "VISUAL_CONFIRMED", t["score"]), reverse=True)[:24], rejected


def thermal_candidates(pixels):
    """Multi-scale center-surround on original 160x120 gray pixels; no temperature claim."""
    import cv2
    import numpy as np
    raw = np.asarray(pixels, dtype=np.float32)
    if raw.shape != (120, 160) or not np.isfinite(raw).all():
        raise ValueError("Expected finite original Thermal160 120x160 frame")
    response = np.zeros_like(raw)
    for size in (5, 11, 21):
        mean = cv2.boxFilter(raw, -1, (size, size))
        variance = np.maximum(cv2.boxFilter(raw * raw, -1, (size, size)) - mean * mean, 0)
        response = np.maximum(response, (raw - mean) / np.maximum(np.sqrt(variance), 2.0))
    mask = (response > 3.0).astype(np.uint8)
    mask[:2] = mask[-2:] = 0
    mask[:, :2] = mask[:, -2:] = 0
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    output = []
    for i in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[i])
        if area > 600 or max(w, h) / max(1, min(w, h)) > 6:
            continue
        peak = float(response[labels == i].max())
        output.append(dict(x=float(centers[i][0]), y=float(centers[i][1]), area=area,
                           box=[x, y, w, h], score=min(.95, peak / 10)))
    return sorted(output, key=lambda o: o["score"], reverse=True)[:24]

SCENARIOS = []
def render(tracks, observations, config, title, details='', background=None, thermal=None):
    canvas = np.full((720, 1100, 3), (25, 20, 15), np.uint8)

    def text(s, x, y, scale=0.55, color=(215, 220, 225)):
        cv2.putText(canvas, str(s), (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
    text('DRONE / MULTISENSOR EVIDENCE', 28, 38, 0.8, (235, 235, 235))
    text(title, 28, 67, 0.58, (80, 200, 255))
    (x0, y0, w, h) = (25, 95, 680, 430)
    if background is not None:
        canvas[y0:y0 + h, x0:x0 + w] = cv2.resize(background, (w, h))
    else:
        canvas[y0:y0 + h, x0:x0 + w] = (48, 38, 26)
        for x in range(x0, x0 + w, 68):
            cv2.line(canvas, (x, y0), (x, y0 + h), (65, 52, 38))
        for y in range(y0, y0 + h, 43):
            cv2.line(canvas, (x0, y), (x0 + w, y), (65, 52, 38))
    colors = {'vision': (100, 240, 100), 'sound': (240, 170, 70), 'thermal': (60, 140, 255)}
    for o in observations:
        (px, py) = (int(x0 + o['x'] * w), int(y0 + o['y'] * h))
        cv2.circle(canvas, (px, py), 9, colors[o['kind']], 2)
        offset = {'vision': -12, 'sound': 6, 'thermal': 24}[o['kind']]
        text(o['kind'], px + 30, py + offset, 0.4, colors[o['kind']])
    for t in tracks:
        (px, py) = (int(x0 + t['x'] * w), int(y0 + t['y'] * h))
        cv2.rectangle(canvas, (px - 24, py - 20), (px + 24, py + 20), (100, 230, 230), 2)
        text('#%d %s' % (t['id'], t['state']), max(30, px - 80), max(115, py - 30), 0.42)
    text('SENSOR WEIGHTS / no missing-channel renormalization', 25, 558, 0.5)
    for (i, kind) in enumerate(colors):
        x = 25 + 230 * i
        text('%s %.0f%%' % (kind.upper(), config['weights'][kind] * 100), x, 587, 0.5, colors[kind])
        cv2.rectangle(canvas, (x, 600), (x + int(200 * config['weights'][kind]), 613), colors[kind], -1)
    text('Evidence score is NOT accuracy or class probability.', 25, 650, 0.52)
    text(details[:115], 25, 685, 0.43)
    text('TARGET EVIDENCE', 735, 115, 0.65)
    t = tracks[0] if tracks else None
    text(t['state'] if t else 'NO TARGET', 735, 155, 0.51, (80, 220, 255))
    text('Score: %.3f' % t['score'] if t else 'Score: --', 735, 192, 0.7)
    text('New visual hits: %d' % t['hits'] if t else 'New visual hits: 0', 735, 224)
    for (i, k) in enumerate(colors):
        value = t['contributions'][k] if t else 0
        yy = 270 + i * 70
        text('%s contribution %.3f' % (k, value), 735, yy, 0.48)
        cv2.rectangle(canvas, (735, yy + 12), (735 + int(value * 320), yy + 25), colors[k], -1)
    if thermal is not None:
        thumb = cv2.applyColorMap(np.asarray(thermal, np.uint8), cv2.COLORMAP_INFERNO)
        canvas[480:660, 735:975] = cv2.resize(thumb, (240, 180))
        text('THERMAL / original 160x120', 735, 465, 0.43)
    else:
        for (i, s) in enumerate(SCENARIOS):
            text('%d  %s' % (i + 1, s), 735, 470 + i * 30, 0.43)
    return canvas
"""Bundled by build.py with existing verified-source UART adapters."""
import time
import json
import math
import numpy as np
import cv2


def project(matrix, x, y):
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape not in ((2, 3), (3, 3)) or not np.isfinite(m).all():
        raise ValueError("Invalid calibration matrix")
    p = m @ np.array([x, y, 1.0])
    if len(p) == 3:
        if abs(p[2]) < 1e-9:
            return None
        p = p[:2]/p[2]
    return tuple(float(v) for v in p) if np.isfinite(p).all() and np.all((p >= 0) & (p <= 1)) else None


def run_board(config_path="/root/drone_sensor_demo/config.json"):
    from maix import app, camera, display, image, nn
    config = load_config(config_path)
    engine = Fusion(config)
    sensor = sound_api["SoundDirectionSensor"]()
    thermal = thermal_api["ThermalReader"](dict(thermal_api["CONFIG"]))
    cam = disp = model = None
    sound_ok = False
    try:
        # Explicit generic model fallback is displayed; it cannot confirm drones.
        model = getattr(nn, config["model_type"])(model=config["model_path"], dual_buff=False)
        iw, ih = model.input_width(), model.input_height()
        labels = [str(label).lower() for label in model.labels]
        has_drone = any(label in config["drone_labels"] for label in labels)
        cam = camera.Camera(iw, ih, image.Format.FMT_RGB888, fps=config["camera_fps"])
        disp = display.Display()
        thermal.start()
        try:
            sensor.open_uart()
            sound_ok = True
        except Exception as exc:
            print("SOUND OFFLINE", repr(exc))
        calibration_ok = config["calibration_resolution"] == [iw, ih]
        seq, last_log, last_camera = 0, 0, time.monotonic()
        while not app.need_exit():
            frame = cam.read(block_ms=200)
            if frame is None:
                if time.monotonic()-last_camera > 5:
                    raise RuntimeError("Camera stalled for 5 seconds")
                continue
            captured = time.monotonic()
            last_camera = captured
            objects = model.detect(frame, conf_th=.35, iou_th=.45)
            now = time.monotonic()
            seq += 1
            obs = []
            for idx, obj in enumerate(objects):
                label = str(model.labels[obj.class_id]).lower()
                if label in config["drone_labels"]:
                    obs.append(observation("vision", (obj.x+obj.w/2)/iw, (obj.y+obj.h/2)/ih,
                                           obj.score, captured, "%d:%d" % (seq, idx), label))
            tf, thermal_status, _ = thermal.snapshot()
            candidates = []
            if tf is not None and tf["valid_temperature"] and 0 <= now-tf["received_at"] <= config["max_age"]["thermal"]:
                candidates = thermal_candidates(tf["pixels"])
                if calibration_ok and config["thermal_to_rgb"] is not None:
                    for idx, item in enumerate(candidates):
                        p = project(config["thermal_to_rgb"], item["x"], item["y"])
                        if p:
                            obs.append(observation("thermal", *p, item["score"], tf["received_at"],
                                                   "%s:%d" % (tf["received_at"], idx)))
            sound_status = "OFFLINE"
            if sound_ok:
                try:
                    sensor.poll(timeout_ms=2)
                    s = sensor.snapshot()
                    sound_status = s.get("reason", "OK")
                    bounds = config["sound_calibration_bounds"]
                    if s["valid"] and s["vector_xy"] and calibration_ok and config["sound_to_rgb"] is not None and bounds:
                        x, y = s["vector_xy"]
                        if bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3]:
                            p = project(config["sound_to_rgb"], x, y)
                            if p:
                                stamp = time.monotonic()-s["age_seconds"]
                                obs.append(observation("sound", *p, s["quality"], stamp, s["frames"]))
                except Exception as exc:
                    sound_status = "ERROR: " + str(exc)
            now = time.monotonic()
            tracks, rejected = engine.update(obs, now)
            rgb = image.image2cv(frame, ensure_bgr=False, copy=True)
            title = "LIVE / UAV model labels present" if has_drone else "LIVE / MODEL HAS NO UAV LABEL - candidates only"
            details = "T:%s (%d blobs) S:%s CAL:%s" % (thermal_status, len(candidates), sound_status,
                        "CHECK MATRICES" if calibration_ok else "UNSET - auxiliary fusion disabled")
            # Missing mappings must be visible even when resolution happens to match.
            if not config["thermal_to_rgb"] or not config["sound_to_rgb"]:
                details += " / MAPPING MISSING"
            fresh_thermal = tf is not None and 0 <= now-tf["received_at"] <= config["max_age"]["thermal"]
            canvas = render(tracks, obs, config, title, details,
                            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), tf["pixels"] if fresh_thermal else None)
            disp.show(image.cv2image(cv2.resize(canvas, (640, 480)), bgr=True, copy=True))
            if now-last_log >= 1:
                print("DRONE_FUSION " + json.dumps(dict(mode="live", weights=config["weights"],
                      drone_model_labels=has_drone, tracks=tracks, rejected=rejected,
                      thermal_candidates=candidates, sound_status=sound_status, calibration=details)))
                last_log = now
    finally:
        for resource in (sensor, thermal, cam):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    print("CLEANUP", repr(exc))
        cam = disp = model = None


if __name__ == "__main__":
    run_board()
