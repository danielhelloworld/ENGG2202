"""TN160 telemetry30 UART input. Protocol derived from Sipeed MaixPy (Apache-2.0).

See THIRD_PARTY.md. This is not the legacy 19201-byte or USB/UVC protocol.
"""
import struct
import threading


def valid_body(body):
    if len(body) != BODY_SIZE:
        return False
    vtemp, lo, hi = struct.unpack_from(">Hhh", body, PIXELS)
    if vtemp & 0xC000:
        return False
    if lo == hi == 32767:
        return True
    return -1000 <= lo <= hi <= 3200


def decode_body(body, received_at):
    vtemp, lo, hi = struct.unpack_from(">Hhh", body, PIXELS)
    return {
        "pixels": np.frombuffer(body[:PIXELS], np.uint8).reshape(THERMAL_H, THERMAL_W),
        "lo": lo, "hi": hi, "vtemp": vtemp & 0x3FFF,
        "telemetry": body[PIXELS:], "received_at": received_at,
        "valid_temperature": lo != 32767 and hi != 32767 and hi > lo,
    }


class ThermalParser:
    """Bounded incremental parser; acquire sync using two consecutive plausible frames.

    Once locked, accepts a complete frame without throwing it away just because
    noise follows it. There is no firmware CRC; false sync is still possible.
    """
    def __init__(self, skip=10):
        self.buffer = bytearray()
        self.locked = False
        self.skip = skip
        self.frames = 0
        self.output_frames = 0
        self.discarded_bytes = 0

    def feed(self, chunk, received_at=None):
        now = time.monotonic() if received_at is None else received_at
        self.buffer.extend(chunk)
        latest = None
        while True:
            idx = self.buffer.find(b"\xff")
            if idx < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                self.locked = False
                break
            if idx:
                self.discarded_bytes += idx
                del self.buffer[:idx]
                self.locked = False
            if len(self.buffer) < WIRE_SIZE:
                break
            body = bytes(self.buffer[1:WIRE_SIZE])
            if not valid_body(body):
                del self.buffer[0]
                self.discarded_bytes += 1
                self.locked = False
                continue
            if not self.locked:
                if len(self.buffer) < WIRE_SIZE * 2:
                    break
                next_body = bytes(self.buffer[WIRE_SIZE + 1:WIRE_SIZE * 2])
                if self.buffer[WIRE_SIZE] != 255 or not valid_body(next_body):
                    del self.buffer[0]
                    self.discarded_bytes += 1
                    continue
                self.locked = True
            del self.buffer[:WIRE_SIZE]
            self.frames += 1
            if self.skip > 0:
                self.skip -= 1
            else:
                self.output_frames += 1
                latest = decode_body(body, now)
            # A frame may end exactly at the read boundary; retain the sync state.
            if not self.buffer:
                break
        return latest


class ThermalReader:
    """One UART owner thread, latest-frame mailbox, finite reads and reconnection."""
    def __init__(self, config):
        self.config = config
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.status = "THERMAL STARTING"
        self.total_frames = 0
        self.reset_gpio = None
        self.thread = threading.Thread(target=self._run, name="thermal-uart", daemon=True)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.latest, self.status, self.total_frames

    def _status(self, message, clear=False):
        with self.lock:
            self.status = message
            if clear:
                self.latest = None

    def _open(self):
        from maix import gpio, pinmap, err
        from maix.peripheral import uart
        c = self.config
        if c["configure_uart_pins"]:
            for pin, function in (("B0", "UART2_TX"), ("B1", "UART2_RX")):
                err.check_raise(pinmap.set_pin_function(pin, function), "UART2 pinmap: " + pin)
        err.check_raise(pinmap.set_pin_function(c["reset_pin"], c["reset_gpio"]), "Thermal reset pinmap")
        self.reset_gpio = gpio.GPIO(c["reset_gpio"], gpio.Mode.OUT)
        idle = 1 - c["reset_active"]
        self.reset_gpio.value(idle)
        time.sleep(0.02)
        self.reset_gpio.value(c["reset_active"])
        time.sleep(0.12)
        self.reset_gpio.value(idle)
        time.sleep(0.40)
        serial = uart.UART(port=c["uart_port"], baudrate=2000000)
        try:
            serial.write(b"\x44")
        finally:
            serial.close()
        time.sleep(0.10)
        return uart.UART(port=c["uart_port"], baudrate=4000000)

    def _run(self):
        while not self.stop_event.is_set():
            serial = None
            try:
                self._status("THERMAL CONNECTING", clear=True)
                LOG.info("THERMAL opening %s / telemetry30", self.config["uart_port"])
                serial = self._open()
                LOG.info("THERMAL UART ready; waiting for valid frames")
                parser = ThermalParser(self.config["skip_frames"])
                last_valid = time.monotonic()
                while not self.stop_event.is_set():
                    try:
                        chunk = serial.read(32768, timeout=2)
                    except TypeError:
                        chunk = serial.read(32768, 2)
                    if chunk:
                        before = parser.output_frames
                        frame = parser.feed(bytes(chunk))
                        if frame is not None:
                            if before == 0:
                                LOG.info("THERMAL first frame received; temperature_valid=%s", frame["valid_temperature"])
                            last_valid = time.monotonic()
                            with self.lock:
                                self.latest = frame
                                self.status = "THERMAL OK" if frame["valid_temperature"] else "UNCALIBRATED"
                                self.total_frames += parser.output_frames - before
                    else:
                        self.stop_event.wait(0.001)
                    if time.monotonic() - last_valid > self.config["thermal_retry_seconds"]:
                        raise RuntimeError("No telemetry30 frames; check module firmware/wiring")
            except Exception as exc:
                LOG.warning("Thermal input: %s", exc)
                self._status("THERMAL OFFLINE / CHECK FIRMWARE", clear=True)
            finally:
                if serial is not None:
                    try:
                        serial.close()
                    except Exception:
                        LOG.exception("UART close failed")
            self.stop_event.wait(1.0)

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            LOG.warning("UART thread did not exit within 3 seconds")
