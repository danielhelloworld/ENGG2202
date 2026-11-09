"""Small RTSP/1.0 server: baseline RTP/JPEG over TCP (RFC 2326 + RFC 2435).

No camera binding: only receives the application's fully composed RGB images.
One pending image per encoder/client bounds latency and memory for slow clients.
"""
import re
import secrets
import socket
from urllib.parse import urlsplit


def parse_jpeg(jpeg):
    if jpeg[:2] != b"\xff\xd8" or jpeg[-2:] != b"\xff\xd9":
        raise ValueError("JPEG SOI/EOI missing")
    pos, tables, info = 2, {}, None
    while pos + 4 <= len(jpeg):
        if jpeg[pos] != 255:
            raise ValueError("Invalid JPEG marker")
        while jpeg[pos] == 255:
            pos += 1
        marker = jpeg[pos]
        size = int.from_bytes(jpeg[pos + 1:pos + 3], "big")
        if size < 2 or pos + 1 + size > len(jpeg):
            raise ValueError("Truncated JPEG segment")
        data = jpeg[pos + 3:pos + 1 + size]
        pos += 1 + size
        if marker == 0xDB:
            off = 0
            while off < len(data):
                table = data[off]
                if table >> 4 != 0 or off + 65 > len(data):
                    raise ValueError("Only 8-bit JPEG quantization supported")
                tables[table & 15] = data[off + 1:off + 65]
                off += 65
        elif marker == 0xC0:
            if len(data) != 15 or data[0] != 8 or data[5] != 3:
                raise ValueError("Expected 3-component baseline JPEG")
            height, width = struct.unpack_from(">HH", data, 1)
            sampling = data[7]
            if (sampling not in (0x21, 0x22) or data[10] != 0x11 or data[13] != 0x11 or
                    data[8] != 0 or data[11] != 1 or data[14] != 1):
                raise ValueError("RTP/JPEG requires YUV 4:2:2 or 4:2:0 with Q tables 0/1")
            if width % 8 or height % 8 or not 8 <= width <= 2040 or not 8 <= height <= 2040:
                raise ValueError("Stream dimensions must be multiples of 8 and <=2040")
            info = {"width": width, "height": height, "type": 1 if sampling == 0x22 else 0}
        elif marker in (0xC1, 0xC2, 0xDD):
            raise ValueError("Progressive/extended/restart JPEG is unsupported")
        elif marker == 0xDA:
            if info is None or 0 not in tables or 1 not in tables:
                raise ValueError("Missing JPEG frame header or quantization tables")
            if len(data) != 10 or data[0] != 3 or data[2] != 0 or data[4] != 0x11 or data[6] != 0x11:
                raise ValueError("Expected standard interleaved JPEG scan")
            info["tables"] = tables[0] + tables[1]
            info["scan"] = jpeg[pos:-2]
            if not info["scan"] or len(info["scan"]) >= 1 << 24:
                raise ValueError("Invalid JPEG scan length")
            return info
    raise ValueError("No JPEG scan")


def jpeg_rtp_packets(frame, timestamp, sequence, ssrc, mtu=1400):
    offset = 0
    scan = frame["scan"]
    while offset < len(scan):
        quant = struct.pack(">BBH", 0, 0, len(frame["tables"])) + frame["tables"] if offset == 0 else b""
        count = min(mtu - 20 - len(quant), len(scan) - offset)
        if count <= 0:
            raise ValueError("RTP MTU too small")
        last = offset + count == len(scan)
        rtp = struct.pack(">BBHII", 0x80, 26 | (0x80 if last else 0), sequence & 65535, timestamp & 0xffffffff, ssrc)
        header = b"\0" + offset.to_bytes(3, "big") + bytes((frame["type"], 255, frame["width"] // 8, frame["height"] // 8))
        yield rtp + header + quant + scan[offset:offset + count]
        offset += count
        sequence += 1


def rtcp_report(ssrc, timestamp, packet_count, byte_count):
    ntp = time.time() + 2208988800
    seconds = int(ntp)
    report = struct.pack(">BBHIIIIII", 0x80, 200, 6, ssrc, seconds,
                         int((ntp - seconds) * (1 << 32)), timestamp,
                         packet_count & 0xffffffff, byte_count & 0xffffffff)
    cname = b"maixcam2-fusion"
    chunk = struct.pack(">I", ssrc) + bytes((1, len(cname))) + cname + b"\0"
    chunk += b"\0" * (-len(chunk) % 4)
    return report + struct.pack(">BBH", 0x81, 202, len(chunk) // 4) + chunk


class RtspClient:
    def __init__(self, server, conn, address):
        self.server, self.conn, self.address = server, conn, address
        conn.settimeout(1.0)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.alive = threading.Event()
        self.alive.set()
        self.playing = threading.Event()
        self.write_lock = threading.Lock()
        self.condition = threading.Condition()
        self.pending = None
        self.channel, self.rtcp_channel = 0, 1
        self.session = secrets.token_hex(8)
        self.setup_done = False
        self.sequence, self.ssrc = secrets.randbits(16), secrets.randbits(32)
        self.packet_count = self.byte_count = 0
        self.last_rtcp = 0.0
        self.reader = threading.Thread(target=self._serve, name="rtsp-control", daemon=True)
        self.writer = threading.Thread(target=self._write_loop, name="rtsp-video", daemon=True)

    def start(self):
        self.writer.start()
        self.reader.start()

    def publish(self, frame, timestamp):
        if self.playing.is_set():
            with self.condition:
                self.pending = frame, timestamp
                self.condition.notify()

    def _response(self, cseq, code=200, reason="OK", headers=None, body=b""):
        h = {"CSeq": cseq, "Server": "MaixFusion-RTSP", "Content-Length": str(len(body))}
        if headers:
            h.update(headers)
        message = "RTSP/1.0 %d %s\r\n" % (code, reason)
        message += "".join("%s: %s\r\n" % (k, v) for k, v in h.items()) + "\r\n"
        with self.write_lock:
            self.conn.sendall(message.encode("ascii") + body)

    def _request(self, request):
        lines = request.decode("latin1").split("\r\n")
        method, url, version = lines[0].split(" ", 2)
        headers = {}
        for line in lines[1:]:
            if ":" in line:
                key, value = line.split(":", 1)
                headers[key.lower()] = value.strip()
        cseq = headers.get("cseq", "0")
        path = urlsplit(url).path.rstrip("/")
        if method != "OPTIONS" and path not in ("/live", "/live/trackID=0"):
            self._response(cseq, 404, "Not Found")
            return
        if method == "OPTIONS":
            self._response(cseq, headers={"Public": "OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, GET_PARAMETER, TEARDOWN"})
        elif method == "DESCRIBE":
            base = url.rstrip("/") + "/"
            sdp = ("v=0\r\no=- 1 1 IN IP4 0.0.0.0\r\ns=MaixCAM2 Thermal YOLO Fusion\r\n"
                   "c=IN IP4 0.0.0.0\r\nt=0 0\r\na=control:*\r\n"
                   "m=video 0 RTP/AVP 26\r\na=rtpmap:26 JPEG/90000\r\n"
                   "a=framerate:%d\r\na=control:trackID=0\r\n" % self.server.fps).encode("ascii")
            self._response(cseq, headers={"Content-Type": "application/sdp", "Content-Base": base}, body=sdp)
        elif method == "SETUP":
            transport = headers.get("transport", "")
            match = re.search(r"interleaved=(\d+)-(\d+)", transport, re.I)
            if "RTP/AVP/TCP" not in transport.upper() or not match:
                self._response(cseq, 461, "Unsupported Transport")
                return
            channels = tuple(map(int, match.groups()))
            if channels[0] == channels[1] or not all(0 <= ch <= 255 for ch in channels):
                self._response(cseq, 461, "Unsupported Transport")
                return
            self.channel, self.rtcp_channel = channels
            self.setup_done = True
            self._response(cseq, headers={"Session": self.session,
                "Transport": "RTP/AVP/TCP;unicast;interleaved=%d-%d;ssrc=%08X" % (*channels, self.ssrc)})
        elif method in ("PLAY", "PAUSE", "TEARDOWN", "GET_PARAMETER"):
            session = headers.get("session", "").split(";")[0]
            if not self.setup_done or session != self.session:
                self._response(cseq, 454, "Session Not Found")
                return
            if method in ("PAUSE", "TEARDOWN"):
                self.playing.clear()
            self._response(cseq, headers={"Session": self.session})
            if method == "PLAY":
                self.playing.set()
            elif method == "TEARDOWN":
                self.close()
        else:
            self._response(cseq, 405, "Method Not Allowed")

    def _serve(self):
        buffer = bytearray()
        last_request = time.monotonic()
        try:
            while self.alive.is_set():
                try:
                    data = self.conn.recv(8192)
                except socket.timeout:
                    if not self.setup_done and time.monotonic() - last_request > 15:
                        break
                    continue
                if not data:
                    break
                buffer.extend(data)
                if len(buffer) > 65536:
                    raise ValueError("RTSP request too large")
                while buffer:
                    # Ignore incoming RTP/RTCP interleaved packets from the client.
                    if buffer[0] == 36:
                        if len(buffer) < 4:
                            break
                        end = 4 + int.from_bytes(buffer[2:4], "big")
                        if len(buffer) < end:
                            break
                        del buffer[:end]
                        continue
                    idx = buffer.find(b"\r\n\r\n")
                    if idx < 0:
                        break
                    head = bytes(buffer[:idx + 4])
                    match = re.search(rb"(?im)^Content-Length:\s*(\d+)", head)
                    content_len = int(match.group(1)) if match else 0
                    if content_len > 8192:
                        raise ValueError("RTSP body too large")
                    end = idx + 4 + content_len
                    if len(buffer) < end:
                        break
                    del buffer[:end]
                    last_request = time.monotonic()
                    self._request(head)
        except (OSError, ValueError) as exc:
            LOG.debug("RTSP control ended: %s", exc)
        finally:
            self.close()
            self.server.remove(self)

    def _write_loop(self):
        try:
            while self.alive.is_set():
                with self.condition:
                    if self.pending is None:
                        self.condition.wait(timeout=.5)
                    pending, self.pending = self.pending, None
                if pending is None or not self.playing.is_set():
                    continue
                frame, timestamp = pending
                with self.write_lock:
                    for packet in jpeg_rtp_packets(frame, timestamp, self.sequence, self.ssrc):
                        self.conn.sendall(struct.pack(">BBH", 36, self.channel, len(packet)) + packet)
                        self.sequence = (self.sequence + 1) & 65535
                        self.packet_count += 1
                        self.byte_count += len(packet) - 12
                    if time.monotonic() - self.last_rtcp > 3:
                        report = rtcp_report(self.ssrc, timestamp, self.packet_count, self.byte_count)
                        self.conn.sendall(struct.pack(">BBH", 36, self.rtcp_channel, len(report)) + report)
                        self.last_rtcp = time.monotonic()
        except OSError as exc:
            LOG.debug("RTSP slow/disconnected client: %s", exc)
            self.close()

    def close(self):
        self.alive.clear()
        self.playing.clear()
        with self.condition:
            self.condition.notify_all()
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.conn.close()


class JpegRtspServer:
    def __init__(self, host="0.0.0.0", port=8554, fps=8, quality=70, max_clients=2):
        self.host, self.port, self.fps, self.quality = host, port, fps, quality
        self.max_clients = max_clients
        self.clients, self.clients_lock = [], threading.Lock()
        self.stop_event = threading.Event()
        self.condition = threading.Condition()
        self.pending = None
        self.last_submit = 0.0
        self.error = ""
        self.encoded = 0
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener.bind((host, port))
            self.port = self.listener.getsockname()[1]
            self.listener.listen(max_clients)
            self.listener.settimeout(.5)
        except Exception:
            self.listener.close()
            raise
        self.accept_thread = threading.Thread(target=self._accept, name="rtsp-accept", daemon=True)
        self.encode_thread = threading.Thread(target=self._encode, name="rtsp-jpeg", daemon=True)
        self.accept_thread.start()
        self.encode_thread.start()

    def client_count(self):
        with self.clients_lock:
            return sum(c.playing.is_set() and c.alive.is_set() for c in self.clients)

    def remove(self, client):
        with self.clients_lock:
            if client in self.clients:
                self.clients.remove(client)

    def _accept(self):
        while not self.stop_event.is_set():
            try:
                conn, address = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self.clients_lock:
                if len(self.clients) >= self.max_clients:
                    conn.close()
                    continue
                client = RtspClient(self, conn, address)
                self.clients.append(client)
            client.start()

    def submit(self, rgb, now=None):
        now = time.monotonic() if now is None else now
        if now - self.last_submit < 1.0 / self.fps or not self.client_count():
            return
        self.last_submit = now
        with self.condition:
            self.pending = rgb.copy(), int(now * 90000) & 0xffffffff
            self.condition.notify()

    def _encode(self):
        while not self.stop_event.is_set():
            with self.condition:
                if self.pending is None:
                    self.condition.wait(timeout=.5)
                pending, self.pending = self.pending, None
            if pending is None:
                continue
            try:
                rgb, timestamp = pending
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality,
                    cv2.IMWRITE_JPEG_PROGRESSIVE, 0, cv2.IMWRITE_JPEG_OPTIMIZE, 0,
                    cv2.IMWRITE_JPEG_RST_INTERVAL, 0])
                if not ok:
                    raise RuntimeError("OpenCV JPEG encoding failed")
                frame = parse_jpeg(jpeg.tobytes())
                with self.clients_lock:
                    clients = list(self.clients)
                for client in clients:
                    client.publish(frame, timestamp)
                self.encoded += 1
                self.error = ""
            except Exception as exc:
                if not self.error:
                    LOG.exception("RTSP encoding failed")
                self.error = str(exc)

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        self.listener.close()
        with self.clients_lock:
            clients = list(self.clients)
        for client in clients:
            client.close()
        for thread in (self.accept_thread, self.encode_thread):
            thread.join(timeout=2.0)
        for client in clients:
            client.reader.join(timeout=1.5)
            client.writer.join(timeout=1.5)
