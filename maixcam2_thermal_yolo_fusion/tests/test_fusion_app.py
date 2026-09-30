"""Local algorithm + real FFmpeg RTSP decode tests. No MaixCAM hardware is mocked as passed."""
import json
import math
from pathlib import Path
import runpy
import shutil
import socket
import struct
import subprocess
import threading
import time
import types
import unittest

import cv2
import numpy as np

APP = Path(__file__).resolve().parents[1]
m = types.SimpleNamespace(**runpy.run_path(str(APP / 'main.py')))


def body(pixel=127, lo=200, hi=400, vtemp=8192):
    return bytes([pixel]) * 19200 + struct.pack('>HhhiHHfHHBBhhh', vtemp, lo, hi, 0, 0, 0, 0., 0, 0, 0, 0, 0, 0, 0)


def wire(**kwargs):
    return b'\xff' + body(**kwargs)


class ThermalTests(unittest.TestCase):
    def test_new_protocol_and_endianness(self):
        frame = m.decode_body(body(lo=-100, hi=400), 1.25)
        self.assertEqual(frame['pixels'].shape, (120, 160))
        self.assertEqual((frame['lo'], frame['hi'], frame['received_at']), (-100, 400, 1.25))

    def test_fragmented_ff_pixels_and_two_frame_sync(self):
        parser = m.ThermalParser(skip=0)
        raw = wire(pixel=255) + wire(pixel=42)
        self.assertIsNone(parser.feed(raw[:19231], 1))
        self.assertIsNone(parser.feed(raw[19231:30000], 1))
        frame = parser.feed(raw[30000:], 1)
        self.assertEqual(parser.frames, 2)
        self.assertEqual(int(frame['pixels'][0, 0]), 42)

    def test_every_split_boundary_including_header_and_tail(self):
        raw = wire() + wire(pixel=33) + wire(pixel=66)
        for size in (1, 7, 4096, 19230, 19231, 32768):
            parser = m.ThermalParser(skip=0)
            frames = []
            for pos in range(0, len(raw), size):
                frame = parser.feed(raw[pos:pos + size], 2)
                if frame is not None:
                    frames.append(frame)
            self.assertEqual(parser.frames, 3)
            self.assertEqual(int(frames[-1]['pixels'][0, 0]), 66)
            self.assertLess(len(parser.buffer), m.WIRE_SIZE * 2)

    def test_skip_exactly_ten(self):
        parser = m.ThermalParser(skip=10)
        self.assertIsNone(parser.feed(wire() * 10, 1))
        self.assertIsNotNone(parser.feed(wire(), 2))

    def test_noise_after_locked_frame_does_not_discard_it(self):
        parser = m.ThermalParser(skip=0)
        parser.feed(wire() * 2, 1)
        frame = parser.feed(wire(pixel=55) + b'noise', 2)
        self.assertEqual(int(frame['pixels'][0, 0]), 55)
        frame = parser.feed(wire(pixel=56) * 2, 3)
        self.assertEqual(int(frame['pixels'][0, 0]), 56)

    def test_noise_and_corruption_resync(self):
        parser = m.ThermalParser(skip=0)
        frame = parser.feed(b'noise' + wire(vtemp=0xffff) + wire(pixel=70) * 2, 1)
        self.assertEqual(int(frame['pixels'][0, 0]), 70)
        self.assertGreater(parser.discarded_bytes, 0)

    def test_no_temperature_for_invalid_or_flat_range(self):
        for lo, hi in ((32767, 32767), (200, 200)):
            self.assertFalse(m.decode_body(body(lo=lo, hi=hi), 0)['valid_temperature'])


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.c = dict(m.CONFIG)
        # These legacy geometry assertions explicitly exercise a full-frame footprint.
        self.c['thermal_rect'] = [0., 0., 1., 1.]
        self.a = m.Alignment(self.c, (640, 480))

    def test_letterbox_and_detection_geometry(self):
        a = m.Alignment(self.c, (640, 640))
        self.assertEqual(a.display_box((0, 0, 640, 640)), (80, 0, 480, 480))

    def test_raw_254_scale_and_clipped_ff(self):
        for pixel, expected in ((0, 20), (127, 30), (254, 40), (255, 40)):
            frame = m.decode_body(body(pixel=pixel), 1)
            result = m.roi_temperature(frame, (0, 0, 640, 480), self.a)
            self.assertAlmostEqual(result['max'], expected, places=4)

    def test_flipped_hot_pixel_matches_roi(self):
        frame = m.decode_body(body(pixel=0), 1)
        frame['pixels'] = frame['pixels'].copy()
        frame['pixels'][10, 20] = 254
        x, y = self.a.mx[10, 20], self.a.my[10, 20]
        self.assertGreater(x, 500)
        self.assertGreater(y, 400)
        result = m.roi_temperature(frame, (x - .1, y - .1, .2, .2), self.a)
        self.assertEqual(result['samples'], 1)
        self.assertAlmostEqual(result['max'], 40)

    def test_outside_thermal_footprint_has_no_temperature(self):
        self.c['thermal_rect'] = [.25, .25, .5, .5]
        a = m.Alignment(self.c, (640, 480))
        self.assertIsNone(m.roi_temperature(m.decode_body(body(), 1), (0, 0, 40, 40), a))

    def test_stale_desync_and_uncalibrated_states(self):
        frame = m.decode_body(body(), 1)
        self.assertEqual(m.thermal_state(frame, 1, 2, self.c), ('THERMAL STALE', False, False))
        self.assertEqual(m.thermal_state(frame, 1.4, 1.4, self.c), ('THERMAL DESYNC', False, False))
        uncal = m.decode_body(body(lo=32767, hi=32767), 1)
        self.assertEqual(m.thermal_state(uncal, 1, 1, self.c), ('UNCALIBRATED', True, False))

    def test_stale_frame_does_not_colorize_visible_image(self):
        rgb = np.full((480, 640, 3), (12, 34, 56), np.uint8)
        frame = m.decode_body(body(), 1)
        out, _, _ = m.compose(rgb, [], frame, self.a, 3, 3)
        np.testing.assert_array_equal(out[200, 200], rgb[200, 200])

    def test_alpha_zero_keeps_base(self):
        rgb = np.full((480, 640, 3), (60, 80, 90), np.uint8)
        out, _, _ = m.compose(rgb, [], m.decode_body(body(), 1), self.a, 1, 1, alpha=0)
        np.testing.assert_array_equal(out[200, 200], rgb[200, 200])

    def test_touch_geometry_for_nonstandard_screen(self):
        self.assertEqual(m.touch_to_canvas(640, 400, 1280, 800, 640, 480), (320, 240))


class RtpTests(unittest.TestCase):
    def test_quantization_fragmentation_and_marker(self):
        rng = np.random.default_rng(7)
        rgb = rng.integers(0, 255, (240, 320, 3), np.uint8)
        ok, jpeg = cv2.imencode('.jpg', rgb, [cv2.IMWRITE_JPEG_QUALITY, 70])
        self.assertTrue(ok)
        frame = m.parse_jpeg(jpeg.tobytes())
        packets = list(m.jpeg_rtp_packets(frame, 12345, 65535, 99, mtu=600))
        self.assertGreater(len(packets), 2)
        scan = bytearray()
        for idx, packet in enumerate(packets):
            self.assertLessEqual(len(packet), 600)
            v, pt, seq, ts, ssrc = struct.unpack_from('>BBHII', packet)
            self.assertEqual((v, pt & 127, seq, ts, ssrc), (128, 26, (65535 + idx) & 65535, 12345, 99))
            self.assertEqual(bool(pt & 128), idx == len(packets) - 1)
            offset = int.from_bytes(packet[13:16], 'big')
            self.assertEqual(offset, len(scan))
            self.assertEqual(packet[17], 255)
            payload = packet[20:]
            if idx == 0:
                self.assertEqual(payload[:4], b'\0\0\0\x80')
                self.assertEqual(payload[4:132], frame['tables'])
                payload = payload[132:]
            scan.extend(payload)
        self.assertEqual(bytes(scan), frame['scan'])

    def test_udp_transport_is_rejected_explicitly(self):
        server = m.JpegRtspServer('127.0.0.1', 0)
        try:
            with socket.create_connection(('127.0.0.1', server.port), timeout=2) as sock:
                request = ('SETUP rtsp://127.0.0.1:%d/live/trackID=0 RTSP/1.0\r\n'
                           'CSeq: 1\r\nTransport: RTP/AVP;unicast;client_port=5000-5001\r\n\r\n') % server.port
                sock.sendall(request.encode())
                self.assertIn(b'461 Unsupported Transport', sock.recv(4096))
        finally:
            server.close()

    @unittest.skipUnless(shutil.which('ffmpeg'), 'FFmpeg not installed')
    def test_real_ffmpeg_decodes_composite_over_rtsp_tcp(self):
        c = dict(m.CONFIG)
        alignment = m.Alignment(c, (640, 480))
        rgb, thermal, detections = m.demo_frame()
        now = thermal['received_at']
        reference, _, _ = m.compose(rgb, detections, thermal, alignment, now, now)
        ui = m.TouchUI(alignment)
        ui.draw(reference, 'SYNTHETIC VALIDATION | no physical sensors')
        m.draw_text_rgb(reference, 'SYNTHETIC TEST', 340, 88, (255, 255, 255), .6)
        server = m.JpegRtspServer('127.0.0.1', 0, fps=8)
        stop = threading.Event()
        def produce():
            while not stop.wait(.04):
                server.submit(reference)
        producer = threading.Thread(target=produce, daemon=True)
        producer.start()
        started = time.monotonic()
        try:
            url = 'rtsp://127.0.0.1:%d/live' % server.port
            result = subprocess.run([shutil.which('ffmpeg'), '-hide_banner', '-loglevel', 'error',
                '-rtsp_transport', 'tcp', '-analyzeduration', '1000000', '-probesize', '32768',
                '-i', url, '-frames:v', '3', '-pix_fmt', 'rgb24', '-f', 'rawvideo', 'pipe:1'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
            self.assertEqual(len(result.stdout), 3 * 640 * 480 * 3, result.stderr.decode(errors='replace'))
            decoded = np.frombuffer(result.stdout[:640 * 480 * 3], np.uint8).reshape(480, 640, 3)
            mae = float(np.abs(decoded.astype(np.int16) - reference.astype(np.int16)).mean())
            self.assertLess(mae, 12)
            output = APP / 'validation'
            output.mkdir(exist_ok=True)
            cv2.imwrite(str(output / 'composite_reference.png'), cv2.cvtColor(reference, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(output / 'rtsp_decoded.png'), cv2.cvtColor(decoded, cv2.COLOR_RGB2BGR))
            report = {'test': 'real FFmpeg decode via RTSP TCP', 'frames': 3,
                      'width': 640, 'height': 480, 'rgb_mean_absolute_error': mae,
                      'seconds': round(time.monotonic() - started, 3), 'hardware_tested': False,
                      'synthetic_inputs': True, 'stderr': result.stderr.decode(errors='replace')}
            (output / 'rtsp_decode.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        finally:
            stop.set()
            producer.join(timeout=1)
            server.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
