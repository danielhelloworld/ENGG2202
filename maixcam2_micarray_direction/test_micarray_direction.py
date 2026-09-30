"""Pure host tests for the independent MA-USB8 direction program."""

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().with_name("micarray_direction.py")
SPEC = importlib.util.spec_from_file_location("micarray_direction_standalone", MODULE_PATH)
direction = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(direction)


def blob(col, row, value=230):
    grid = bytearray([8] * 256)
    for rr in range(row - 1, row + 2):
        for cc in range(col - 1, col + 2):
            grid[rr * 16 + cc] = value
    return bytes(grid)


class ParserTests(unittest.TestCase):
    def test_chunk_boundaries_and_ff_inside_payload(self):
        payload = bytearray(blob(12, 8))
        payload[0:16] = direction.HEADER
        stream = b"noise" + direction.HEADER + payload + direction.HEADER + blob(8, 3) + direction.HEADER
        parser = direction.HotmapFrameParser()
        output = []
        for byte in stream:
            output.extend(parser.feed(bytes([byte])))
        self.assertEqual(output, [bytes(payload), blob(8, 3)])
        self.assertEqual(parser.discarded_bytes, 5)

    def test_bounded_after_noise(self):
        parser = direction.HotmapFrameParser()
        self.assertEqual(parser.feed(b"X" * (direction.MAX_BUFFER * 3)), [])
        self.assertLessEqual(len(parser.buffer), 15)


class DirectionTests(unittest.TestCase):
    def test_right_blob_and_rotation(self):
        estimator = direction.SoundDirectionEstimator()
        result = estimator.update(blob(12, 8), now=10.0)
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["image_bearing_deg"], 90, delta=12)
        rotated = estimator.snapshot(now=10.1, rotation=90)
        self.assertAlmostEqual(rotated["image_bearing_deg"], 180, delta=12)

    def test_ambiguous_center_and_stale(self):
        estimator = direction.SoundDirectionEstimator()
        self.assertIsNone(estimator.update(blob(8, 8), now=1.0)["image_bearing_deg"])
        self.assertEqual(estimator.snapshot(now=2.5)["reason"], "stale")
        two = bytearray(blob(3, 8))
        other = blob(12, 8)
        for index, value in enumerate(other):
            two[index] = max(two[index], value)
        self.assertEqual(estimator.update(bytes(two), now=3.0)["reason"],
                         "ambiguous_multiple_sources")

    def test_callable_sensor_feed_and_no_serial_side_effect(self):
        sensor = direction.SoundDirectionSensor(mirror=True)
        self.assertIsNone(sensor.serial)
        packet = direction.HEADER + blob(12, 8) + direction.HEADER
        self.assertEqual(sensor.feed_bytes(packet, now=10.0), 1)
        result = sensor.snapshot(now=10.0)
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["image_bearing_deg"], 270, delta=12)
        self.assertEqual(result["frames"], 1)


if __name__ == "__main__":
    unittest.main()
