"""Manufacturer's >=1ms idle framing contract, including setup bursts."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m=app.m


class UartFrameGapTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    make_driver=controls.SeparateControlTests.make_driver
    enable=controls.SeparateControlTests.enable

    def test_configuration_and_position_frames_are_not_queued_back_to_back(self):
        d,bus=self.make_driver((0.,0.));stamps=[];original=bus.write
        def stamped(data):
            stamps.append((self.clock.now,bytes(data)))
            return original(data)
        bus.write=stamped
        self.enable(d)
        self.assertGreater(len(stamps),5)
        for (before,_),(after,packet) in zip(stamps,stamps[1:]):
            with self.subTest(opcode=packet[2]):self.assertGreaterEqual(after-before,.002-1e-9)
