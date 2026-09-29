import queue
import sys
import time
import unittest

from gimbal_gui import GimbalWorker, pyserial_install_command


class GUIWorkerTests(unittest.TestCase):
    def setUp(self):
        self.events = queue.Queue()
        self.worker = GimbalWorker(None, 20, 100, self.events, demo_mode=True)
        self.worker.start()

    def tearDown(self):
        if self.worker.is_alive():
            self.worker.request_stop()
            self.worker.join(2.0)

    def wait_for(self, kind, predicate=lambda _payload: True, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                event_kind, payload = self.events.get(timeout=0.1)
            except queue.Empty:
                continue
            if event_kind == "error":
                self.fail(payload)
            if event_kind == kind and predicate(payload):
                return payload
        self.fail("timed out waiting for GUI worker event: %s" % kind)

    def test_disable_keeps_feedback_and_reenable_restores_commands(self):
        self.wait_for("connected")
        self.worker.set_target(360.0, 0.0)
        self.wait_for("target", lambda value: value == (360.0, 0.0))

        self.worker.set_enabled(False)
        self.wait_for("motor_enabled", lambda value: value is False)
        self.wait_for("position")
        self.assertTrue(self.worker.is_alive())

        self.worker.set_enabled(True)
        self.wait_for("motor_enabled", lambda value: value is True)
        self.worker.set_target(0.0, 0.0)
        self.wait_for("target", lambda value: value == (0.0, 0.0))

    def test_motion_and_pid_commands_are_serialized_with_feedback(self):
        self.wait_for("connected")
        self.worker.set_motion_parameters(15, 50)
        self.assertEqual(self.wait_for("motion"), (15, 50))
        self.worker.set_speed_pid("X", 10, 10, save=False)
        self.assertEqual(self.wait_for("pid"), ("X", 10, 10, False))

    def test_install_hint_targets_the_running_interpreter(self):
        command = pyserial_install_command()
        self.assertIn(sys.executable, command)
        self.assertTrue(command.endswith("-m pip install pyserial"))


if __name__ == "__main__":
    unittest.main()
