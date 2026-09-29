import unittest

from demo import DryRunTransport
from f32c_protocol import F32CGimbal
from motor_status import read_dual_motor_status, read_motor_status


class MotorStatusTests(unittest.TestCase):
    def setUp(self):
        self.transport = DryRunTransport()
        self.gimbal = F32CGimbal(self.transport)
        self.gimbal.start(power_on_delay_s=0.0)

    def tearDown(self):
        self.gimbal.stop()

    def test_parameter_address_is_separate_from_motor_address(self):
        status = read_motor_status(self.gimbal, 2, (0x01, 0x04))
        self.assertEqual(status["response_motor_address"], 2)
        self.assertEqual(
            status["parameters"]["total_angle"]["parameter_address"], 0x01
        )
        self.assertEqual(
            status["parameters"]["bus_voltage"]["parameter_address"], 0x04
        )
        self.assertEqual(status["parameters"]["bus_voltage"]["value"], 12.0)

    def test_dual_motor_angle_values(self):
        self.gimbal.set_relative_angles(12.3, -45.6)
        status = read_dual_motor_status(self.gimbal, (0x01,))
        self.assertAlmostEqual(
            status["x"]["parameters"]["total_angle"]["value"], 12.3
        )
        self.assertAlmostEqual(
            status["y"]["parameters"]["total_angle"]["value"], -45.6
        )


if __name__ == "__main__":
    unittest.main()

