"""Separate MaixVision maintenance entry. Does not load YOLO or run tracking."""
ZERO_AXIS = "Y"  # X or Y, one axis per run.
CONFIRM_PHYSICAL_ZERO = False  # Set True only after supporting load at true zero.


def run_zero(c=CONFIG, axis=ZERO_AXIS, confirm=CONFIRM_PHYSICAL_ZERO):
    if not confirm:
        raise ValueError("Calibration not confirmed. Support the disabled load at true physical zero first.")
    if axis not in ("X", "Y"):
        raise ValueError("ZERO_AXIS must be X or Y")
    from maix import sys as maix_sys
    if str(maix_sys.device_name()).lower() != "maixcam2":
        raise RuntimeError("This UART configuration requires MaixCAM2")
    # Calibration is explicitly authorized here, independent of sound readiness.
    maintenance = dict(c, motor_enabled=False)
    motor = MotorService(maintenance)
    motor.calibrate_mechanical_zero(c["motor_ids"][0 if axis == "X" else 1], confirm=True)


if __name__ == "__main__":
    run_zero()
