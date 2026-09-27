# Hardware Bring-Up Procedure

This procedure deliberately separates visual validation from motor activation. Keep people, cables, tools, and hard obstacles outside the gimbal travel envelope.

## 1. Verify power with all devices disconnected

1. Confirm the motor supply is within 8-15 V.
2. Confirm the MaixCAM 2 regulator produces a stable 5 V under load.
3. Confirm the two supplies share signal ground only where intended.
4. Confirm polarity before connecting either device.
5. Do not connect or disconnect an F32C interface while powered.

## 2. Verify TTL wiring

1. Connect MaixCAM 2 A21/UART4_TX to the gimbal RX line.
2. Connect MaixCAM 2 A22/UART4_RX to the gimbal TX line.
3. Connect grounds.
4. Confirm the signals are 3.3 V TTL.
5. Confirm pan motor ID `1` and tilt motor ID `2` using the vendor tool or known configuration.

## 3. Run vision-only mode

Keep `GimbalConfig.enabled = False`.

1. Place one person in view.
2. Run `main.py`.
3. Confirm a green box follows the person.
4. Confirm the mode reaches `TRACK` after three valid frames.
5. Briefly hide the person and confirm `COAST`, followed by `SEARCH` if the person stays hidden.
6. Confirm the yellow predicted marker continues through a short occlusion.
7. Inspect `/root/logs/person_tracker.csv`.

Do not continue if detections are unstable or the application reports an unexpected target angle sign.

## 4. Establish mechanical zero and limits

With motor power removed, place the camera at the intended neutral forward position.

1. Determine the F32C total-angle reading at neutral for each axis.
2. Set `zero_offset_deg` so the application's logical pose reads approximately `(0, 0)` at neutral.
3. Determine conservative pan and tilt travel limits that leave clearance before cables, hard stops, and structural interference.
4. Enter those limits in `minimum_deg` and `maximum_deg`.
5. Keep initial maximum rates below 20 deg/s if the axis direction has not yet been confirmed.

Do not issue the F32C persistent zero-setting command during normal application startup. Treat zero as calibration data in `config.py` until the complete mechanism has been validated.

## 5. Verify one axis at a time

The application currently initializes both motors when gimbal output is enabled. For initial bench verification, mechanically support the payload and temporarily set both axes to narrow limits around zero.

1. Set pan limits to approximately `-5` and `+5` degrees.
2. Set tilt limits to approximately `-3` and `+3` degrees.
3. Set maximum rates and accelerations to conservative values.
4. Set `GimbalConfig.enabled = True`.
5. Start the application with the person centered.
6. Move slowly to the image right. If the camera turns away from the person, stop immediately and reverse the pan `motor_sign` or camera `pan_pixel_sign` after determining which coordinate is wrong.
7. Repeat vertically and correct the tilt signs.

Never diagnose an incorrect sign by allowing the gimbal to run into a limit.

## 6. Expand the motion envelope

After direction and zero are correct:

1. Increase limits in small steps while checking cable clearance.
2. Increase maximum rate and acceleration gradually.
3. Tune F32C internal PID using the vendor procedure if the installed payload oscillates or overshoots.
4. Increase the software command rate only after UART feedback remains reliable.
5. Validate normal exit, application exception, feedback disconnection, and power cycling.

## 7. Closed-loop person test

1. Start with one person, uniform lighting, and a clear background.
2. Walk slowly across the field of view.
3. Confirm the target remains near the image center without sustained oscillation.
4. Add a second person and confirm the tracker does not jump when the locked person remains inside the association gate.
5. Occlude the target for 0.2-0.4 seconds and verify successful coast and reacquisition.
6. Extend the occlusion until the system enters local search.
7. Review CSV data for bearing rate, covariance growth, gimbal command, and state transitions.

## 8. Acceptance criteria for Stage 1

- Stable person lock without continuous gimbal oscillation.
- Correct pan and tilt response in every image quadrant.
- No command beyond configured software limits.
- Recovery from a 0.3 second person occlusion under a predictable walking motion.
- Transition to `FAULT` when both motor angles are not received within the configured timeout.
- Normal stop behavior matches the configured hold/disable policy.
- A usable telemetry record exists for every tuning run.
