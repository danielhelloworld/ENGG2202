# MaixCAM 2 Person-Tracking Gimbal

This project is the Stage 1 closed-loop validation build for a future airborne-drone tracker. It runs on a MaixCAM 2, detects one person with YOLO11, estimates the person's world bearing, and commands a two-axis WHEELTEC F32C brushless gimbal over 3.3 V TTL UART.

The project is intentionally divided so the person detector can later be replaced by a custom drone detector without changing the state estimator, loss-recovery state machine, gimbal controller, motor protocol, or telemetry pipeline.

## Current stage

Implemented:

- YOLO11 person detection using COCO class ID `0`;
- stable single-person selection when several people are visible;
- pixel-to-bearing conversion using configurable camera field of view;
- dependency-free constant-angular-velocity Kalman filtering;
- `BOOT`, `ACQUIRE`, `TRACK`, `COAST`, `SEARCH`, and `FAULT` states;
- prediction during short detector loss;
- expanding local search after the coast interval expires;
- acceleration- and rate-limited gimbal position commands;
- F32C direct multi-turn position protocol and angle feedback parsing;
- non-blocking CSV telemetry and display overlays;
- a vision-only mode for safe first execution.
- a headless default mode with one-second console status reports.

Deferred to later stages:

- MixFormerV2 appearance tracking between YOLO frames;
- frame-time gimbal-pose interpolation;
- IMU rate fusion;
- dynamic high-resolution ROI detection;
- custom drone YOLO model and appearance-based reacquisition;
- full 3D camera-to-gimbal rotation calibration.

## Safety default

`config.py` ships with:

```python
enabled: bool = False
```

The application will run the complete vision, target-selection, estimation, state-machine, overlay, and logging pipeline, but it will not open UART4 or move the gimbal.

The display is also disabled by default so the application can be started over SSH when the device's display service is already occupied. Detection count, selected-person pixels, estimated world bearing, mode, and gimbal pose are printed once per second. Set `UiConfig.enabled = True` to request the onboard display when it is available.

Do not enable the gimbal until motor IDs, axis directions, zero offsets, mechanical soft limits, wiring, and power supplies have been checked using the procedure in [docs/BRINGUP.md](docs/BRINGUP.md).

## Hardware interface

| Signal | MaixCAM 2 | F32C gimbal |
|---|---|---|
| TX | A21 / UART4_TX | RX |
| RX | A22 / UART4_RX | TX |
| Ground | GND | GND |

UART settings are `115200 8N1`. MaixCAM 2 and F32C use 3.3 V logic. Do not apply 5 V logic to the MaixCAM pins.

Use separate regulated power paths:

- F32C motors: 8-15 V, normally a 3S supply;
- MaixCAM 2: stable regulated 5 V;
- common signal ground between the two systems.

Do not hot-plug any F32C connector.

## Model

The default model path is:

```text
/root/models/yolo11n.mud
```

The model must be a MaixCAM 2 YOLO11 package containing the `.mud` file and every referenced `.axmodel` file. Its label list must use the standard COCO order, where `person` is class ID `0`.

If the model is stored elsewhere, change `ModelConfig.path` in `config.py`.

## Deployment

1. Copy the entire `maixcam2_person_tracker` directory to the MaixCAM 2 with MaixVision.
2. Confirm the YOLO11 model exists at the configured path.
3. Open and run `main.py` from the project directory.
4. Verify that a person receives a green detection box and that the mode progresses from `ACQUIRE` to `TRACK`.
5. Review the yellow predicted point and the CSV log at `/root/logs/person_tracker.csv`.
6. Stop the application and complete the motor bring-up checklist before setting `GimbalConfig.enabled` to `True`.

The Mac is a development and monitoring host only. The real-time loop runs entirely on the MaixCAM 2.

## Workbench integration

The optional [MaixCAM 2 Workbench](../workbench/README.md) provides a browser dashboard, MJPEG proxy, telemetry categories, and runtime status. It does not take ownership of the camera or inference loop.

To publish status, set the `endpoint` default inside `WorkbenchConfig` in `config.py` to the Mac USB-network address ending in `/api/telemetry`:

```python
endpoint: str = "http://10.177.5.100:8760/api/telemetry"
```

The publisher is disabled when the endpoint is empty. When enabled, it sends the newest camera, person detection, tracking, gimbal, model, and process snapshot from a background thread. A disconnected dashboard does not block the vision loop. Keep the workbench gateway bound to the Mac's USB-network interface; its telemetry endpoint is unauthenticated.

## Configuration that must be measured

The values in `config.py` are conservative starting points, not final calibration values:

- camera horizontal and vertical field of view;
- pan and tilt motor direction signs;
- logical zero offsets;
- pan and tilt mechanical soft limits;
- stable maximum rate and acceleration under the installed payload;
- detector confidence and reacquisition gate;
- coast and search timing.

## Runtime behavior

The tracker requires three consecutive person detections before commanding the gimbal. During a brief miss it enters `COAST` and commands the predicted future bearing. If the miss lasts too long or uncertainty grows too large, it enters `SEARCH` and scans an expanding region around the predicted bearing. If local search fails, the old track is cleared and the system returns to full-frame acquisition.

The F32C motors run their internal position and speed loops. MaixCAM 2 sends direct multi-turn position targets at up to 50 Hz by default and polls total-angle feedback from each motor in turn.

## Project layout

```text
main.py                         Application entry point
application.py                  Real-time orchestration
config.py                       All tunable values
domain.py                       Shared data contracts
perception/detector_yolo.py     YOLO11 adapter
perception/target_selector.py   Single-person association
estimation/camera_model.py      Pixel/bearing conversion
estimation/target_filter.py     Constant-velocity Kalman filter
control/state_machine.py        Loss and recovery behavior
control/search_pattern.py       Expanding local scan
control/gimbal_controller.py    Command prediction and motion limits
drivers/f32c_protocol.py        Binary F32C protocol
drivers/gimbal_uart.py          UART ownership and motor state
telemetry/csv_logger.py         Bounded asynchronous logging
telemetry/mjpeg_stream.py       Annotated MJPEG camera endpoint
telemetry/workbench_publisher.py Optional asynchronous desktop telemetry
telemetry/overlay.py            On-screen diagnostics
docs/ARCHITECTURE.md            Design and module boundaries
docs/BRINGUP.md                 Safe hardware activation procedure
```

## Moving to the drone stage

Replace `YoloPersonDetector` with a detector that returns the same `Detection` data type. Change the model path, class ID, label, confidence threshold, and reacquisition gate. The rest of the application does not depend on the target category.

For a distant drone, use at least a `640x480` model input, collect data through the actual lens and sky background, and expect optics and motion blur to become limiting factors before NPU throughput.
