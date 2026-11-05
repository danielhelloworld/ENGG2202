# ENGG2202 MaixCAM 2 Project

## Complete MaixVision apps snapshot (2026-11-06)

The latest full local application snapshot is in **[apps/](apps/)**: all 199 files from six modules, with the original directory layout and bytes preserved.

- [YOLO click tracker](apps/yolo_click_tracker/README.md) - current gimbal tracking app, including continuous yaw and overhead tracking logic.
- [Motor joystick](apps/motor_joystick/README.md) - standalone touch/web motor control and calibration tools.
- [MicArray direction](apps/micarray_direction/README.md) - independent sound-direction module.
- [Sound-guided YOLO gimbal](apps/sound_yolo_gimbal/README.md) - sound-to-visual tracking integration.
- [Thermal and YOLO fusion](apps/thermal_yolo_fusion/README.md) - thermal alignment, hot-region tracking, and RGB detection.
- [Drone sensor demo](apps/drone_sensor_demo/README.md) - sensor-fusion strategy demo.

See the [publication record](docs/publications/apps-20261106.md) and [per-file SHA-256 manifest](docs/publications/apps-20261106-manifest.json). Earlier top-level application folders below are historical snapshots. Publishing files does not start hardware; read each app's startup behavior before running it.

This repository contains the MaixCAM 2 vision-tracking prototype, its system architecture, and a local developer workbench for device monitoring.

## Components

- [Tracker architecture](maixcam2_drone_tracker_architecture.md) — system boundaries, runtime flow, and staged development plan.
- [Person-tracking application](maixcam2_person_tracker/README.md) — Stage 1 YOLO person detection, target association, bearing estimation, and safe vision-only gimbal flow.
- [MicArray sound-direction display](maixcam2_micarray_direction/README.md) — independent MaixVision UART app with reusable 16×16 sound-map direction API; no gimbal commands.
- [Sound-guided YOLO gimbal tracking](maixcam2_sound_yolo_gimbal/README.md) — standalone MaixVision sound-to-slew-to-visual tracking, fixed mechanical zero, manual arming, and latched zero-loss protection; PC-tested, hardware validation pending.
- [Thermal160 + YOLO MaixVision app](maixcam2_thermal_yolo_fusion/README.md) — RGB detection, manual multi-distance thermal alignment, hot-region tracking, temperature overlay, and RTSP/JPEG output.
- [Developer workbench](workbench/README.md) — local monitoring dashboard, MJPEG stream proxy, telemetry schema, extension points, and setup instructions.
- `Daniel's code/` — existing ENGG2202 project files.

The workbench is a development monitor. Camera capture and inference remain on the MaixCAM 2; the Mac gateway displays the video and receives telemetry without taking ownership of the vision loop.
