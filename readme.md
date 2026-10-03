# ENGG2202 MaixCAM 2 Project

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
