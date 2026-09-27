# ENGG2202 MaixCAM 2 Project

This repository contains the MaixCAM 2 vision-tracking prototype, its system architecture, and a local developer workbench for device monitoring.

## Components

- [Tracker architecture](maixcam2_drone_tracker_architecture.md) — system boundaries, runtime flow, and staged development plan.
- [Person-tracking application](maixcam2_person_tracker/README.md) — Stage 1 YOLO person detection, target association, bearing estimation, and safe vision-only gimbal flow.
- [Developer workbench](workbench/README.md) — local monitoring dashboard, MJPEG stream proxy, telemetry schema, extension points, and setup instructions.
- `Daniel's code/` — existing ENGG2202 project files.

The workbench is a development monitor. Camera capture and inference remain on the MaixCAM 2; the Mac gateway displays the video and receives telemetry without taking ownership of the vision loop.
