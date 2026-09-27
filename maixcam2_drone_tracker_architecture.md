# MaixCAM 2 Drone-Tracking System Architecture

The approved architecture has been converted into an English implementation project at:

- [Project README](maixcam2_person_tracker/README.md)
- [Software architecture](maixcam2_person_tracker/docs/ARCHITECTURE.md)
- [Hardware bring-up procedure](maixcam2_person_tracker/docs/BRINGUP.md)

## System direction

The final system uses a MaixCAM 2, one camera, and a two-axis WHEELTEC F32C brushless gimbal. MaixCAM 2 owns perception, target-state estimation, loss recovery, and gimbal target generation. Each F32C motor retains responsibility for its internal position and speed loops.

The runtime chain is:

```text
Camera
  -> YOLO detector
  -> single-target association
  -> world-bearing Kalman filter
  -> TRACK / COAST / SEARCH state machine
  -> bounded position trajectory
  -> F32C direct multi-turn position commands
```

Stage 1 uses the COCO `person` class to validate the complete visual closed loop. The later drone model will implement the same `Detection` interface, allowing the estimator, controller, motor protocol, and telemetry modules to remain unchanged.

## Key design decisions

- Real-time control runs entirely on MaixCAM 2. The Mac is used only for development, monitoring, and log retrieval.
- Prediction is performed in installation-frame azimuth and elevation rather than image pixels.
- Short losses enter `COAST`; longer losses enter an expanding local `SEARCH`; failed local recovery clears the old identity and returns to full-frame `ACQUIRE`.
- F32C direct multi-turn position mode is used for frequent target updates and safer hold behavior after software stops sending commands.
- UART4 on MaixCAM 2 uses A21/A22 at 115200 8N1 with motor IDs 1 and 2.
- Motor and MaixCAM power paths are separately regulated and share signal ground.
- Gimbal output is disabled by default in software until axis signs, zero offsets, and mechanical limits are verified.

The implementation project documents contain the detailed module contracts, state transitions, source layout, safety behavior, calibration parameters, and Stage 1 acceptance criteria.
