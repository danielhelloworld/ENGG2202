# MaixCAM 2 Workbench

The workbench is a local developer console for a MaixCAM 2 vision project. It combines the device MJPEG camera feed with structured telemetry and process health on one page. The browser talks only to the workbench gateway; the gateway proxies the device video stream and accepts telemetry snapshots.

## Features

- Overview page for camera, detections, tracking state, gimbal feedback, and running file.
- Live camera page with the annotated MJPEG feed.
- Data explorer organized into vision, tracking, gimbal, and system categories.
- Runtime view for the active entry point, process ID, uptime, model, and device services.
- Device connection settings for IP address and video-stream port.
- Extensible JSON telemetry schema with namespaced `extensions` data.
- Server-sent events for browser updates; no third-party JavaScript packages are required.

## Architecture

```text
MaixCAM 2 tracker
  ├── MjpegStreamer ── HTTP MJPEG :8080 ───────────────┐
  └── WorkbenchPublisher ── POST /api/telemetry ───────┤
                                                       ▼
Mac Workbench gateway :8760 ── same-origin MJPEG proxy + SSE ── Browser UI
```

The MaixCAM 2 tracker remains responsible for camera capture and inference. `WorkbenchPublisher` sends a small latest-value snapshot on a background thread so a disconnected workbench does not block the vision loop. The Mac gateway keeps the newest snapshot in memory and exposes a stable browser API. The web UI renders registered categories and automatically includes unregistered metrics under extension namespaces.

## Start the workbench

The MaixCAM 2 USB network commonly gives the device `10.177.5.1` and the Mac `10.177.5.100`. Confirm the Mac address in Network settings before starting the gateway. Replace it below if the USB interface uses another address.

```bash
cd ENGG2202-github
python3 workbench/server.py --host 10.177.5.100 --port 8760 --device-ip 10.177.5.1 --video-port 8080
```

Open `http://10.177.5.100:8760/` in a browser. The gateway must bind to an address reachable by both the browser and MaixCAM 2. Bind to the Mac's USB-network interface only. The telemetry endpoint has no authentication and should not be exposed on a shared network or the public internet.

## Connect the MaixCAM 2 tracker

Set `WorkbenchConfig.endpoint` in `maixcam2_person_tracker/config.py` to the Mac USB-network address shown in the workbench's **Connection Settings** page:

```python
class WorkbenchConfig:
    endpoint = "http://10.177.5.100:8760/api/telemetry"
```

In the current code, `WorkbenchConfig` is a frozen dataclass. Set the `endpoint` field in its declaration, or provide `MAIX_WORKBENCH_URL` in the MaixCAM 2 process environment. Then start `maixcam2_person_tracker/main.py` on the device. The tracker starts its annotated MJPEG endpoint on port `8080` and publishes camera, person-detection, tracking, gimbal, model, and process metrics to the workbench.

The initial gimbal setting remains disabled. Starting the dashboard does not start or stop the tracker and does not send motor commands.

## Telemetry schema

Each `POST /api/telemetry` payload is a JSON object with a `schema_version` and category objects:

```json
{
  "schema_version": 1,
  "runtime": {"running": true, "script": "main.py", "pid": 123, "uptime_seconds": 12.5, "model_path": "/root/models/yolo11n.mud"},
  "camera": {"available": true, "width": 640, "height": 480, "fps": 15.0},
  "vision": {"person_count": 1, "selected_label": "person", "confidence": 0.87, "center_x": 320, "center_y": 240},
  "tracking": {"mode": "TRACK", "azimuth_deg": 0.2, "elevation_deg": -1.1, "azimuth_rate_deg_s": 0.0, "elevation_rate_deg_s": 0.3},
  "gimbal": {"enabled": false, "pan_deg": 0.0, "tilt_deg": 0.0, "pan_valid": true, "tilt_valid": true},
  "system": {"camera_available": true, "npu_ready": true},
  "extensions": {"depth": {"meters": 1.4}}
}
```

Use `extensions.<module-name>` for new features that do not belong to an existing category. The Data Explorer adds each namespace as a tab and lists its metrics without requiring a schema migration. For richer labels, units, charts, or controls, extend `CATEGORY_DEFS` and `METRICS` in `web/app.js`, then add the matching UI module in `web/`.

## HTTP API

| Route | Method | Purpose |
|---|---|---|
| `/api/health` | GET | Gateway health check |
| `/api/state` | GET | Latest telemetry snapshot and connection age |
| `/api/events` | GET | Server-sent state updates |
| `/api/config` | GET/POST | Read or update device IP and stream port |
| `/api/telemetry` | POST | Ingest the device telemetry snapshot |
| `/api/camera/stream` | GET | Same-origin proxy for the device MJPEG stream |

Telemetry is considered connected while a snapshot has arrived within the last three seconds. The latest snapshot is kept in memory; the gateway does not persist telemetry or camera video to disk.
