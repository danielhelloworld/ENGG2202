"""Bundled by build.py with existing verified-source UART adapters."""
import time
import json
import math
import numpy as np
import cv2


def project(matrix, x, y):
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape not in ((2, 3), (3, 3)) or not np.isfinite(m).all():
        raise ValueError("Invalid calibration matrix")
    p = m @ np.array([x, y, 1.0])
    if len(p) == 3:
        if abs(p[2]) < 1e-9:
            return None
        p = p[:2]/p[2]
    return tuple(float(v) for v in p) if np.isfinite(p).all() and np.all((p >= 0) & (p <= 1)) else None


def run_board(config_path="/root/drone_sensor_demo/config.json"):
    from maix import app, camera, display, image, nn
    config = load_config(config_path)
    engine = Fusion(config)
    sensor = sound_api["SoundDirectionSensor"]()
    thermal = thermal_api["ThermalReader"](dict(thermal_api["CONFIG"]))
    cam = disp = model = None
    sound_ok = False
    try:
        # Explicit generic model fallback is displayed; it cannot confirm drones.
        model = getattr(nn, config["model_type"])(model=config["model_path"], dual_buff=False)
        iw, ih = model.input_width(), model.input_height()
        labels = [str(label).lower() for label in model.labels]
        has_drone = any(label in config["drone_labels"] for label in labels)
        cam = camera.Camera(iw, ih, image.Format.FMT_RGB888, fps=config["camera_fps"])
        disp = display.Display()
        thermal.start()
        try:
            sensor.open_uart()
            sound_ok = True
        except Exception as exc:
            print("SOUND OFFLINE", repr(exc))
        calibration_ok = config["calibration_resolution"] == [iw, ih]
        seq, last_log, last_camera = 0, 0, time.monotonic()
        while not app.need_exit():
            frame = cam.read(block_ms=200)
            if frame is None:
                if time.monotonic()-last_camera > 5:
                    raise RuntimeError("Camera stalled for 5 seconds")
                continue
            captured = time.monotonic()
            last_camera = captured
            objects = model.detect(frame, conf_th=.35, iou_th=.45)
            now = time.monotonic()
            seq += 1
            obs = []
            for idx, obj in enumerate(objects):
                label = str(model.labels[obj.class_id]).lower()
                if label in config["drone_labels"]:
                    obs.append(observation("vision", (obj.x+obj.w/2)/iw, (obj.y+obj.h/2)/ih,
                                           obj.score, captured, "%d:%d" % (seq, idx), label))
            tf, thermal_status, _ = thermal.snapshot()
            candidates = []
            if tf is not None and tf["valid_temperature"] and 0 <= now-tf["received_at"] <= config["max_age"]["thermal"]:
                candidates = thermal_candidates(tf["pixels"])
                if calibration_ok and config["thermal_to_rgb"] is not None:
                    for idx, item in enumerate(candidates):
                        p = project(config["thermal_to_rgb"], item["x"], item["y"])
                        if p:
                            obs.append(observation("thermal", *p, item["score"], tf["received_at"],
                                                   "%s:%d" % (tf["received_at"], idx)))
            sound_status = "OFFLINE"
            if sound_ok:
                try:
                    sensor.poll(timeout_ms=2)
                    s = sensor.snapshot()
                    sound_status = s.get("reason", "OK")
                    bounds = config["sound_calibration_bounds"]
                    if s["valid"] and s["vector_xy"] and calibration_ok and config["sound_to_rgb"] is not None and bounds:
                        x, y = s["vector_xy"]
                        if bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3]:
                            p = project(config["sound_to_rgb"], x, y)
                            if p:
                                stamp = time.monotonic()-s["age_seconds"]
                                obs.append(observation("sound", *p, s["quality"], stamp, s["frames"]))
                except Exception as exc:
                    sound_status = "ERROR: " + str(exc)
            now = time.monotonic()
            tracks, rejected = engine.update(obs, now)
            rgb = image.image2cv(frame, ensure_bgr=False, copy=True)
            title = "LIVE / UAV model labels present" if has_drone else "LIVE / MODEL HAS NO UAV LABEL - candidates only"
            details = "T:%s (%d blobs) S:%s CAL:%s" % (thermal_status, len(candidates), sound_status,
                        "CHECK MATRICES" if calibration_ok else "UNSET - auxiliary fusion disabled")
            # Missing mappings must be visible even when resolution happens to match.
            if not config["thermal_to_rgb"] or not config["sound_to_rgb"]:
                details += " / MAPPING MISSING"
            fresh_thermal = tf is not None and 0 <= now-tf["received_at"] <= config["max_age"]["thermal"]
            canvas = render(tracks, obs, config, title, details,
                            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), tf["pixels"] if fresh_thermal else None)
            disp.show(image.cv2image(cv2.resize(canvas, (640, 480)), bgr=True, copy=True))
            if now-last_log >= 1:
                print("DRONE_FUSION " + json.dumps(dict(mode="live", weights=config["weights"],
                      drone_model_labels=has_drone, tracks=tracks, rejected=rejected,
                      thermal_candidates=candidates, sound_status=sound_status, calibration=details)))
                last_log = now
    finally:
        for resource in (sensor, thermal, cam):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    print("CLEANUP", repr(exc))
        cam = disp = model = None


if __name__ == "__main__":
    run_board()
