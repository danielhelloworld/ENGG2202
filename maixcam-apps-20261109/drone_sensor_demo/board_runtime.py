"""Three-sensor board entry with the original thermal overlay calibration UI."""
import time
import json
import numpy as np
import cv2


def project(matrix, x, y):
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape not in ((2, 3), (3, 3)) or not np.isfinite(m).all():
        raise ValueError('Invalid calibration matrix')
    p = m @ np.array([x, y, 1.0])
    if len(p) == 3:
        if abs(p[2]) < 1e-9:
            return None
        p = p[:2]/p[2]
    return tuple(float(v) for v in p) if np.isfinite(p).all() and np.all((p >= 0) & (p <= 1)) else None


def run_board(config_path='/root/drone_sensor_demo/config.json'):
    from maix import app, camera, display, image, nn, touchscreen
    config = load_config(config_path)
    engine = Fusion(config)
    sensor = thermal = cam = disp = model = ts = None
    sound_ok = False
    try:
        model = getattr(nn, config['model_type'])(model=config['model_path'], dual_buff=False)
        iw, ih = model.input_width(), model.input_height()
        labels = [str(label).lower() for label in model.labels]
        has_drone = any(label in config['drone_labels'] for label in labels)
        alignment = make_alignment(config, (iw, ih))
        ui = thermal_api['TouchUI'](alignment)
        integration = ThermalIntegration(config, alignment)
        geometry = integration.geometry_key()
        cam = camera.Camera(iw, ih, image.Format.FMT_RGB888, fps=config['camera_fps'])
        disp = display.Display()
        try:
            ts = touchscreen.TouchScreen()
        except Exception as exc:
            print('TOUCH OFFLINE', repr(exc))
        thermal = thermal_api['ThermalReader'](alignment.config)
        thermal.start()
        sensor = sound_api['SoundDirectionSensor']()
        try:
            sensor.open_uart()
            sound_ok = True
        except Exception as exc:
            print('SOUND OFFLINE', repr(exc))
        sound_calibration_ok = config['calibration_resolution'] == [iw, ih]
        seq, last_log, last_camera = 0, 0, time.monotonic()
        paused_before = False
        while not app.need_exit():
            if ts is not None:
                tx, ty, pressed = ts.read()
                dx, dy, dw, dh, _, _ = thermal_api['letterbox_geometry'](
                    alignment.config['width'], alignment.config['height'], disp.width(), disp.height())
                tx = (tx-dx)*alignment.config['width']/dw
                ty = (ty-dy)*alignment.config['height']/dh
                if ui.touch(tx, ty, pressed):
                    app.set_exit_flag(True)
                    break
            paused = ui.registration.active or ui.calibrating
            new_geometry = integration.geometry_key()
            if new_geometry != geometry or paused != paused_before:
                engine = Fusion(config)
                geometry, paused_before = new_geometry, paused
            frame = cam.read(block_ms=200)
            if frame is None:
                if time.monotonic()-last_camera > 5:
                    raise RuntimeError('Camera stalled for 5 seconds')
                continue
            captured = time.monotonic()
            last_camera = captured
            objects = [] if paused else model.detect(frame, conf_th=.35, iou_th=.45)
            seq += 1
            obs, detections = [], []
            for idx, obj in enumerate(objects):
                label = str(model.labels[obj.class_id]).lower()
                detections.append(dict(box=[obj.x, obj.y, obj.w, obj.h], label=label,
                                       score=obj.score, captured_at=captured))
                if label in config['drone_labels']:
                    obs.append(observation('vision', (obj.x+obj.w/2)/iw, (obj.y+obj.h/2)/ih,
                                           obj.score, captured, '%d:%d' % (seq, idx), label))
            # Exactly one mailbox snapshot per RGB loop; no second UART owner.
            tf, reader_status, count = thermal.snapshot()
            now = time.monotonic()
            result, thermal_obs, calibration_status = integration.update(tf, count, now, paused)
            obs.extend(thermal_obs)
            sound_status = 'OFFLINE'
            if sound_ok:
                try:
                    sensor.poll(timeout_ms=2)
                    s = sensor.snapshot()
                    sound_status = s.get('reason', 'OK')
                    bounds = config['sound_calibration_bounds']
                    if (not paused and s['valid'] and s['vector_xy'] and sound_calibration_ok
                            and config['sound_to_rgb'] is not None and bounds):
                        x, y = s['vector_xy']
                        if bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3]:
                            p = project(config['sound_to_rgb'], x, y)
                            if p:
                                stamp = time.monotonic()-s['age_seconds']
                                obs.append(observation('sound', *p, s['quality'], stamp, s['frames']))
                except Exception as exc:
                    sound_status = 'ERROR: ' + str(exc)
            now = time.monotonic()
            tracks, rejected = engine.update(obs, now)
            if frame.format() != image.Format.FMT_RGB888:
                frame = frame.to_format(image.Format.FMT_RGB888)
            rgb = image.image2cv(frame, ensure_bgr=False, copy=True)
            del frame
            ui.registration.observe(rgb, tf, captured, now)
            canvas, _, thermal_status = thermal_api['compose'](rgb, detections, tf, alignment,
                now, captured, ui.mode, ui.alpha, ui.show_temperatures, ui.show_heat)
            integration.draw(canvas, now, captured)
            draw_fusion_tracks(canvas, tracks, alignment)
            title = 'UAV MODEL' if has_drone else 'MODEL HAS NO UAV LABEL'
            thermal_api['draw_text_rgb'](canvas, '%s | T:%s S:%s' %
                (title, result['status'], sound_status), 8, 104, scale=.40)
            weights = config['weights']
            thermal_api['draw_text_rgb'](canvas, 'V/S/T %.0f/%.0f/%.0f | %s' %
                (100*weights['vision'], 100*weights['sound'], 100*weights['thermal'], calibration_status),
                8, alignment.config['height']-116, scale=.40)
            ui.draw(canvas, '%s | %s' % (reader_status, calibration_status))
            if ui.registration.active:
                canvas = ui.registration.render()
            disp.show(image.cv2image(canvas, bgr=False, copy=True), fit=image.Fit.FIT_CONTAIN)
            if now-last_log >= 1:
                print('DRONE_FUSION ' + json.dumps(dict(mode='live', weights=weights,
                    drone_model_labels=has_drone, tracks=tracks, rejected=rejected,
                    thermal_candidates=result['candidates'], thermal_status=result['status'],
                    thermal_fusion_observations=len(thermal_obs), sound_status=sound_status,
                    calibration=calibration_status, distance_m=alignment.distance)))
                last_log = now
    finally:
        for resource in (sensor, thermal, cam):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    print('CLEANUP', repr(exc))
        cam = disp = model = ts = None


if __name__ == '__main__':
    run_board()
