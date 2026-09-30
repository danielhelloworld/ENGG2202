"""Appended to the fusion bundle by build_hardware_probe.py; not a standalone module."""

def run_hardware_probe():
    import platform
    import traceback
    import maix
    from maix import app, camera, display, image, nn
    from maix import sys as maix_sys
    from maix.peripheral import uart

    logging.basicConfig(level=logging.INFO)
    report = {"test": "MaixCAM2 actual hardware probe", "synthetic": False,
              "python": platform.python_version(),
              "maixpy": getattr(maix, "__version__", "unknown"),
              "board": maix_sys.device_name(), "checks": {}, "errors": [],
              "frames": 0, "detections": 0, "inferences": 0,
              "thermal_updates": 0, "valid_temperature_frames": 0,
              "rtsp_client_seen": False, "rtsp_decode_verified": False}
    def emit(event, **details):
        print("MAIX_PROBE " + json.dumps(dict(event=event, **details)), flush=True)
    def failure(stage, exc):
        report["checks"][stage] = "FAIL"
        report["errors"].append({"stage": stage, "error": str(exc)})
        emit("ERROR", stage=stage, error=str(exc))
        traceback.print_exc()

    cam = disp = detector = reader = server = None
    start = time.monotonic()
    try:
        emit("ENVIRONMENT", board=report["board"], python=report["python"],
             maixpy=report["maixpy"], numpy=np.__version__, opencv=cv2.__version__,
             uart_devices=list(uart.list_devices()))
        if report["board"] != "MaixCAM2":
            raise RuntimeError("Wrong board: expected MaixCAM2; GPIO/UART test not started")
        report["checks"]["environment"] = "PASS"
        c = dict(CONFIG)
        validate_config(c)
        iw, ih, fmt = 640, 480, image.Format.FMT_RGB888
        try:
            path, kind = select_model(c)
            emit("MODEL_LOADING", path=path, kind=kind)
            detector = getattr(nn, kind)(model=path, dual_buff=False)
            iw, ih, fmt = detector.input_width(), detector.input_height(), detector.input_format()
            report["model"] = path
            report["checks"]["model_load"] = "PASS"
        except Exception as exc:
            failure("model_load", exc)

        cam = camera.Camera(iw, ih, fmt, fps=c["camera_fps"])
        report["checks"]["camera_open"] = "PASS"
        try:
            disp = display.Display()
        except Exception as exc:
            failure("display_open", exc)
        alignment = Alignment(c, (iw, ih), c.get("alignment_path"))
        ui = TouchUI(alignment)
        reader = ThermalReader(c)
        reader.start()
        try:
            server = JpegRtspServer(c["rtsp_host"], c["rtsp_port"], c["stream_fps"],
                                    c["jpeg_quality"], c["max_clients"])
            report["checks"]["rtsp_listen"] = "PASS"
            emit("RTSP_READY", urls=rtsp_addresses(server.port), duration_seconds=90)
        except Exception as exc:
            failure("rtsp_listen", exc)

        preview_start = last_log = time.monotonic()
        while not app.need_exit() and time.monotonic() - preview_start < 90:
            frame_start = time.monotonic()
            frame = cam.read()
            visible_at = time.monotonic()
            if frame is None:
                time.sleep(.01)
                continue
            thermal, status, count = reader.snapshot()
            detections = []
            if detector is not None:
                try:
                    objects = detector.detect(frame, conf_th=c["confidence"], iou_th=c["iou"])
                    report["inferences"] += 1
                    report["detections"] += len(objects)
                    for obj in sorted(objects, key=lambda item: item.score, reverse=True)[:c["max_objects"]]:
                        label = str(detector.labels[obj.class_id]).encode("ascii", "replace").decode("ascii")
                        detections.append({"box": (obj.x, obj.y, obj.w, obj.h),
                                           "score": obj.score, "label": label})
                except Exception as exc:
                    failure("inference", exc)
                    detector = None
            rgb = cv2.cvtColor(image.image2cv(frame, ensure_bgr=True, copy=False), cv2.COLOR_BGR2RGB)
            now = time.monotonic()
            canvas, _, thermal_status = compose(rgb, detections, thermal, alignment, now, visible_at)
            ui.draw(canvas, "HARDWARE PROBE | %ds left | %s" %
                    (max(0, int(90 - (now - preview_start))), status))
            if disp is not None:
                disp.show(image.cv2image(canvas, bgr=False, copy=True), fit=image.Fit.FIT_CONTAIN)
                report["checks"]["display_show_call"] = "PASS"
            if server is not None:
                server.submit(canvas, now)
                report["rtsp_client_seen"] |= server.client_count() > 0
                if server.error:
                    report["rtsp_error"] = str(server.error)
            report["frames"] += 1
            report["thermal_updates"] = count
            report["thermal_status"] = thermal_status
            if thermal is not None and thermal["valid_temperature"]:
                report["valid_temperature_frames"] += 1
                report["last_thermal"] = {"lo_c": thermal["lo"] / 10,
                                          "hi_c": thermal["hi"] / 10,
                                          "age_seconds": round(now - thermal["received_at"], 3)}
            if now - last_log >= 5:
                emit("PROGRESS", frames=report["frames"], inferences=report["inferences"],
                     detections=report["detections"], thermal_updates=count,
                     thermal_status=thermal_status, reader_status=status,
                     fps=round(report["frames"] / (now - preview_start), 2),
                     rtsp_clients=server.client_count() if server else 0)
                last_log = now
            if c["loop_fps"]:
                time.sleep(max(0, 1 / c["loop_fps"] - (time.monotonic() - frame_start)))
        report["checks"]["camera_frames"] = "PASS" if report["frames"] else "FAIL"
        report["checks"]["inference"] = report["checks"].get("inference", "PASS" if report["inferences"] else "NOT_RUN")
        report["checks"]["thermal_frames"] = "PASS" if report["thermal_updates"] else "FAIL"
        report["checks"]["thermal_temperature_data"] = "PASS" if report["valid_temperature_frames"] else "FAIL"
    except Exception as exc:
        failure("hardware_probe", exc)
    finally:
        for resource in (reader, server, cam, disp):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    failure("cleanup_" + type(resource).__name__, exc)
        report["elapsed_seconds"] = round(time.monotonic() - start, 2)
        report["note"] = "Temperature validity is not calibration accuracy; display.show is not visual confirmation; RTSP requires external decode verification."
        emit("RESULT", report=report)


if __name__ == "__main__":
    run_hardware_probe()
