# Thermal UAV detector: fixed-camera baseline

This is a **candidate detector**, not a drone classifier or a probability estimate. It consumes decoded Thermal160/TN160 telemetry30 frames from the existing `ThermalReader`; it never opens UART2 itself. Coordinates are original, unflipped 160×120 pixels. No motor or camera control is present. The 5/10/30/50/100 m goals are unverified.

## PC replay

Python 3 standard library only:

```sh
cd apps/thermal_uav_detector
python3 -m unittest discover -s tests -v
python3 replay.py --synthetic --output synthetic_results.jsonl
python3 replay.py --input capture.jsonl --output replay_results.jsonl
```

JSONL input: one object per line with `frame_id` or `reader_count`, `received_at` in monotonic seconds, `valid_temperature` boolean, `lo` and `hi` signed tenths of °C as provided by telemetry30, and `pixels` as 120 rows of 160 integer values in 0–255. `received_at` must share the replay clock; replay calls `update(frame, frame['received_at'])`. This file contains decoded raw arrays, not pseudocolor screenshots. The synthetic generator is explicitly labeled `SYNTHETIC` in every output row.

## Detection and state

Three clipped-window center-surround responses (radii 2, 5, 11) preserve a one-pixel peak and larger hot blocks. A frame-adaptive median/MAD threshold plus a minimum gray-level contrast extracts connected regions. Area 1 is allowed. Very large or elongated edge regions are rejected. `quality_score` is a bounded heuristic from local contrast and shape; it is **not** accuracy or drone probability. `local_contrast` is raw gray-level difference, not °C. Temperature is intentionally not displayed here. If another UI displays temperature, use only valid `lo + (hi-lo)*min(pixel/254,1)` in tenths of °C, matching the existing protocol.

Tracks require three new valid frame hits, hold for up to two new-frame misses and at most 0.8 s after last detection. Nearest predicted position plus area-ratio gate handles short gaps. A close competing match is rejected. Held/predicted tracks have `detected_this_frame=false`; they never produce adapter evidence. Duplicate, stale, future, invalid-temperature and invalid-pixel frames return explicit status and no candidates. Motion check measures the fraction of large changes after removing a global brightness offset. Broad changes set `MOTION_UNCERTAIN` for three frames; no motion compensation or arbitrary gimbal support is claimed. The fixed-camera mode must be used for fusion.

`bad_pixels` is a list of `[x,y]` **raw sensor coordinates**. Default is empty. The synthetic demo masks `[120,20]`, deliberately injected as a bad pixel. A real mask must come from repeated dark/flat-field captures and be documented per device. Masking a pixel can hide a real distant point target there.

## Board entry

With a local checkout of the named ENGG2202 branch, run:

```sh
python3 apps/thermal_uav_detector/build.py --repo /path/to/ENGG2202
```

This generates `board_main.py` plus upstream Sipeed attribution/license from `apps/thermal_yolo_fusion`. Open `board_main.py` in MaixVision. **Before running**, check that no other application owns UART2; the entry opens only the existing thermal reader. MaixPy, numpy, and cv2 follow the existing reader's dependencies. Board execution has not been performed here. The separate `maixvision.py` exposes `run(reader)` for a host that already owns a `ThermalReader` and must not open a second UART reader.

## Fusion adapter

`observations(result, calibration, rgb_resolution, distance_m)` emits only confirmed, real detections. Calibration must contain `thermal_to_rgb` (3×3 raw thermal to normalized RGB), matching `resolution`, `thermal_size=[160,120]`, `raw_unflipped=true`, `fixed_camera=true`, `verified=true`, and an independently checked `distance_range_m`. Missing/invalid metadata returns `[]`. The adapter emits thermal-local ID as metadata only; fusion owns its own track IDs. See `HANDOFF.md`.
