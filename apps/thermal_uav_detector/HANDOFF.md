# Integration handoff

## Source checked

Read from `feature/maixcam2-micarray-direction`: `apps/drone_sensor_demo/{README.md,core.py,board_runtime.py,build.py}` and `apps/thermal_yolo_fusion/src/{config.py,thermal.py,hot_tracker.py}`. The requested `MaixCam/AGENTS.md`, `MaixCam/MEMORY.md`, and `MaixCam/README.md` were absent at the inspected branch. Local project `AGENTS.md` was read. The old `hot_tracker.py` uses a 25 °C and 5-pixel gate; this module does not. No source outside this new directory was changed.

## Main task merge

1. This directory is delivered at `apps/thermal_uav_detector/` on an isolated branch. Merge it into the target branch without editing the other apps.
2. The fusion owner obtains `tf, status, count = thermal.snapshot()` once per loop, makes a shallow copy and adds `reader_count=count`, then calls `result = detector.update(tf, now)`. Do not start another ThermalReader.
3. After independent calibration checks, wrap the existing `config['thermal_to_rgb']` in metadata required by `adapter.observations`. The existing matrix contract is raw unflipped thermal pixels to normalized RGB. Existing `calibration_resolution` must match the model input dimensions. Also verify orientation and distance interval, and check sample projections against independent paired targets. A display alignment matrix is not interchangeable.
4. Pass adapter output to `Fusion.update`. Its fields match `core.observation`; extra `thermal_track_id` is metadata only. Never create `VISUAL_CONFIRMED` from thermal output. Keep visual/sound/thermal weights at the fusion owner's defaults 0.70/0.15/0.15 and leave missing visual weight unallocated.

Example (host owns reader):

```python
from detector import Detector
from adapter import observations
engine = Detector()
# inside existing loop, using same monotonic clock as ThermalReader:
tf, thermal_status, count = thermal.snapshot()
tf = dict(tf, reader_count=count) if tf is not None else None
result = engine.update(tf, now)
calibration = dict(thermal_to_rgb=config['thermal_to_rgb'],
    resolution=[iw, ih], thermal_size=[160, 120], raw_unflipped=True,
    fixed_camera=True, verified=independent_calibration_passed,
    distance_range_m=[measured_min_m, measured_max_m])
thermal_obs = observations(result, calibration, (iw, ih), measured_distance_m)
# append thermal_obs to the existing observation list
```

The metadata booleans must come from actual checks; this example does not assert that a calibration exists. `reader_count` is important because repeated snapshots of a mailbox frame must not create hits. `received_at` remains the evidence timestamp; the thermal reader currently does not expose a separate acquisition timestamp.

## Verification and limitations

See `TEST_REPORT.md`. No device or real marked Thermal160 recording was available. Fixed hot pixels can still look like persistent hovering targets if not masked. Cloud/building edges, moving backgrounds and target crossings can produce false candidates or ambiguous associations. Global movement detector only suppresses broad changes after they occur; it is not registration. The 160×120 resolution limits distant targets and exact localization. No distance-specific detection rate is claimed.

Next: collect complete fixed-camera sessions at each distance with positives and negatives; label raw-frame boxes and preserve timestamps. Split by whole capture/session. Match detections to labels by one-to-one IoU or center-distance tolerance chosen before evaluation. Report detection/omission rate, false positives per minute, localization error, track interruptions and latency for each distance. Then separately evaluate moving-camera data with pose/registration.
