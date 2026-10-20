# Public dataset sources

## Halmstad multi-sensor drone detection dataset

- Official source: https://github.com/DroneDetectionThesis/Drone-detection-dataset
- License: CC0-1.0
- Citation: F. Svanstrom, C. Englund and F. Alonso-Fernandez,
  "A Dataset for Multi-Sensor Drone Detection," Data in Brief, 2021.
- Used modalities: visible-light videos only; no IR or audio.
- Source classes: Drone, Bird, Airplane and Helicopter.
- Training mapping: Drone becomes YOLO class `0`; Bird, Airplane and Helicopter
  are retained as hard-negative images with empty YOLO labels.
- Sampling unit: complete source video. Frames from one source video never
  cross train/validation/test/calibration splits.
- Downloaded visible-light subset: all 285 official RGB videos and their 285
  annotation files (about 127 MiB locally).
- Extraction: every 15th frame (about 2 fps), producing 6,082 images.
- Split seed: `2202`; assignment is recorded locally in
  `dataset/halmstad_split.json`.
- Session split: 200 train, 43 validation, 28 internal test and 14 calibration
  videos, stratified by the four source classes.
- Storage policy: selected files are processed one video at a time. Source
  videos may be deleted after conversion, validation and manifest recording.

Initial conversion pilot:

- `V_DRONE_046`
- `V_DRONE_048`
- `V_BIRD_005`
- `V_AIRPLANE_048`

The pilot was used only to verify annotation decoding and is not a separate
source in the final split.
