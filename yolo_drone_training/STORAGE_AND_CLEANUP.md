# Storage and cleanup record

There is no fixed storage limit for `yolo_drone_training`. Report current use
with:

```bash
python scripts/check_storage.py
```

## Working policy

- Download only selected RGB video/annotation pairs, never an entire large dataset.
- Process one source video at a time.
- After extracted images and YOLO labels pass validation, delete the downloaded
  source video and source annotation instead of keeping duplicate data.
- Sample by complete video session and cap the prepared dataset before training.
- Keep enough free disk space for the dataset, training outputs and temporary
  files used by the operating system.

## Keep until the project is complete

- final train/validation/test images and YOLO labels;
- `best.pt`, final evaluation plots and metrics;
- final ONNX, `.mud`, and companion `.axmodel` files;
- dataset source, license, citation and split manifests;
- the Python virtual environment while training or export may be repeated.

## Safe cleanup after a verified training/deployment run

- `raw/sources/halmstad_visible/` (about 127 MiB) after the extracted dataset,
  manifests and final model are backed up and verified;
- `dataset/incoming/halmstad_pilot/` and its preview after final dataset QA;
- `runs/drone_yolo11n_640_smoke/`, the interrupted all-negative fraction test;
- downloaded ZIP files and original source videos already converted and checked;
- duplicated files in `dataset/incoming/`;
- smoke-test and failed training run folders;
- `last.pt` and superseded checkpoints after `best.pt` is backed up;
- obsolete ONNX variants after the selected MaixCAM2 package is verified;
- calibration ZIP after the converted model package is verified (source image
  manifest is retained so it can be recreated).

Never delete the final model package, `best.pt`, metrics, manifests, or the only
copy of annotations automatically. Review the cleanup list with the user first.
