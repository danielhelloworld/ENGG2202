# Drone YOLO11n 640×480 v1 — board deployment

This package is a **pre-board-validation candidate** for MaixCAM2. Class mapping: `0 = drone` (UAV and drone are the same class). The model requires image input width 640 and height 480. Upload the `.mud` and **all** `.axmodel` files from `board_model/` into `/root/models/` on the board. The application must load the `.mud`, not an `.axmodel` or the training `.pt` file.

The board package contains `model_10515.mud`, `model_10515_npu.axmodel`, and `model_10515_vnpu.axmodel`. Update the Fusion and F32C gimbal programs' `model_path` to `/root/models/model_10515.mud` after copying the board files. This release does not change either teammate program. Validate camera preprocessing, bounding boxes, confidence thresholds, false alarms, missed UAVs, frame rate, and gimbal motion on MaxiCAM2 and F32C hardware before claiming deployment success.

The 100 calibration images are for INT8 conversion and are **not** published as GitHub Release assets. The model, calibration ZIP, and complete independent test output live in the versioned USB delivery folder. `SHA256SUMS.txt` in that folder records the exact artifact hashes.

Independent test: 889 images / 493 drone instances, precision 0.838825, recall 0.758621, mAP50 0.718590, mAP50–95 0.290373, using `best.pt` at 640×640 and confidence 0.25. This does not measure the 640×480 converted board model.

