# YOLO11 UAV detector training

This directory trains the visible-light detector consumed by
`maixcam2_sound_yolo_gimbal`. It deliberately contains no dataset or model
weights. Keep the first model single-class (`drone`, class ID `0`); use birds,
kites, aircraft, glare and empty sky as labelled-negative images.

`python scripts/check_storage.py` can be used to report current disk use. It
does not enforce a fixed storage limit. See `STORAGE_AND_CLEANUP.md` for files
that can be removed after a verified training and deployment run.

## 1. Create the environment

Python 3.11 is recommended by the MaixPy training documentation. On macOS:

```bash
cd yolo_drone_training
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-macos.txt
```

On a Windows computer with an NVIDIA GPU, open PowerShell and first run
`nvidia-smi`. Create a Windows environment (do not copy the macOS `.venv`),
install the CUDA-enabled PyTorch build selected for that computer at
<https://pytorch.org/get-started/locally/>, and then install the project
requirements:

```powershell
cd D:\ENGG2202\yolo_drone_training
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
# Run the CUDA PyTorch command produced by the official selector first.
python -m pip install -r requirements.txt
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'No CUDA GPU')"
```

The CUDA check must print `True` before a long GPU training run. If PowerShell
blocks activation, run `Set-ExecutionPolicy -Scope Process Bypass` for that
PowerShell session and activate the environment again.

If Python 3.11 is unavailable, install it first. Do not install the training
stack into MaixCAM2; training runs on the development computer or a GPU server.

## 2. Download and prepare the public data

Create the local folders, download the Halmstad visible-light subset, and
convert it with deterministic, session-level splits:

```bash
python scripts/prepare_dataset.py
python scripts/download_halmstad_visible.py
python scripts/import_halmstad.py \
  --source raw/sources/halmstad_visible \
  --dataset dataset \
  --stride 15
```

The current split reserves 70% of sessions for training, 15% for validation,
10% for internal testing and 5% for converter calibration. Drone frames become
class `0`; bird, airplane and helicopter frames are labelled negatives.

Split by complete recording session. Recommended starting target:

- train: at least 1,500 varied UAV images plus negative images;
- validation: at least 300 images from different sessions;
- test: at least 300 untouched images from different sessions;
- include many small targets and real deployment backgrounds;
- do not copy neighbouring video frames across splits.

Run the checker before every training run:

```bash
python scripts/check_dataset.py
```

Optionally inspect a contact sheet before training:

```bash
python scripts/preview_yolo.py \
  --dataset dataset --split train \
  --output artifacts/halmstad_train_preview.jpg
```

## 3. Train

The default uses Apple Metal (`mps`), batch size 8 and YOLO11n at 640 pixels
because small UAVs need more spatial detail than the 320-pixel quick-start
model:

```bash
python scripts/train.py
```

Before the long run, run one meaningful end-to-end epoch over the complete
training split at a smaller resolution:

```bash
python scripts/train.py \
  --epochs 1 --imgsz 320 --batch 16 \
  --name drone_yolo11n_320_smoke
```

The project applies narrow compatibility fixes during validation because
PyTorch 2.2 on MPS rejects Ultralytics' disabled autocast context and can
misread an MPS scalar used as a tensor dimension. The fixes do not change the
loss formula or model calculations and are not applied to CUDA or CPU runs.

Do not use Ultralytics' `fraction` option with this dataset: filenames are
grouped by source class, so taking the first fraction can select only negative
examples.

If macOS runs out of memory, retry with `--batch 4`. On an NVIDIA machine use
`--device 0 --batch -1`; on CPU use `--device cpu` (slow). The best weights are
normally written to:

```text
runs/drone_yolo11n_640/weights/best.pt
```

For a Windows/NVIDIA smoke test followed by the full run:

```powershell
python scripts\check_dataset.py --dataset dataset
python scripts\train.py --epochs 1 --imgsz 640 --batch 8 --device 0 --workers 4 --name drone_yolo11n_640_gpu_smoke
python scripts\train.py --epochs 100 --imgsz 640 --batch -1 --device 0 --workers 4 --name drone_yolo11n_640_gpu
```

Use `--project E:\ENGG2202_runs` if checkpoints should be written directly to
an external drive. Resume an interrupted run from its latest checkpoint:

```powershell
python scripts\train.py --resume-from E:\ENGG2202_runs\drone_yolo11n_640_gpu\weights\last.pt
```

## 4. Evaluate the untouched test set

```bash
python scripts/evaluate.py \
  --model runs/drone_yolo11n_640/weights/best.pt
```

Do not approve the model from mAP alone. Inspect false positives on birds,
kites, aircraft and background details, plus missed tiny/blurred UAVs.

## 5. Export fixed-shape ONNX

The MaixCAM2 converter requires a fixed input shape, ONNX opset 17 and no
dynamic axes. This project exports 640×480 for small-target detection:

```bash
python scripts/export_maixcam2.py \
  --model runs/drone_yolo11n_640/weights/best.pt \
  --height 480 --width 640
```

For a faster first board test, use `--height 240 --width 320` and compare its
small-UAV recall against 640×480.

## 6. Prepare INT8 calibration images

Put 20--100 separate, representative real-camera scenes in
`dataset/calibration/images/`, including targets and negatives. These images do
not need labels and must not come from the held-out test split. Then run:

```bash
python scripts/make_calibration_zip.py --count 100
```

Submit the ONNX and `artifacts/drone_calibration_images.zip` to the official
Maix model converter, selecting **MaixCAM2**, **YOLO11**, and the same width and
height used by ONNX export. Download the result containing one `.mud` and one or
two `.axmodel` files.

## 7. Deploy later

Upload every generated model file into the same MaixCAM2 directory, for example:

```text
/root/models/drone_yolo11.mud
/root/models/drone_yolo11_npu.axmodel
/root/models/drone_yolo11_vnpu.axmodel
```

Then update `maixcam2_sound_yolo_gimbal/src/config.py` to use the new `.mud`,
`model_kind="YOLO11"` and `class_ids=[0]`, rebuild `main.py`, and perform a
vision-only board test before enabling the gimbal. That integration change is
intentionally deferred until the final converted filenames are known.
