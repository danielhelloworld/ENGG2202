# Dataset layout

Create the following directories locally. Images and labels are intentionally
ignored by Git because datasets and their licenses must be managed separately.

Create them with:

```bash
python scripts/prepare_dataset.py
```

```text
dataset/
  incoming/
    positive/
    negative/
  images/
    train/
    val/
    test/
  labels/
    train/
    val/
    test/
  calibration/
    images/
```

Each image has a matching `.txt` label with the same stem. Each object line is:

```text
0 center_x center_y width height
```

Coordinates are normalized to `0..1`. Class `0` is `drone`. A negative image
(bird, kite, aircraft, empty sky, cloud, building, glare) has a matching empty
label file. Split by original video/flight session, not by random neighbouring
frames, so nearly identical frames cannot leak between train, validation and
test sets.

Use `incoming/` only while collecting and reviewing files. Before annotation,
keep the original recording-session ID in every filename, such as
`S03_cloudy_far_000125.jpg`. Decide which complete sessions belong to train,
validation and test before moving their images into the final split folders.

`calibration/images/` is a separate set of 20--100 representative real-camera
images used only during MaixCAM2 model conversion. It needs no labels and must
not be sampled from the held-out test set.
