"""Paired RGB+thermal early fusion for the pinned Ultralytics 8.4.104 runtime.

Input order is RGB (3 channels) plus rendered grayscale thermal (1 channel).
Detection boxes are expressed in the RGB image coordinate system. Thermal is
resized to the RGB canvas; this baseline does not assert pixel registration.
"""
from __future__ import annotations

import math
import random
from copy import copy
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics.data.dataset import YOLODataset
from ultralytics.models.yolo.detect.train import DetectionTrainer
from ultralytics.models.yolo.detect.val import DetectionValidator
from ultralytics.utils.torch_utils import unwrap_model


class PairedRGBTDataset(YOLODataset):
    def load_image(self, i, rect_mode=True, resize_short=False):
        if self.ims[i] is not None:
            return self.ims[i], self.im_hw0[i], self.im_hw[i]
        path = Path(self.im_files[i])
        rgb_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if rgb_bgr is None:
            raise FileNotFoundError(path)
        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        h0, w0 = rgb.shape[:2]
        thermal_path = path.with_name(path.name.replace("_rgb.jpg", "_thermal.jpg"))
        has_pair_name = path.name.endswith("_rgb.jpg")
        thermal = cv2.imread(str(thermal_path), cv2.IMREAD_GRAYSCALE) if has_pair_name else None
        if thermal is None and not self.data.get("allow_missing_thermal", False):
            raise FileNotFoundError(f"Missing thermal pair for {path}")
        if rect_mode:
            if resize_short:
                ratio = self.imgsz / min(h0, w0)
                w, h = (math.ceil(w0 * ratio), self.imgsz) if h0 < w0 else (self.imgsz, math.ceil(h0 * ratio))
            else:
                ratio = self.imgsz / max(h0, w0)
                w, h = min(math.ceil(w0 * ratio), self.imgsz), min(math.ceil(h0 * ratio), self.imgsz)
        else:
            w = h = self.imgsz
        rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_LINEAR)
        thermal = np.zeros((h, w), dtype=np.uint8) if thermal is None else cv2.resize(thermal, (w, h), interpolation=cv2.INTER_LINEAR)
        if self.augment:
            # Sensor-specific photometric changes; geometry is shared downstream.
            hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV).astype(np.float32)
            hsv[..., 0] = (hsv[..., 0] + random.uniform(-2.7, 2.7)) % 180
            hsv[..., 1] *= random.uniform(0.75, 1.25)
            hsv[..., 2] *= random.uniform(0.85, 1.15)
            hsv[..., 1:] = np.clip(hsv[..., 1:], 0, 255)
            rgb = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB)
            thermal = np.clip(thermal.astype(np.float32) * random.uniform(0.9, 1.1) + random.uniform(-8, 8), 0, 255).astype(np.uint8)
        image = np.concatenate((rgb, thermal[..., None]), axis=2)
        if self.augment:
            self.ims[i], self.im_hw0[i], self.im_hw[i] = image, (h0, w0), image.shape[:2]
            self.buffer.append(i)
            if 1 < len(self.buffer) >= self.max_buffer_length:
                j = self.buffer.pop(0)
                if self.cache != "ram":
                    self.ims[j], self.im_hw0[j], self.im_hw[j] = None, None, None
        return image, (h0, w0), image.shape[:2]


def build_paired_dataset(args, img_path, batch, data, mode, stride, rect=False):
    if data.get("channels") != 4:
        raise ValueError("RGB-T fusion requires data.yaml channels: 4")
    return PairedRGBTDataset(
        img_path=img_path, imgsz=args.imgsz, batch_size=batch,
        augment=mode == "train", hyp=args, rect=args.rect or rect,
        cache=False, single_cls=True, stride=stride,
        pad=0.0 if mode == "train" else 0.5, prefix=f"RGBT {mode}: ",
        task="detect", classes=args.classes, data=data,
        fraction=args.fraction if mode == "train" else 1.0,
    )


class PairedRGBTValidator(DetectionValidator):
    def build_dataset(self, img_path, mode="val", batch=None):
        return build_paired_dataset(self.args, img_path, batch, self.data, mode, self.stride)

    def preprocess(self, batch):
        batch = super().preprocess(batch)
        if batch["img"].shape[1] != 4:
            raise RuntimeError("Validation did not load paired four-channel data")
        return batch


class PairedRGBTTrainer(DetectionTrainer):
    def build_dataset(self, img_path, mode="train", batch=None):
        stride = max(int(unwrap_model(self.model).stride.max()), 32)
        return build_paired_dataset(self.args, img_path, batch, self.data, mode, stride, rect=mode == "val")

    def get_model(self, cfg=None, weights=None, verbose=True):
        model = super().get_model(cfg=cfg, weights=weights, verbose=verbose)
        if model.model[0].conv.in_channels != 4:
            raise RuntimeError("Fusion model must accept RGB+T (four channels)")
        source = weights.get("model") if isinstance(weights, dict) else weights
        if source is not None and source.model[0].conv.in_channels == 3:
            # Preserve v1's RGB prediction at initialization; learn thermal contribution.
            with torch.no_grad():
                model.model[0].conv.weight[:, 3:].zero_()
        return model

    def get_validator(self):
        self.loss_names = "box_loss", "cls_loss", "dfl_loss"
        return PairedRGBTValidator(self.test_loader, save_dir=self.save_dir, args=copy(self.args), _callbacks=self.callbacks)

    def preprocess_batch(self, batch):
        batch = super().preprocess_batch(batch)
        if batch["img"].shape[1] != 4:
            raise RuntimeError("Training did not load paired four-channel data")
        return batch
