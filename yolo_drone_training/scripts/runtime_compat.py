"""Runtime setup and narrowly scoped compatibility fixes for local training."""

from __future__ import annotations

import os
from contextlib import nullcontext
from pathlib import Path


def configure_project_caches(project_root: Path) -> None:
    """Keep third-party caches in the project instead of macOS user folders."""
    matplotlib_cache = project_root / ".cache" / "matplotlib"
    ultralytics_cache = project_root / ".cache" / "ultralytics"
    matplotlib_cache.mkdir(parents=True, exist_ok=True)
    ultralytics_cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(matplotlib_cache))
    os.environ.setdefault("YOLO_CONFIG_DIR", str(ultralytics_cache))


def patch_mps_validation_autocast() -> None:
    """Patch two PyTorch 2.2 MPS edge cases used during YOLO validation."""
    import torch
    from ultralytics.engine import validator
    from ultralytics.utils.loss import v8DetectionLoss
    from ultralytics.utils.ops import xywh2xyxy

    original_autocast = validator.autocast
    if not getattr(original_autocast, "_engg2202_mps_safe", False):
        def mps_safe_autocast(enabled: bool, device: str = "cuda"):
            if device == "mps":
                return nullcontext()
            return original_autocast(enabled, device=device)

        mps_safe_autocast._engg2202_mps_safe = True
        validator.autocast = mps_safe_autocast

    original_preprocess = v8DetectionLoss.preprocess
    if not getattr(original_preprocess, "_engg2202_mps_safe", False):
        def mps_safe_preprocess(
            self,
            targets: torch.Tensor,
            batch_size: int,
            scale_tensor: torch.Tensor,
        ) -> torch.Tensor:
            if self.device.type != "mps":
                return original_preprocess(self, targets, batch_size, scale_tensor)
            nl, ne = targets.shape
            if nl == 0:
                return torch.zeros(batch_size, 0, ne - 1, device=self.device)
            batch_idx = targets[:, 0].long()
            # PyTorch 2.2 MPS can return invalid values for both a scalar shape
            # and counts.max().item(). Compute these tiny indexing values on CPU.
            batch_indices = batch_idx.cpu().tolist()
            per_image_counts = [0] * batch_size
            within_indices = []
            for image_index in batch_indices:
                if not 0 <= image_index < batch_size:
                    raise ValueError(f"invalid batch index {image_index} for size {batch_size}")
                within_indices.append(per_image_counts[image_index])
                per_image_counts[image_index] += 1
            max_targets = max(per_image_counts)
            out = torch.zeros(batch_size, max_targets, ne - 1, device=self.device)
            within_idx = torch.tensor(within_indices, dtype=torch.long, device=self.device)
            out[batch_idx, within_idx] = targets[:, 1:]
            out[..., 1:5] = xywh2xyxy(out[..., 1:5].mul_(scale_tensor))
            return out

        mps_safe_preprocess._engg2202_mps_safe = True
        v8DetectionLoss.preprocess = mps_safe_preprocess
