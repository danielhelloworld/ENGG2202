"""YOLO11 adapter that exposes detector results through project data types."""

from typing import List

from maix import nn

from config import ModelConfig
from domain import BoundingBox, Detection


class YoloPersonDetector:
    def __init__(self, config: ModelConfig) -> None:
        self.config = config
        self.model = nn.YOLO11(model=config.path, dual_buff=config.dual_buffer)

    @property
    def input_width(self) -> int:
        return self.model.input_width()

    @property
    def input_height(self) -> int:
        return self.model.input_height()

    @property
    def input_format(self):
        return self.model.input_format()

    def detect(self, frame, timestamp_ms: int) -> List[Detection]:
        results = self.model.detect(
            frame,
            conf_th=self.config.confidence_threshold,
            iou_th=self.config.iou_threshold,
        )
        detections: List[Detection] = []
        for result in results:
            if result.class_id != self.config.target_class_id:
                continue
            detections.append(
                Detection(
                    timestamp_ms=timestamp_ms,
                    box=BoundingBox(result.x, result.y, result.w, result.h),
                    class_id=result.class_id,
                    confidence=float(result.score),
                )
            )
        return detections
