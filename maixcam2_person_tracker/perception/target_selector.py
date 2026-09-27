"""Select one stable person from all person detections."""

import math
from typing import List, Optional, Tuple

from config import SelectionConfig
from domain import Detection, GimbalState, TrackState
from estimation.camera_model import CameraModel


def _intersection_over_union(first, second) -> float:
    left = max(first.x, second.x)
    top = max(first.y, second.y)
    right = min(first.x + first.width, second.x + second.width)
    bottom = min(first.y + first.height, second.y + second.height)
    intersection = max(0, right - left) * max(0, bottom - top)
    union = first.area + second.area - intersection
    return intersection / union if union > 0 else 0.0


class TargetSelector:
    def __init__(
        self,
        config: SelectionConfig,
        camera_model: CameraModel,
    ) -> None:
        self.config = config
        self.camera_model = camera_model
        self.last_detection: Optional[Detection] = None

    def reset(self) -> None:
        self.last_detection = None

    def _angular_distance(
        self,
        detection: Detection,
        track: TrackState,
        gimbal: GimbalState,
    ) -> float:
        azimuth, elevation = self.camera_model.pixel_to_world(
            detection.box.center_x,
            detection.box.center_y,
            gimbal,
        )
        return math.hypot(
            azimuth - track.azimuth_deg,
            elevation - track.elevation_deg,
        )

    def _score_without_track(self, detection: Detection) -> float:
        center_distance = math.hypot(
            detection.box.center_x - self.camera_model.cx,
            detection.box.center_y - self.camera_model.cy,
        )
        maximum_distance = math.hypot(self.camera_model.cx, self.camera_model.cy)
        center_score = 1.0 - min(1.0, center_distance / maximum_distance)
        frame_area = self.camera_model.width * self.camera_model.height
        area_score = min(1.0, detection.box.area / max(1.0, frame_area * 0.30))
        return (
            self.config.confidence_weight * detection.confidence
            + self.config.center_weight * center_score
            + self.config.area_weight * area_score
        )

    def _score_with_track(
        self,
        detection: Detection,
        track: TrackState,
        gimbal: GimbalState,
    ) -> Tuple[float, float]:
        angular_distance = self._angular_distance(detection, track, gimbal)
        angular_score = 1.0 - min(
            1.0, angular_distance / self.config.reacquire_gate_deg
        )
        continuity_score = 0.0
        if self.last_detection is not None:
            continuity_score = _intersection_over_union(
                detection.box, self.last_detection.box
            )
        score = (
            0.50 * angular_score
            + 0.35 * detection.confidence
            + 0.15 * continuity_score
        )
        return score, angular_distance

    def select(
        self,
        detections: List[Detection],
        predicted_track: Optional[TrackState],
        gimbal: GimbalState,
    ) -> Optional[Detection]:
        if not detections:
            return None

        if predicted_track is None or not gimbal.valid:
            selected = max(detections, key=self._score_without_track)
            self.last_detection = selected
            return selected

        scored = []
        for detection in detections:
            score, distance = self._score_with_track(
                detection, predicted_track, gimbal
            )
            if distance <= self.config.reacquire_gate_deg:
                scored.append((score, detection))

        if not scored:
            return None

        selected = max(scored, key=lambda item: item[0])[1]
        self.last_detection = selected
        return selected
