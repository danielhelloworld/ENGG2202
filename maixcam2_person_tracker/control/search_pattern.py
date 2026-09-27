"""Generate an expanding local search around the last predicted bearing."""

import math
from typing import Tuple

from config import SearchConfig


class ExpandingSearchPattern:
    def __init__(self, config: SearchConfig) -> None:
        self.config = config

    def target(
        self,
        center_pan_deg: float,
        center_tilt_deg: float,
        elapsed_s: float,
    ) -> Tuple[float, float]:
        pan_amplitude = min(
            self.config.maximum_pan_amplitude_deg,
            self.config.initial_pan_amplitude_deg
            + self.config.pan_growth_deg_s * elapsed_s,
        )
        tilt_amplitude = min(
            self.config.maximum_tilt_amplitude_deg,
            self.config.initial_tilt_amplitude_deg
            + self.config.tilt_growth_deg_s * elapsed_s,
        )
        pan_phase = 2.0 * math.pi * self.config.pan_frequency_hz * elapsed_s
        tilt_phase = pan_phase * self.config.tilt_frequency_ratio
        return (
            center_pan_deg + pan_amplitude * math.sin(pan_phase),
            center_tilt_deg + tilt_amplitude * math.sin(tilt_phase),
        )
