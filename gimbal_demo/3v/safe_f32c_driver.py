"""Safety-enforced F32C gimbal driver.

The Y-axis mechanical limit is enforced here, below every GUI or application
control. UI widgets may clamp values for convenience, but this driver rejects
an out-of-range command before any position frame is written to the serial bus.
"""

from __future__ import annotations

import math

from f32c_protocol import (
    DEFAULT_Y_MAX_DEG,
    DEFAULT_Y_MIN_DEG,
    F32CGimbal,
    F32CLimitError,
)


class SafeF32CGimbal(F32CGimbal):
    """F32C driver with a non-bypassable relative Y-axis hard limit."""

    def __init__(
        self,
        transport,
        *,
        y_min_deg: float = DEFAULT_Y_MIN_DEG,
        y_max_deg: float = DEFAULT_Y_MAX_DEG,
        **kwargs,
    ) -> None:
        if not math.isfinite(y_min_deg) or not math.isfinite(y_max_deg):
            raise ValueError("Y hard limits must be finite")
        if y_min_deg >= y_max_deg:
            raise ValueError("y_min_deg must be smaller than y_max_deg")
        super().__init__(
            transport, y_min_deg=y_min_deg, y_max_deg=y_max_deg, **kwargs
        )

    def validate_relative_angles(self, x_deg: float, y_deg: float) -> None:
        """Validate a target before any serial position frame is generated."""
        if not math.isfinite(x_deg) or not math.isfinite(y_deg):
            raise F32CLimitError("X/Y targets must be finite numbers")
        if not self.y_min_deg <= y_deg <= self.y_max_deg:
            raise F32CLimitError(
                "Y target %.3f deg violates hard limit %.1f..%.1f deg"
                % (y_deg, self.y_min_deg, self.y_max_deg)
            )

    def clamp_y_for_ui(self, y_deg: float) -> float:
        """Convenience clamp for widgets; driver validation remains authoritative."""
        return min(self.y_max_deg, max(self.y_min_deg, float(y_deg)))

    def set_relative_angles(self, x_deg: float, y_deg: float) -> None:
        self.validate_relative_angles(x_deg, y_deg)
        super().set_relative_angles(x_deg, y_deg)
