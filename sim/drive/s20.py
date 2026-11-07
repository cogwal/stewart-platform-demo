"""Actuonix S20 linear stepper actuator: the physical side of one leg.

Open loop: the drive's step counter and the rod's real position are independent. The rod stops at
its mechanical end stops; steps commanded beyond them are lost (the motor skips), which the drive
cannot see except through stallGuard.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class S20:
    stroke_mm: float = 100.0
    position_mm: float = 37.0  # unknown to the drive at power-up
    stalled: bool = False      # pushing against an end stop during the last move
    jam_mm: float | None = None  # fault injection: an obstacle the rod can't pass (either direction)

    def move(self, delta_mm: float) -> float:
        """Apply a commanded displacement. Returns the displacement actually achieved."""
        target = self.position_mm + delta_mm
        lo, hi = 0.0, self.stroke_mm
        if self.jam_mm is not None:
            if self.position_mm <= self.jam_mm:
                hi = min(hi, self.jam_mm)
            else:
                lo = max(lo, self.jam_mm)
        clamped = min(max(target, lo), hi)
        self.stalled = delta_mm != 0.0 and clamped != target
        achieved = clamped - self.position_mm
        self.position_mm = clamped
        return achieved

    @property
    def at_retracted_stop(self) -> bool:
        return self.position_mm <= 0.0
