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
    # Static force model (config/platform.yaml simulation:), set by the platform physics in sim/drive/__main__.py.
    load_n: float = 0.0          # external axial force, + = compressing the leg (resists extension)
    max_force_n: float = 0.0     # 0 = no force limit
    derate_at_10mm_s: float = 0.3
    overloaded: bool = False     # the last move was refused: motor skipped steps against the load

    def available_force(self, speed_mm_s: float) -> float:
        return self.max_force_n * max(0.2, 1.0 - self.derate_at_10mm_s * abs(speed_mm_s) / 10.0)

    def move(self, delta_mm: float, dt: float = 0.001) -> float:
        """Apply a commanded displacement. Returns the displacement actually achieved."""
        against = delta_mm * self.load_n > 0  # extending under compression or retracting under tension
        if self.max_force_n > 0 and delta_mm != 0.0 and against and \
                abs(self.load_n) > self.available_force(delta_mm / dt if dt > 0 else 0.0):
            self.overloaded = self.stalled = True  # steps lost, rod stays
            return 0.0
        self.overloaded = False
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
