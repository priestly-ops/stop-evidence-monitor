"""One-dimensional wall-relative motion proxy for a tightly controlled test world.

It is not a general purpose velocity sensor. It may miss tangential motion,
rotation, moving landmarks, and attacks that spoof ranges or timestamps.
Receipt timestamps and sensor stamps are kept separate.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from statistics import median
from typing import Optional, Sequence


@dataclass(frozen=True)
class RangeRate:
    receipt_s: float
    sensor_stamp_s: float
    forward_speed_mps: float
    wall_range_m: float


class FixedWallLidarMotion:
    """Only valid when a single fixed wall is perpendicular to the robot's heading."""

    def __init__(self, max_range_spread_m: float = 0.10,
                 max_scan_gap_s: float = 0.5,
                 max_abs_speed_mps: float = 2.0) -> None:
        self.max_range_spread_m = max_range_spread_m
        self.max_scan_gap_s = max_scan_gap_s
        self.max_abs_speed_mps = max_abs_speed_mps
        self.last: Optional[tuple[float, float]] = None
        self.accepted_scans = 0
        self.rejected_scans = 0
        self.speed_samples = 0

    def observe(self, stamp_s: float, receipt_s: float, ranges: Sequence[float],
                min_range: float, max_range: float) -> Optional[RangeRate]:
        if not (math.isfinite(stamp_s) and math.isfinite(receipt_s) and
                math.isfinite(min_range) and math.isfinite(max_range) and
                0 < min_range < max_range and len(ranges) >= 3):
            self.rejected_scans += 1
            return None
        center = len(ranges) // 2
        beam = ranges[center - 1:center + 2]
        if (len(beam) != 3 or
                not all(math.isfinite(x) and min_range < x < max_range for x in beam) or
                max(beam) - min(beam) > self.max_range_spread_m):
            self.rejected_scans += 1
            self.last = None  # cannot bridge an unobserved interval
            return None
        d = median(beam)
        prev = self.last
        if prev is not None and stamp_s <= prev[0]:
            self.rejected_scans += 1  # repeated/out-of-order sensor timestamps
            return None
        self.last = (stamp_s, d)
        self.accepted_scans += 1
        if prev is None:
            return None
        dt = stamp_s - prev[0]
        if dt > self.max_scan_gap_s or dt < 0.015:
            self.rejected_scans += 1
            return None
        vx = (prev[1] - d) / dt
        if not math.isfinite(vx) or abs(vx) > self.max_abs_speed_mps:
            self.rejected_scans += 1
            self.last = None
            return None
        self.speed_samples += 1
        return RangeRate(receipt_s, stamp_s, vx, d)
