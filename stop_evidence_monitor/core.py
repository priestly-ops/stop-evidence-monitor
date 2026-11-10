"""Pure-Python stop-outcome evidence classifier.

Not an emergency-stop controller and not a claim of physical safety.
All times are monotonic receipt times in seconds, supplied by the caller.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Deque, Optional


class Status(str, Enum):
    IDLE = "IDLE"
    STOP_PENDING = "STOP_PENDING"
    STOP_EVIDENCED = "STOP_EVIDENCED"
    STOP_FAILED = "STOP_FAILED"
    UNVERIFIABLE = "UNVERIFIABLE"


@dataclass(frozen=True)
class Config:
    deadline_s: float = 1.5
    hold_s: float = 0.3
    min_stopped_samples: int = 3
    motion_fresh_s: float = 0.25
    command_fresh_s: float = 0.5
    max_motion_gap_s: float = 0.25
    linear_stop_mps: float = 0.02
    angular_stop_radps: float = 0.03
    independent_source: str = "independent_motion"
    require_veto: bool = False
    veto_source: str = "veto_motion"
    veto_fresh_s: float = 0.25

    def __post_init__(self) -> None:
        for name in ("deadline_s", "hold_s", "motion_fresh_s", "command_fresh_s",
                     "max_motion_gap_s", "linear_stop_mps", "angular_stop_radps", "veto_fresh_s"): 
            value = getattr(self, name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.min_stopped_samples < 2:
            raise ValueError("min_stopped_samples must be >= 2")
        if not self.veto_source or self.veto_source == self.independent_source:
            raise ValueError("veto must use a distinct source")
        if not self.independent_source or self.independent_source.lower() in {"odom", "odometry"}:
            raise ValueError("odom by itself cannot be the designated independent motion source")


@dataclass(frozen=True)
class VelocitySample:
    at_s: float
    linear_mps: float
    angular_radps: float


@dataclass(frozen=True)
class Decision:
    status: Status
    reason: str
    stop_age_s: Optional[float]


class StopEvidenceMonitor:
    """Fail-closed evaluation of a single latched stop request.

    Command samples MUST represent a downstream command observed after the
    safety arbiter, never a planner's intended command. Motion samples MUST
    come from a separately justified motion-observation path; raw /odom
    (especially if affected by the triggering fault) is explicitly ignored.
    """

    def __init__(self, cfg: Config = Config()) -> None:
        self.cfg = cfg
        self._requested_s: Optional[float] = None
        self._last_command: Optional[VelocitySample] = None
        self._motion: Deque[VelocitySample] = deque(maxlen=256)
        self._last_veto: Optional[VelocitySample] = None
        self._latched_problem: Optional[Decision] = None
        self._had_stop_evidence = False

    @staticmethod
    def _valid(at_s: float, linear_mps: float, angular_radps: float) -> bool:
        return all(isfinite(v) for v in (at_s, linear_mps, angular_radps))

    def request_stop(self, at_s: float) -> None:
        if not isfinite(at_s):
            raise ValueError("request time must be finite")
        self._requested_s = at_s
        self._last_command = None
        self._motion.clear()
        self._last_veto = None
        self._latched_problem = None
        self._had_stop_evidence = False

    def record_downstream_command(self, at_s: float, linear_mps: float, angular_radps: float) -> bool:
        if (self._requested_s is None or not self._valid(at_s, linear_mps, angular_radps)
                or at_s < self._requested_s):
            return False
        if self._last_command is not None and at_s < self._last_command.at_s:
            return False
        self._last_command = VelocitySample(at_s, linear_mps, angular_radps)
        return True

    def record_motion(self, at_s: float, linear_mps: float, angular_radps: float,
                      source: str) -> bool:
        if source != self.cfg.independent_source:
            return False
        if (self._requested_s is None or not self._valid(at_s, linear_mps, angular_radps)
                or at_s < self._requested_s):
            return False
        if self._motion and at_s < self._motion[-1].at_s:
            return False
        self._motion.append(VelocitySample(at_s, linear_mps, angular_radps))
        return True

    def record_veto_motion(self, at_s, linear_mps, angular_radps, source):
        """Negative-only evidence; cannot confirm standstill."""
        if source != self.cfg.veto_source:
            return False
        if (self._requested_s is None or
                not self._valid(at_s, linear_mps, angular_radps) or
                at_s < self._requested_s):
            return False
        if self._last_veto and at_s < self._last_veto.at_s:
            return False
        self._last_veto = VelocitySample(
            at_s, linear_mps, angular_radps
        )
        return True

    def _moving(self, sample: VelocitySample) -> bool:
        return (abs(sample.linear_mps) > self.cfg.linear_stop_mps
                or abs(sample.angular_radps) > self.cfg.angular_stop_radps)

    def _stopped_hold(self, now_s: float) -> bool:
        if len(self._motion) < self.cfg.min_stopped_samples:
            return False
        latest = self._motion[-1]
        if now_s < latest.at_s or now_s - latest.at_s > self.cfg.motion_fresh_s:
            return False
        run = [latest]
        for sample in reversed(list(self._motion)[:-1]):
            newer = run[-1]
            if (newer.at_s - sample.at_s > self.cfg.max_motion_gap_s
                    or self._moving(sample)):
                break
            run.append(sample)
        return (not self._moving(latest)
                and len(run) >= self.cfg.min_stopped_samples
                and run[0].at_s - run[-1].at_s >= self.cfg.hold_s)

    def evaluate(self, now_s: float) -> Decision:
        if not isfinite(now_s):
            raise ValueError("evaluation time must be finite")
        if self._requested_s is None:
            return Decision(Status.IDLE, "no stop request", None)
        if now_s < self._requested_s:
            raise ValueError("evaluation time precedes stop request")
        if self._latched_problem:
            return self._latched_problem

        age = now_s - self._requested_s
        after_deadline = age >= self.cfg.deadline_s
        cmd = self._last_command
        motion = self._motion[-1] if self._motion else None
        cmd_fresh = (cmd is not None and 0 <= now_s - cmd.at_s <= self.cfg.command_fresh_s)
        motion_fresh = (motion is not None and 0 <= now_s - motion.at_s <= self.cfg.motion_fresh_s)

        if cmd_fresh and self._moving(cmd) and (after_deadline or self._had_stop_evidence):
            return self._latch(Status.STOP_FAILED, "downstream command is still nonzero", age)
        if motion_fresh and self._moving(motion) and (after_deadline or self._had_stop_evidence):
            return self._latch(Status.STOP_FAILED, "independent motion still observed", age)

        veto_ok = True
        if self.cfg.require_veto:
            v = self._last_veto
            fresh = (v is not None and
                     0 <= now_s - v.at_s <= self.cfg.veto_fresh_s)
            if fresh and self._moving(v):
                if after_deadline or self._had_stop_evidence:
                    return self._latch(
                        Status.UNVERIFIABLE,
                        "secondary motion conflicts with stop evidence",
                        age)
                return Decision(
                    Status.STOP_PENDING, "conflicting motion evidence", age)
            veto_ok = fresh and not self._moving(v)

        # Success cannot be established using evidence that first arrived after
        # the deadline. Keep observing a previously-evidenced stop for regressions.
        in_time = (self._had_stop_evidence or (
            cmd is not None and motion is not None
            and cmd.at_s <= self._requested_s + self.cfg.deadline_s
            and motion.at_s <= self._requested_s + self.cfg.deadline_s))
        if (in_time and veto_ok and cmd_fresh and not self._moving(cmd) and motion_fresh
                and self._stopped_hold(now_s)):
            self._had_stop_evidence = True
            return Decision(Status.STOP_EVIDENCED,
                            "fresh downstream zero command plus sustained motion evidence; physical safety not certified",
                            age)

        if self._had_stop_evidence:
            return self._latch(Status.UNVERIFIABLE,
                               "previous stop evidence is no longer fresh or consistent", age)
        if after_deadline:
            return self._latch(Status.UNVERIFIABLE,
                               "deadline reached without sufficient fresh downstream and independent-motion evidence",
                               age)
        return Decision(Status.STOP_PENDING, "collecting evidence before deadline", age)

    def _latch(self, status: Status, reason: str, age: float) -> Decision:
        result = Decision(status, reason, age)
        self._latched_problem = result
        return result
