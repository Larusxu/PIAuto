"""Control-loop neutral contracts. No chip-specific imports allowed here."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping
import numpy as np

class Loop(str, Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    CURRENT = "current"

class DQMode(str, Enum):
    SHARED = "shared"
    INDEPENDENT = "independent"

@dataclass(frozen=True)
class PI:
    kp: float
    ki: float
    def __post_init__(self):
        if not np.isfinite([self.kp, self.ki]).all() or min(self.kp, self.ki) < 0:
            raise ValueError("PI gains must be finite and nonnegative")

@dataclass(frozen=True)
class Gains:
    position: PI
    velocity: PI
    d: PI
    q: PI
    def shared_current(self, pi: PI) -> "Gains":
        return Gains(self.position, self.velocity, pi, pi)
    def set_loop(self, loop: Loop, pi: PI, axis: str = "q", mode: DQMode = DQMode.SHARED) -> "Gains":
        if loop == Loop.POSITION:
            return Gains(pi, self.velocity, self.d, self.q)
        if loop == Loop.VELOCITY:
            return Gains(self.position, pi, self.d, self.q)
        if mode == DQMode.SHARED:
            return self.shared_current(pi)
        if axis not in ("d", "q"):
            raise ValueError("axis must be d or q")
        return Gains(self.position, self.velocity, pi if axis == "d" else self.d,
                     pi if axis == "q" else self.q)
    def loop_gain(self, loop: Loop, axis: str = "q") -> PI:
        return self.position if loop == Loop.POSITION else self.velocity if loop == Loop.VELOCITY else (self.d if axis == "d" else self.q)

@dataclass(frozen=True)
class Capabilities:
    position_step: bool = False
    velocity_step: bool = False
    d_step: bool = False
    q_step: bool = False
    chirp: bool = False
    safe_stop: bool = False
    rollback: bool = False
    telemetry_hz: float = 0
    timestamp_quality: str = "unknown"
    verified_telemetry: bool = False
    verified_command_scheduling: bool = False
    def supports(self, loop: Loop, axis: str = "q") -> bool:
        return self.position_step if loop == Loop.POSITION else self.velocity_step if loop == Loop.VELOCITY else (self.d_step if axis == "d" else self.q_step)

@dataclass(frozen=True)
class SafetyEnvelope:
    max_step: float
    max_speed: float
    max_current: float
    max_duration_s: float
    min_position: float = -float("inf")
    max_position: float = float("inf")
    def validate(self, spec: "Experiment") -> None:
        if not (0 < abs(spec.step) <= self.max_step and 0 < spec.duration_s <= self.max_duration_s):
            raise ValueError("step/duration outside safety envelope")
        if not (self.min_position <= spec.start <= self.max_position and
                self.min_position <= spec.start+spec.step <= self.max_position):
            raise ValueError("position outside safety envelope")
        if self.max_speed <= 0 or self.max_current <= 0:
            raise ValueError("invalid safety constraints")

@dataclass(frozen=True)
class Experiment:
    loop: Loop = Loop.POSITION
    axis: str = "q"
    dq_mode: DQMode = DQMode.SHARED
    start: float = 0.0
    step: float = 1.0
    duration_s: float = 2.0
    settle_band: float = 0.02
    def __post_init__(self):
        if not 0 < self.settle_band < 1 or not self.duration_s > 0 or self.step == 0:
            raise ValueError("invalid experiment")
        if self.axis not in ("d", "q"):
            raise ValueError("axis must be d or q")

@dataclass
class Trace:
    timestamp_s: np.ndarray
    target: np.ndarray
    actual: np.ndarray
    signal: str = "position"
    units: str = "rad"
    command_time_s: float = 0.0
    dropped_frames: int = 0
    metadata: dict = field(default_factory=dict)
    def __post_init__(self):
        self.timestamp_s = np.asarray(self.timestamp_s, dtype=float)
        self.target = np.asarray(self.target, dtype=float)
        self.actual = np.asarray(self.actual, dtype=float)
        if not (len(self.timestamp_s) == len(self.target) == len(self.actual)):
            raise ValueError("trace length mismatch")

@dataclass(frozen=True)
class Quality:
    valid: bool
    reasons: tuple[str, ...]
    sampling_hz: float

@dataclass(frozen=True)
class Metrics:
    overshoot: float
    overshoot_percent: float
    rise_time_s: float | None
    settling_time_s: float | None
    steady_error: float
    rms_error: float
    crossings: int
    peak: float
    quality: Quality

@dataclass(frozen=True)
class Proposal:
    baseline: Gains
    candidate: Gains
    metrics: Metrics
    rationale: str
    confidence: str
    predicted_metrics: Mapping[str, float] = field(default_factory=dict)
