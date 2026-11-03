"""Chip-neutral streaming experiment contracts shared by V1 and V2.

Adapters own transport, clock provenance, decoding and safety. Algorithms consume
only normalized samples. This module deliberately contains no chip/register code.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Iterable, Protocol, runtime_checkable
import numpy as np
from ..contracts import Experiment, SafetyEnvelope, Gains

@dataclass(frozen=True)
class SignalSpec:
    name: str
    units: str
    required: bool = True

@dataclass
class SampleBatch:
    device_id: str
    motor_id: str
    test_id: str
    timestamp_s: np.ndarray
    signals: dict[str, np.ndarray]
    units: dict[str, str] = field(default_factory=dict)
    dropped_frames: int = 0
    timestamp_source: str = "unknown"
    synchronized: bool = False
    metadata: dict = field(default_factory=dict)

    def validate(self) -> None:
        t = np.asarray(self.timestamp_s, dtype=float)
        if t.ndim != 1 or len(t) < 2 or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
            raise ValueError("timestamps must be finite, one-dimensional and strictly increasing")
        for name, values in self.signals.items():
            a = np.asarray(values, dtype=float)
            if a.shape != t.shape or not np.isfinite(a).all():
                raise ValueError(f"signal {name!r} must be finite and match timestamp length")
        if self.dropped_frames < 0:
            raise ValueError("dropped_frames cannot be negative")
        self.timestamp_s = t
        self.signals = {k: np.asarray(v, dtype=float) for k, v in self.signals.items()}

@dataclass(frozen=True)
class ExperimentHandle:
    test_id: str
    started_at_s: float | None = None

@runtime_checkable
class StreamingMotorAdapter(Protocol):
    """Optional richer contract; legacy MotorAdapter remains supported."""
    def prepare_experiment(self, spec: Experiment, safety: SafetyEnvelope,
                           signals: Iterable[SignalSpec]) -> ExperimentHandle: ...
    def start_experiment(self, handle: ExperimentHandle) -> None: ...
    def read_samples(self, handle: ExperimentHandle, timeout_s: float) -> SampleBatch: ...
    def stop_experiment(self, handle: ExperimentHandle) -> None: ...
    def apply_pi(self, gains: Gains) -> None: ...
    def verify_pi(self, gains: Gains) -> bool: ...

def validate_batch(batch: SampleBatch, required: Iterable[str], *,
                   max_drop_fraction: float = 0.02) -> dict:
    batch.validate()
    missing = sorted(set(required) - set(batch.signals))
    if missing:
        raise ValueError("missing required signals: " + ", ".join(missing))
    n = len(batch.timestamp_s)
    fraction = batch.dropped_frames / max(n + batch.dropped_frames, 1)
    if fraction > max_drop_fraction:
        raise ValueError(f"telemetry drop fraction {fraction:.3%} exceeds limit")
    dt = np.diff(batch.timestamp_s)
    return {"samples": n, "sampling_hz": float(1 / np.median(dt)),
            "dropped_frames": batch.dropped_frames, "drop_fraction": fraction,
            "timestamp_source": batch.timestamp_source,
            "synchronized": batch.synchronized}
