"""Adapter contract: hardware safety must be enforced independently on the MCU."""
from __future__ import annotations
from abc import ABC, abstractmethod
from ..contracts import Capabilities, Experiment, Gains, SafetyEnvelope, Trace

class MotorAdapter(ABC):
    @abstractmethod
    def capabilities(self) -> Capabilities: ...
    @abstractmethod
    def read_pi(self) -> Gains: ...
    @abstractmethod
    def snapshot(self) -> object: ...
    @abstractmethod
    def run_step(self, spec: Experiment, safety: SafetyEnvelope) -> Trace: ...
    @abstractmethod
    def apply_candidate(self, gains: Gains, *, approved: bool) -> None: ...
    @abstractmethod
    def restore(self, snapshot: object) -> None: ...
    @abstractmethod
    def safe_stop(self) -> None: ...
    @abstractmethod
    def faults(self) -> list[str]: ...

class ReadOnlyAdapter(MotorAdapter):
    """No motor motion through this adapter; offline traces only."""
    def run_step(self, spec, safety):
        raise RuntimeError("offline adapter cannot move a motor")
    def apply_candidate(self, gains, *, approved):
        raise RuntimeError("read-only adapter")
    def restore(self, snapshot):
        raise RuntimeError("read-only adapter")
    def safe_stop(self):
        pass
    def faults(self):
        return []
