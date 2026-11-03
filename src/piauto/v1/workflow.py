"""Explicit approval + test + accept/rollback; no implicit writes or initialization."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from .candidate import propose
from ..metrics import measure_step
from ..contracts import Experiment, SafetyEnvelope, Proposal, Trace, Metrics
from ..adapters.base import MotorAdapter

class State(str,Enum):
    IDLE="idle"
    PROPOSED="proposed"
    VERIFYING="verifying"
    ACCEPTED="accepted"
    ROLLED_BACK="rolled_back"

@dataclass
class Verification:
    candidate_metrics:Metrics
    improved:bool
    state:State

class V1SemiAutoTuner:
    def __init__(self,adapter:MotorAdapter):
        self.adapter=adapter
        self.state=State.IDLE
        self._snapshot=None
        self.proposal:Proposal|None=None
        self.baseline_trace:Trace|None=None
    def analyze_and_propose(self,spec:Experiment,safety:SafetyEnvelope,*,offline_trace:Trace|None=None)->Proposal:
        if self.state!=State.IDLE:raise RuntimeError("finish previous tuning session first")
        safety.validate(spec)
        if self.adapter.faults():raise RuntimeError("motor fault")
        if offline_trace is None:
            cap=self.adapter.capabilities()
            if not (cap.supports(spec.loop,spec.axis) and cap.safe_stop and cap.verified_command_scheduling):
                raise RuntimeError("live step requires validated capabilities and scheduling; use offline trace")
        baseline=self.adapter.read_pi()
        self._snapshot=self.adapter.snapshot()
        try:
            trace=offline_trace if offline_trace is not None else self.adapter.run_step(spec,safety)
        except BaseException:
            self.adapter.safe_stop()
            raise
        metrics=measure_step(trace,spec.settle_band)
        if not metrics.quality.valid:raise ValueError("unreliable step data: "+", ".join(metrics.quality.reasons))
        self.proposal=propose(baseline,spec,metrics)
        # python-control + SciPy: fit measured CLOSED-loop surrogate for diagnostics.
        # This is NOT a hardware PI-to-plant model and must not imply a gain prediction.
        try:
            from .model_diagnostics import diagnose_step
            self.model_diagnostic=diagnose_step(trace)
        except (ImportError, ValueError, RuntimeError, ArithmeticError) as exc:
            self.model_diagnostic={"available":False,"reason":str(exc)}
        self.baseline_trace=trace
        self._spec,self._safety=spec,safety
        self.state=State.PROPOSED
        return self.proposal
    def verify_approved_candidate(self,*,approved:bool)->Verification:
        if self.state!=State.PROPOSED or self.proposal is None:raise RuntimeError("no pending proposal")
        if not approved:raise PermissionError("explicit approval required")
        cap=self.adapter.capabilities()
        if not (cap.safe_stop and cap.rollback and cap.supports(self._spec.loop,self._spec.axis)
                and cap.verified_command_scheduling):
            raise RuntimeError("cannot safely apply or validate candidate on this adapter")
        self.state=State.VERIFYING
        try:
            self.adapter.safe_stop()
            self.adapter.apply_candidate(self.proposal.candidate,approved=True)
            if self.adapter.faults():raise RuntimeError("motor fault after PI write")
            trace=self.adapter.run_step(self._spec,self._safety)
            new=measure_step(trace,self._spec.settle_band)
            if not new.quality.valid:raise RuntimeError("invalid validation telemetry")
            before=self.proposal.metrics
            # Strict non-regression on overshoot and oscillation; improved RMS tracking
            improved=(new.overshoot_percent<=before.overshoot_percent+0.1 and
                      new.crossings<=before.crossings and new.rms_error < before.rms_error*.99)
            if not improved:
                self.adapter.safe_stop()
                self.adapter.restore(self._snapshot)
                self.state=State.ROLLED_BACK
            else:
                self.state=State.ACCEPTED
            return Verification(new,improved,self.state)
        except BaseException:
            self.adapter.safe_stop()
            try:self.adapter.restore(self._snapshot)
            finally:self.state=State.ROLLED_BACK
            raise
    def rollback(self)->None:
        if self._snapshot is None:raise RuntimeError("no snapshot")
        self.adapter.safe_stop()
        self.adapter.restore(self._snapshot)
        self.state=State.ROLLED_BACK
