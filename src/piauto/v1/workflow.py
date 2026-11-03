"""V1 semiautomatic PI tuning, explicit user approval and validated rollback.

Current shared d/q gains require BOTH d and q evidence, live or offline.
No default motor initialization, implicit enabling or register writes.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from collections.abc import Mapping
from .candidate import propose
from ..metrics import measure_step
from ..contracts import Experiment, SafetyEnvelope, Proposal, Trace, Metrics, Loop, DQMode
from ..adapters.base import MotorAdapter

class State(str,Enum):
    IDLE="idle"
    PROPOSED="proposed"
    VERIFYING="verifying"
    ACCEPTED="accepted"
    ROLLED_BACK="rolled_back"

@dataclass
class Verification:
    candidate_metrics: Metrics
    improved: bool
    state: State

def _axes(spec):
    return ("d","q") if spec.loop==Loop.CURRENT and spec.dq_mode==DQMode.SHARED else (spec.axis,)

def _axis_spec(spec,axis):
    return Experiment(loop=spec.loop,axis=axis,dq_mode=spec.dq_mode,
                      start=spec.start,step=spec.step,
                      duration_s=spec.duration_s,settle_band=spec.settle_band)

def _worst(metrics,spec):
    # Select the least satisfactory axis by dimensionless error, then overshoot.
    return max(metrics,key=lambda m:(m.rms_error/max(abs(spec.step),1e-9)
                                      +.02*m.overshoot_percent+.04*m.crossings))

class V1SemiAutoTuner:
    def __init__(self,adapter:MotorAdapter):
        self.adapter=adapter
        self.state=State.IDLE
        self._snapshot=None
        self.proposal:Proposal|None=None
        self.baseline_trace:Trace|None=None
        self.baseline_metrics_by_axis:dict[str,Metrics]={}

    def analyze_and_propose(self,spec:Experiment,safety:SafetyEnvelope,*,
                            offline_trace:Trace|Mapping[str,Trace]|None=None)->Proposal:
        if self.state!=State.IDLE:raise RuntimeError("finish previous tuning session first")
        safety.validate(spec)
        if self.adapter.faults():raise RuntimeError("motor fault")
        axes=_axes(spec)
        if offline_trace is None:
            cap=self.adapter.capabilities()
            if not (all(cap.supports(spec.loop,a) for a in axes)
                    and cap.safe_stop and cap.verified_command_scheduling
                    and cap.verified_telemetry):
                raise RuntimeError("live step requires validated axes and MCU safety")
        else:
            if len(axes)>1 and (not isinstance(offline_trace,Mapping) or
                                     set(axes)-set(offline_trace)):
                raise ValueError("shared d/q offline tuning requires traces for both axes")
        baseline=self.adapter.read_pi()
        self._snapshot=self.adapter.snapshot()
        traces={};metrics={}
        try:
            for a in axes:
                if self.adapter.faults():raise RuntimeError("fault during test")
                trace=(self.adapter.run_step(_axis_spec(spec,a),safety) if offline_trace is None
                       else offline_trace[a] if isinstance(offline_trace,Mapping) else offline_trace)
                m=measure_step(trace,spec.settle_band)
                if not m.quality.valid:
                    raise ValueError("unreliable "+a+" test telemetry: "+", ".join(m.quality.reasons))
                traces[a]=trace;metrics[a]=m
        except BaseException:
            self.adapter.safe_stop()
            raise
        worst=_worst(list(metrics.values()),spec)
        self.proposal=propose(baseline,spec,worst)
        self.baseline_metrics_by_axis=metrics
        self.baseline_trace=traces[spec.axis]
        try:
            from .model_diagnostics import diagnose_step
            self.model_diagnostic=diagnose_step(self.baseline_trace)
        except (ImportError,ValueError,RuntimeError,ArithmeticError) as exc:
            self.model_diagnostic={"available":False,"reason":str(exc)}
        self._spec,self._safety=spec,safety
        self.state=State.PROPOSED
        return self.proposal

    def verify_approved_candidate(self,*,approved:bool)->Verification:
        if self.state!=State.PROPOSED or self.proposal is None:
            raise RuntimeError("no pending proposal")
        if not approved:raise PermissionError("explicit user approval required")
        cap=self.adapter.capabilities()
        axes=_axes(self._spec)
        if not (cap.safe_stop and cap.rollback and cap.verified_telemetry and
                cap.verified_command_scheduling and
                all(cap.supports(self._spec.loop,a) for a in axes)):
            raise RuntimeError("cannot safely apply/verify on this adapter")
        self.state=State.VERIFYING
        try:
            self.adapter.safe_stop()
            self.adapter.apply_candidate(self.proposal.candidate,approved=True)
            if hasattr(self.adapter,"verify_written_pi") and not self.adapter.verify_written_pi(self.proposal.candidate):
                raise RuntimeError("PI readback mismatch")
            measured={}
            for a in axes:
                if self.adapter.faults():raise RuntimeError("motor fault after PI write")
                trace=self.adapter.run_step(_axis_spec(self._spec,a),self._safety)
                m=measure_step(trace,self._spec.settle_band)
                if not m.quality.valid:raise RuntimeError("invalid "+a+" validation telemetry")
                measured[a]=m
            improved=all(
                measured[a].overshoot_percent<=self.baseline_metrics_by_axis[a].overshoot_percent+.1
                and measured[a].crossings<=self.baseline_metrics_by_axis[a].crossings
                and measured[a].rms_error<self.baseline_metrics_by_axis[a].rms_error*.99
                for a in axes
            )
            if not improved:
                self.adapter.safe_stop()
                self.adapter.restore(self._snapshot)
                self.state=State.ROLLED_BACK
            else:self.state=State.ACCEPTED
            return Verification(_worst(list(measured.values()),self._spec),improved,self.state)
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
