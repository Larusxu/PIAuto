"""V2 model-aware full cascade orchestrator (hardware-neutral).

Safe operational contract:
- proven MCU safety capabilities and explicit authorization
- collect and validate per-loop model evidence
- design physical PI from that evidence
- let device adapter convert physical PI to its own register domain
- read back / check the applied candidate
- verify every shared d/q axis by real step metrics before accepting
- rollback on any failed trial; whole-session rollback on exceptions

No generic algorithm may write MCU registers or silently enable the motor.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from typing import Protocol, Sequence, Mapping, runtime_checkable
from ..contracts import PI,Gains,Loop,DQMode,Experiment,SafetyEnvelope
from ..adapters.base import MotorAdapter
from ..metrics import measure_step
from .designer import ModelEvidence, LoopDesign, design_current,design_velocity,design_position
from .workflow import _score
from .staged import TuningStage
import numpy as np

@runtime_checkable
class IdentifiedMotorAdapter(Protocol):
    """Additional methods required for true model-driven experiments."""
    def collect_model_evidence(self,loop:Loop,axis:str,safety:SafetyEnvelope)->ModelEvidence: ...
    def convert_physical_pi(self,gains:Gains,loop:Loop,axis:str,pi:PI,
                            mode:DQMode)->Gains: ...
    def verify_written_pi(self,gains:Gains)->bool: ...

@dataclass(frozen=True)
class ModelSettings:
    current_bandwidth_hz: float = 15.
    pwm_hz: float = 20000.
    velocity_lambda_s: float = .5
    position_horizon_s: float = 1.5
    max_gain_factor: float = 1.5
    min_improvement: float = .01
    def validate(self):
        if not (0<self.current_bandwidth_hz<=self.pwm_hz/20 and
                self.velocity_lambda_s>0 and self.position_horizon_s>0 and
                1<self.max_gain_factor<=3 and 0<self.min_improvement<1):
            raise ValueError("invalid model settings")

@dataclass(frozen=True)
class StageReport:
    loop: str
    axes: tuple[str,...]
    method: str
    baseline_score: float
    final_score: float
    accepted: bool
    tested: int
    candidate_physical: Mapping[str,PI]
    reason: str

@dataclass(frozen=True)
class FullAutoReport:
    status: str
    stages: tuple[StageReport,...]
    total_trials: int
    accepted_stages: int
    snapshot_restored: bool = False

def _stage_axes(spec):
    return ("d","q") if spec.loop==Loop.CURRENT and spec.dq_mode==DQMode.SHARED else (spec.axis,)

def _plan(adapter,stage,settings):
    spec=stage.experiment
    if spec.loop==Loop.CURRENT:
        axes=_stage_axes(spec)
        evidence={a:adapter.collect_model_evidence(spec.loop,a,stage.safety) for a in axes}
        return design_current(evidence,mode=spec.dq_mode,selected_axis=spec.axis,
                              bandwidth_hz=settings.current_bandwidth_hz,pwm_hz=settings.pwm_hz)
    if spec.loop==Loop.VELOCITY:
        e=adapter.collect_model_evidence(spec.loop,spec.axis,stage.safety)
        return design_velocity(e,lambda_s=settings.velocity_lambda_s)
    e=adapter.collect_model_evidence(spec.loop,spec.axis,stage.safety)
    return design_position(e,baseline=adapter.read_pi().position,horizon_s=settings.position_horizon_s)

def _measure(adapter,spec,safety,axes):
    out=[]
    for axis in axes:
        if adapter.faults():raise RuntimeError("hardware faults during test")
        s=Experiment(loop=spec.loop,axis=axis,dq_mode=spec.dq_mode,start=spec.start,
                     step=spec.step,duration_s=spec.duration_s,settle_band=spec.settle_band)
        m=measure_step(adapter.run_step(s,safety),s.settle_band)
        if not m.quality.valid:raise ValueError("invalid real test telemetry")
        out.append(m)
    return max(_score(m,spec.step) for m in out),out

class V2ModelAutoTuner:
    def __init__(self,adapter:MotorAdapter):
        self.adapter=adapter

    def run(self,stages:Sequence[TuningStage],*,settings:ModelSettings|None=None,
            authorized:bool=False)->FullAutoReport:
        if not authorized:raise PermissionError("explicit full-auto authorization required")
        if not stages:raise ValueError("no tuning stages")
        settings=settings or ModelSettings()
        settings.validate()
        order={Loop.CURRENT:0,Loop.VELOCITY:1,Loop.POSITION:2}
        indices=[order[x.experiment.loop] for x in stages]
        if indices!=sorted(indices) or len(set(indices))!=len(indices):
            raise ValueError("stages must be current->velocity->position without duplicates")
        if not isinstance(self.adapter,IdentifiedMotorAdapter):
            raise RuntimeError("adapter lacks model evidence, conversion or readback hooks")
        cap=self.adapter.capabilities()
        if not (cap.safe_stop and cap.rollback and cap.verified_telemetry and cap.verified_command_scheduling):
            raise RuntimeError("validated MCU safe stop, rollback, telemetry and scheduling required")
        for stage in stages:
            stage.safety.validate(stage.experiment)
            if not 1<=stage.max_rounds<=8:raise ValueError("max_rounds out of bounds")
            if not all(cap.supports(stage.experiment.loop,a) for a in _stage_axes(stage.experiment)):
                raise RuntimeError("test capability missing")
        if self.adapter.faults():raise RuntimeError("motor fault")
        original=self.adapter.snapshot()
        reports=[];trials=0;accepted=0
        try:
            for stage in stages:
                spec=stage.experiment
                axes=_stage_axes(spec)
                baseline=self.adapter.read_pi()
                base_score,base_metrics=_measure(self.adapter,spec,stage.safety,axes)
                if not np.isfinite(base_score):raise RuntimeError("unsettled baseline response")
                proposal=_plan(self.adapter,stage,settings)
                factors=np.linspace(.25,1.,stage.max_rounds)
                reason="no validated improvement"
                did_accept=False
                for alpha in factors:
                    current=self.adapter.read_pi()
                    candidate=current
                    for key,desired in proposal.suggested.items():
                        axis=key if key in ("d","q") else spec.axis
                        old=current.loop_gain(spec.loop,axis)
                        # Bound physical PI change per experiment.
                        safe_kp=np.clip(desired.kp,old.kp/settings.max_gain_factor,
                                        old.kp*settings.max_gain_factor)
                        safe_ki=np.clip(desired.ki,old.ki/settings.max_gain_factor,
                                        old.ki*settings.max_gain_factor)
                        blended=PI(float(old.kp*(1-alpha)+safe_kp*alpha),
                                   float(old.ki*(1-alpha)+safe_ki*alpha))
                        candidate=self.adapter.convert_physical_pi(
                            candidate,spec.loop,axis,blended,spec.dq_mode)
                    if candidate==current:continue
                    self.adapter.safe_stop()
                    self.adapter.apply_candidate(candidate,approved=True)
                    if not self.adapter.verify_written_pi(candidate):
                        raise RuntimeError("PI write readback mismatch")
                    trials+=1
                    try:
                        score,metrics=_measure(self.adapter,spec,stage.safety,axes)
                    except BaseException:
                        self.adapter.safe_stop();self.adapter.restore(current)
                        raise
                    improved=(score<base_score*(1-settings.min_improvement) and
                              all(x.overshoot_percent<=ref.overshoot_percent+.1 and
                                  x.crossings<=ref.crossings for x,ref in zip(metrics,base_metrics)))
                    if improved:
                        accepted+=1;did_accept=True;base_score=score;base_metrics=metrics
                        reason="verified improvement"
                    else:
                        self.adapter.safe_stop()
                        self.adapter.restore(current)
                reports.append(StageReport(spec.loop.value,tuple(axes),proposal.method,
                              float(max(0.,base_score)),float(base_score),did_accept,
                              len(factors),proposal.suggested,reason))
            return FullAutoReport("completed",tuple(reports),trials,accepted)
        except BaseException:
            self.adapter.safe_stop()
            self.adapter.restore(original)
            raise
        finally:
            self.adapter.safe_stop()
