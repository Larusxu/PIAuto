"""Bounded V2 sequential tuning; real motors require validated MCU safety.

Model-assisted fit and frequency response are separate offline tools. A fitted
closed-loop surrogate must not be used to predict changing controller gains.
"""
from __future__ import annotations
from dataclasses import dataclass
from ..contracts import Experiment,SafetyEnvelope,DQMode,Loop
from ..adapters.base import MotorAdapter
from ..metrics import measure_step
from ..v1.candidate import propose

@dataclass(frozen=True)
class AutoResult:
    rounds:int
    accepted:int
    final_metrics:object
    history:tuple[dict,...]

class V2AutoTuner:
    def __init__(self,adapter:MotorAdapter):self.adapter=adapter
    def autotune(self,spec:Experiment,safety:SafetyEnvelope,*,authorized:bool=False,max_rounds:int=3)->AutoResult:
        if not authorized:raise PermissionError("explicit bounded auto tuning authorization required")
        if not 1<=max_rounds<=8:raise ValueError("max_rounds must be 1..8")
        safety.validate(spec)
        cap=self.adapter.capabilities()
        if not (cap.safe_stop and cap.rollback and cap.verified_telemetry and cap.verified_command_scheduling
                and cap.supports(spec.loop,spec.axis)):
            raise RuntimeError("adapter lacks verified MCU safety/telemetry/scheduling for V2")
        if self.adapter.faults():raise RuntimeError("hardware fault")
        initial=self.adapter.snapshot()
        history=[];accepted=0
        try:
            for round_index in range(max_rounds):
                current=self.adapter.read_pi()
                before=measure_step(self.adapter.run_step(spec,safety),spec.settle_band)
                if not before.quality.valid:raise RuntimeError("baseline telemetry invalid")
                candidate=propose(current,spec,before)
                if candidate.candidate==current:break
                self.adapter.safe_stop()
                self.adapter.apply_candidate(candidate.candidate,approved=True)
                after=measure_step(self.adapter.run_step(spec,safety),spec.settle_band)
                passed=(after.quality.valid and after.overshoot_percent<=before.overshoot_percent+0.1
                        and after.crossings<=before.crossings and after.rms_error<before.rms_error*.99)
                history.append({"round":round_index+1,"baseline_rms":before.rms_error,
                                "candidate_rms":after.rms_error,"accepted":passed})
                if not passed:
                    self.adapter.safe_stop();self.adapter.restore(current)
                    break
                accepted+=1
            metrics=measure_step(self.adapter.run_step(spec,safety),spec.settle_band)
            return AutoResult(len(history),accepted,metrics,tuple(history))
        except BaseException:
            self.adapter.safe_stop()
            self.adapter.restore(initial)
            raise
        finally:
            self.adapter.safe_stop()
