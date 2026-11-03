"""V2 safe, bounded, per-loop automatic PI optimization.
Uses measured step-cost (ktune-style metrics), NOT inverted closed-loop PI.
Hardware requires verified MCU safe stop, rollback, telemetry and arbitration.
"""
from __future__ import annotations
from dataclasses import dataclass
from ..contracts import Experiment,SafetyEnvelope,DQMode,Loop,PI,Gains
from ..adapters.base import MotorAdapter
from ..metrics import measure_step

@dataclass(frozen=True)
class AutoResult:
    rounds:int
    accepted:int
    final_metrics:object
    history:tuple[dict,...]

def _score(m,step):
    if not m.quality.valid or m.settling_time_s is None:return float("inf")
    return (m.rms_error/max(abs(step),1e-9)+.02*m.overshoot_percent+
            .04*m.crossings+.03*m.settling_time_s)

def _trial(gains:Gains,spec:Experiment,kp_factor:float,ki_factor:float):
    pi=gains.loop_gain(spec.loop,spec.axis)
    return gains.set_loop(spec.loop,PI(pi.kp*kp_factor,pi.ki*ki_factor),
                          axis=spec.axis,mode=spec.dq_mode)

class V2AutoTuner:
    def __init__(self,adapter:MotorAdapter):self.adapter=adapter

    def _check(self,spec:Experiment,safety:SafetyEnvelope):
        safety.validate(spec)
        cap=self.adapter.capabilities()
        axes=("d","q") if spec.loop==Loop.CURRENT and spec.dq_mode==DQMode.SHARED else (spec.axis,)
        if not(cap.safe_stop and cap.rollback and cap.verified_telemetry and cap.verified_command_scheduling):
            raise RuntimeError("V2 needs verified MCU safety, telemetry, scheduling and rollback")
        if any(not cap.supports(spec.loop,a) for a in axes):
            raise RuntimeError("required loop/axis has no validated test capability")
        if self.adapter.faults():raise RuntimeError("hardware fault")
        return axes

    def _assess(self,spec,safety,axes):
        scores=[];metrics=[]
        for axis in axes:
            s=Experiment(loop=spec.loop,axis=axis,dq_mode=spec.dq_mode,start=spec.start,
                         step=spec.step,duration_s=spec.duration_s,settle_band=spec.settle_band)
            if self.adapter.faults():raise RuntimeError("fault during test")
            m=measure_step(self.adapter.run_step(s,safety),s.settle_band)
            scores.append(_score(m,s.step));metrics.append(m)
        return max(scores),metrics

    def autotune(self,spec:Experiment,safety:SafetyEnvelope,*,authorized:bool=False,max_rounds:int=3)->AutoResult:
        if not authorized:raise PermissionError("explicit bounded auto tuning authorization required")
        if not 1<=max_rounds<=8:raise ValueError("max_rounds must be 1..8")
        axes=self._check(spec,safety)
        initial=self.adapter.snapshot()
        history=[];accepted=0;last_metrics=None
        try:
            baseline,old_metrics=self._assess(spec,safety,axes)
            if baseline==float("inf"):raise RuntimeError("invalid baseline trace")
            last_metrics=old_metrics[0]
            # Tiny coordinate steps rather than global/relay searches on real motors.
            factors=((.90,1.),(1.10,1.),(1.,.90),(1.,1.10))
            for idx in range(max_rounds):
                current=self.adapter.read_pi()
                kpf,kif=factors[idx%len(factors)]
                candidate=_trial(current,spec,kpf,kif)
                if candidate==current:continue
                self.adapter.safe_stop()
                self.adapter.apply_candidate(candidate,approved=True)
                try:
                    score,metrics=self._assess(spec,safety,axes)
                except BaseException:
                    self.adapter.safe_stop();self.adapter.restore(current)
                    raise
                improved=(score<baseline*.99 and all(m.quality.valid for m in metrics) and
                          all(m.overshoot_percent<=old.overshoot_percent+.1 and
                              m.crossings<=old.crossings
                              for m,old in zip(metrics,old_metrics)))
                history.append({"round":idx+1,"axes":list(axes),"before_cost":baseline,
                                "candidate_cost":score,"accepted":improved,
                                "gain_factors":[kpf,kif]})
                if improved:
                    accepted+=1;baseline=score;old_metrics=metrics;last_metrics=metrics[0]
                else:
                    self.adapter.safe_stop();self.adapter.restore(current)
            return AutoResult(len(history),accepted,last_metrics,tuple(history))
        except BaseException:
            self.adapter.safe_stop();self.adapter.restore(initial)
            raise
        finally:
            self.adapter.safe_stop()
