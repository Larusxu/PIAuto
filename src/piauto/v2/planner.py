"""V2 chip-neutral safe experiment and cascade planning.

Selection is deterministic and bounded by host-declared AND device-verified
capabilities. Driver adapter remains responsible for actual MCU motion safety.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Mapping,Iterable
from ..contracts import Loop,DQMode,Capabilities,Experiment,SafetyEnvelope
from .staged import TuningStage
from .excitation import ExcitationPlan,generate_waveform

_ORDER=(Loop.CURRENT,Loop.VELOCITY,Loop.POSITION)

@dataclass(frozen=True)
class AutoPlan:
    stages: tuple[TuningStage,...]
    skipped_verified: tuple[str,...]
    rationale: str

def choose_cascade(*,target:Loop,validated:Iterable[Loop],
                   capabilities:Capabilities,
                   envelopes:Mapping[Loop,SafetyEnvelope],
                   dq_mode:DQMode=DQMode.SHARED,axis:str="q",
                   max_rounds:int=2,duration_s:float=1.5)->AutoPlan:
    validated=set(validated)
    if not 1<=max_rounds<=8 or duration_s<=0:
        raise ValueError("invalid auto plan budget")
    selected=[];skipped=[]
    for loop in _ORDER[:_ORDER.index(target)+1]:
        if loop in validated:
            skipped.append(loop.value)
            continue
        if loop not in envelopes:
            raise ValueError(f"missing safety envelope for {loop.value}")
        axes=("d","q") if loop==Loop.CURRENT and dq_mode==DQMode.SHARED else (axis,)
        if not all(capabilities.supports(loop,a) for a in axes):
            raise RuntimeError(f"no validated test capability for {loop.value}: {axes}")
        e=envelopes[loop]
        safe_step=min(e.max_step*.2,max(e.max_step*.05,1e-9))
        spec=Experiment(loop=loop,axis=axis,dq_mode=dq_mode,start=0.,
                        step=safe_step,duration_s=min(duration_s,e.max_duration_s))
        e.validate(spec)
        selected.append(TuningStage(spec,e,max_rounds=max_rounds))
    if not selected:
        return AutoPlan(tuple(),tuple(skipped),"all requested stages already validated")
    return AutoPlan(tuple(selected),tuple(skipped),
                    "unvalidated inner loops before outer loop; bounded step excitations")

def choose_chirp(*,capabilities:Capabilities,envelope:SafetyEnvelope,
                 center:float,duration_s:float,f_min_hz:float=1.,
                 f_requested_hz:float=15.,amplitude:float|None=None):
    if not capabilities.chirp or not capabilities.verified_telemetry:
        raise RuntimeError("unverified chirp/telemetry capability")
    if capabilities.telemetry_hz<=0:
        raise RuntimeError("missing measured telemetry rate")
    max_freq=min(float(f_requested_hz),float(capabilities.telemetry_hz)/20.)
    if not 0<f_min_hz<=max_freq:
        raise ValueError("insufficient sample rate for requested frequency range")
    available=min(center-envelope.min_position,envelope.max_position-center)
    if available<=0:
        raise ValueError("center position outside safe motion range")
    safe_amplitude=min(float(amplitude) if amplitude is not None else available*.1,
                       available*.2,envelope.max_speed/(2*3.141592653589793*max_freq)*.8)
    if safe_amplitude<=0:
        raise ValueError("no safe excitation amplitude")
    plan=ExcitationPlan(kind="chirp",center=center,amplitude=safe_amplitude,
                        duration_s=min(duration_s,envelope.max_duration_s),
                        sample_hz=capabilities.telemetry_hz,
                        f_start_hz=f_min_hz,f_end_hz=max_freq,
                        position_min=envelope.min_position,
                        position_max=envelope.max_position,speed_max=envelope.max_speed)
    return plan,generate_waveform(plan)
