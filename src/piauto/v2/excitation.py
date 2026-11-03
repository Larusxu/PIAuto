"""Offline sine/chirp plans, using SciPy and patterns from ktune/foc-sysid.
NOT a real motor command scheduler. Adapter must separately validate timing,
mechanics, current, bus arbitration and independent emergency stop.
"""
from dataclasses import dataclass
import numpy as np
from scipy.signal import chirp

@dataclass(frozen=True)
class ExcitationPlan:
    kind:str
    center:float
    amplitude:float
    duration_s:float
    sample_hz:float
    f_start_hz:float
    f_end_hz:float
    position_min:float
    position_max:float
    speed_max:float

@dataclass(frozen=True)
class PlannedWaveform:
    timestamp_s:np.ndarray
    position_target:np.ndarray
    estimated_peak_speed:float
    kind:str

def generate_waveform(plan:ExcitationPlan)->PlannedWaveform:
    if plan.kind not in ("sine","chirp"):
        raise ValueError("kind must be sine or chirp")
    values=[plan.center,plan.amplitude,plan.duration_s,plan.sample_hz,
            plan.f_start_hz,plan.f_end_hz,plan.position_min,plan.position_max,plan.speed_max]
    if not np.isfinite(values).all():raise ValueError("nonfinite waveform settings")
    if not(plan.amplitude>0 and plan.duration_s>0 and plan.sample_hz>0 and
           0<plan.f_start_hz<=plan.f_end_hz and plan.speed_max>0):
        raise ValueError("invalid waveform parameters")
    if plan.sample_hz<20*plan.f_end_hz:
        raise ValueError("sampling must be >=20x max test frequency")
    if plan.center-plan.amplitude<plan.position_min or plan.center+plan.amplitude>plan.position_max:
        raise ValueError("position limits exceeded")
    if plan.duration_s*plan.sample_hz>250000:
        raise ValueError("too many waveform samples")
    estimated_speed=2*np.pi*plan.f_end_hz*plan.amplitude
    if estimated_speed>plan.speed_max:
        raise ValueError("speed bound exceeded")
    t=np.arange(0,plan.duration_s,1/plan.sample_hz)
    y=(np.sin(2*np.pi*plan.f_start_hz*t) if plan.kind=="sine" else
       chirp(t,f0=plan.f_start_hz,f1=plan.f_end_hz,t1=plan.duration_s,method="linear"))
    return PlannedWaveform(t,plan.center+plan.amplitude*y,float(estimated_speed),plan.kind)
