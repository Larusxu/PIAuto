"""Chip-neutral inner-current-loop plant identification and PI design.

Required: stationary rotor, *effective d/q voltage command* and measured same-axis
current. A closed-loop current TARGET is NOT a voltage excitation. Independent
Id and Iq traces required to justify independent fits.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares

@dataclass(frozen=True)
class RLPlant:
    axis: str
    resistance_ohm: float
    inductance_h: float
    time_constant_s: float
    rmse_a: float
    normalized_rmse: float
    fit_quality: str

@dataclass(frozen=True)
class CurrentPI:
    kp: float
    ki: float
    bandwidth_hz: float
    provenance: str = "continuous physical V/A and V/(A*s), NOT register coefficients"

def simulate_rl(t, v, R, L, initial_current):
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    i = np.empty_like(t)
    i[0] = initial_current
    for k in range(1, len(t)):
        decay = np.exp(-(t[k]-t[k-1])*R/L)
        i[k] = decay*i[k-1] + (1-decay)*v[k-1]/R
    return i

def fit_rl_current(t, effective_voltage, measured_current, *, axis: str,
                   stationary_verified: bool = False, decoupling_verified: bool = False) -> RLPlant:
    if axis not in ("d","q"):
        raise ValueError("axis must be d/q")
    if not stationary_verified or not decoupling_verified:
        raise PermissionError("must verify stationary rotor and decoupled effective voltage")
    t,v,i = [np.asarray(a,dtype=float) for a in (t,effective_voltage,measured_current)]
    if len(t)<40 or len(t)!=len(v) or len(t)!=len(i) or not all(np.isfinite(a).all() for a in (t,v,i)):
        raise ValueError("insufficient or invalid samples")
    dt=np.diff(t)
    if np.any(dt<=0) or np.max(dt)>2.5*np.median(dt):
        raise ValueError("invalid sampling timebase")
    if np.ptp(v)<1e-5 or np.ptp(i)<1e-6:
        raise ValueError("insufficient excitation or current response")
    if np.count_nonzero(abs(np.diff(v))>.025*np.ptp(v))>6:
        raise ValueError("a single limited voltage step is required")
    # Estimate R from steady-state final V/I when possible, then jointly fit R,L
    R0=abs(float(v[-1]/i[-1])) if abs(i[-1])>1e-6 else 1.0
    R0=float(np.clip(R0,.01,100.0))
    duration=float(t[-1]-t[0])
    tau0=max(float(np.median(dt))*3, duration*.1)
    scale=float(np.ptp(i))
    fit=least_squares(
        lambda log: (simulate_rl(t,v,np.exp(log[0]),np.exp(log[1]),float(i[0]))-i)/scale,
        np.log([R0,R0*tau0]),bounds=(np.log([1e-5,1e-8]),np.log([1e4,1e3])),
        loss="soft_l1",max_nfev=300
    )
    R,L=np.exp(fit.x)
    pred=simulate_rl(t,v,float(R),float(L),float(i[0]))
    error=float(np.sqrt(np.mean((pred-i)**2)))
    norm=error/scale
    tau=float(L/R)
    quality="valid" if fit.success and norm<.12 and dt.mean()<tau*1.25 and tau<duration else "rejected"
    return RLPlant(axis,float(R),float(L),tau,error,norm,quality)

def current_pi_from_bandwidth(plant: RLPlant, bandwidth_hz: float, *,
                              pwm_hz: float | None = None) -> CurrentPI:
    if plant.fit_quality!="valid" or not 0 < bandwidth_hz < 100000:
        raise ValueError("validated RL model and positive bandwidth required")
    if pwm_hz is not None and bandwidth_hz > pwm_hz / 20:
        raise ValueError("conservative test bandwidth must not exceed PWM/20")
    wc=2*np.pi*bandwidth_hz
    return CurrentPI(float(plant.inductance_h*wc),float(plant.resistance_ohm*wc),bandwidth_hz)
