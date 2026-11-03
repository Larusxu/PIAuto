"""Position-loop plant and PI design from separately validated INNER velocity tracking.

Position target->position actual cannot by itself identify an inner velocity
model. The test inputs here are commanded velocity and measured rotor velocity;
the position loop is OUT of cascade during excitation.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares, minimize
from .sysid import _simulate

@dataclass(frozen=True)
class PositionPlant:
    velocity_gain: float
    velocity_tau_s: float
    rmse: float
    fit_quality: str

@dataclass(frozen=True)
class PositionPIProposal:
    kp: float
    ki: float
    estimated_overshoot_pct: float
    model_score: float
    provenance: str = "continuous position PI; requires hardware codec and verification"

def fit_inner_velocity_tracking(t, velocity_command, velocity_actual, *,
                                inner_loop_verified: bool = False) -> PositionPlant:
    if not inner_loop_verified:
        raise PermissionError("inner velocity loop must be verified and position loop bypassed")
    t,u,y=[np.asarray(v,dtype=float) for v in (t,velocity_command,velocity_actual)]
    if len(t)<40 or len(t)!=len(u) or len(t)!=len(y) or not all(np.isfinite(v).all() for v in (t,u,y)):
        raise ValueError("invalid velocity tracking samples")
    dt=np.diff(t)
    if np.any(dt<=0) or np.max(dt)>2.5*np.median(dt) or np.ptp(u)<1e-7 or np.ptp(y)<1e-7:
        raise ValueError("invalid excitation or timebase")
    gain_guess=(y[-1]-y[0])/(u[-1]-u[0]) if abs(u[-1]-u[0])>0.2*np.ptp(u) else 1.
    tau_min=max(np.median(dt),1e-5);tau_max=(t[-1]-t[0])*5
    ls=least_squares(lambda p:(_simulate(t,u,p[0],p[1],p[2])-y)/np.ptp(y),
                     x0=[max(.01,float(gain_guess)),max(2*tau_min,(t[-1]-t[0])*.15),y[0]],
                     bounds=([1e-6,tau_min,y[0]-np.ptp(y)],
                             [1000,tau_max,y[0]+np.ptp(y)]),
                     loss="soft_l1",max_nfev=250)
    K,tau,offset=map(float,ls.x)
    rmse=float(np.sqrt(np.mean((_simulate(t,u,K,tau,offset)-y)**2)))
    quality="valid" if ls.success and rmse/np.ptp(y)<.12 and tau<(t[-1]-t[0])*.8 else "rejected"
    return PositionPlant(K,tau,rmse,quality)

def _response(plant:PositionPlant, kp:float, ki:float, horizon:float):
    import control as ct
    s=ct.tf("s")
    G=plant.velocity_gain/(s*(1+plant.velocity_tau_s*s))
    C=kp+ki/s
    loop=C*G
    if np.any(np.real(ct.poles(ct.feedback(loop,1)))>=0):
        raise ValueError("unstable model")
    t=np.linspace(0,horizon,600)
    result=ct.step_response(ct.feedback(loop,1),T=t)
    return t,np.asarray(result.outputs).reshape(-1)

def optimize_position_pi(plant:PositionPlant, baseline_kp:float, baseline_ki:float, *,
                         horizon_s:float=1.5,max_gain_factor:float=2.)->PositionPIProposal:
    if plant.fit_quality!="valid" or plant.velocity_gain<=0 or plant.velocity_tau_s<=0:
        raise ValueError("invalid velocity tracking model")
    if baseline_kp<=0 or baseline_ki<0 or not 1 < max_gain_factor<=3 or horizon_s<=0:
        raise ValueError("invalid PI baseline or safety bounds")
    # Routh necessary condition for cubic: Kp > tau*Ki
    def objective(p):
        kp,ki=p
        if kp<=plant.velocity_tau_s*ki*1.05:
            return 1e6
        try:
            t,y=_response(plant,kp,ki,horizon_s)
            overshoot=max(0.,float(np.max(y)-1))
            rms=float(np.sqrt(np.mean((1-y)**2)))
            ss=float(np.mean(np.abs(1-y[-50:])))
            return rms+.2*overshoot+ss
        except Exception:
            return 1e6
    bounds=[(baseline_kp/max_gain_factor,baseline_kp*max_gain_factor),
            (0.,max(baseline_ki*max_gain_factor,baseline_kp/plant.velocity_tau_s*.2))]
    candidate=minimize(objective,[baseline_kp,baseline_ki],method="Powell",bounds=bounds,
                       options={"maxiter":60,"xtol":1e-4})
    if not np.isfinite(candidate.fun) or candidate.fun>=1e5:
        raise ValueError("no stable bounded position PI found")
    kp,ki=map(float,candidate.x)
    _,y=_response(plant,kp,ki,horizon_s)
    return PositionPIProposal(kp,ki,max(0.,float((max(y)-1)*100)),float(candidate.fun))
