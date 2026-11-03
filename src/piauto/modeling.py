"""Closed-loop surrogate models, explicitly not hardware plant identification."""
from __future__ import annotations
import numpy as np
from scipy.optimize import least_squares


def second_order_response(t: np.ndarray, start: float, step: float, command_s: float, zeta:float, wn:float):
    """Unit DC gain, underdamped/overdamped 2nd order closed-loop step model."""
    import control as ct
    system=ct.tf([wn*wn],[1,2*zeta*wn,wn*wn])
    ts=np.maximum(0,np.asarray(t,dtype=float)-command_s)
    _, y=ct.step_response(system,T=ts)
    return start+step*np.asarray(y)


def fit_closed_loop(t, y, *, start, step, command_s):
    """Fits zeta/wn to measured position, for diagnostics only.

    A fitted CLOSED-LOOP transfer function alone cannot predict the effect
    of replacing hardware controller gains; never use it as a PI plant model.
    """
    t=np.asarray(t,dtype=float)
    y=np.asarray(y,dtype=float)
    mask=t>=command_s
    tt=t[mask]
    yy=y[mask]
    if len(tt)<12 or np.ptp(tt)<.03:
        raise ValueError("insufficient post-step samples")
    fit=least_squares(lambda x:second_order_response(tt,start,step,command_s,*x)-yy,
                      x0=[.6,8.],bounds=([.08,.1],[3.,500.]),loss="soft_l1",max_nfev=120)
    predicted=second_order_response(tt,start,step,command_s,*fit.x)
    err=float(np.sqrt(np.mean((predicted-yy)**2)))
    return {"zeta":float(fit.x[0]),"omega_n_rad_s":float(fit.x[1]),"rmse":err,"rmse_to_step":err/abs(step),"model":"closed-loop-second-order-surrogate"}
