"""Conservative local candidates, explicitly NOT a physical PI model inversion."""
from __future__ import annotations
from dataclasses import replace
from scipy.optimize import minimize
import numpy as np
from ..contracts import Experiment, Gains, Loop, PI, Metrics, Proposal


def propose(gains:Gains,spec:Experiment,metrics:Metrics)->Proposal:
    if not metrics.quality.valid:raise ValueError("invalid trace cannot produce PI candidate")
    old=gains.loop_gain(spec.loop,spec.axis)
    # A single step trace cannot estimate partial derivatives wrt gain changes.
    # Generate a bounded and explicitly heuristic trial to be evaluated on hardware.
    if metrics.overshoot_percent>5 or metrics.crossings>1:
        factors=np.array([.93,.90]);reason="Overshoot/oscillation: conservative trial lowering both gains; re-measure required"
    elif metrics.settling_time_s is None:
        factors=np.array([.95,.90]);reason="No observed settling: reduce aggressive control; diagnosis may require inner-loop test"
    elif abs(metrics.steady_error)>abs(spec.step)*.02:
        factors=np.array([1.,1.05]);reason="Steady error: small integral gain trial; check saturation and feedback first"
    else:
        factors=np.array([1.,1.]);reason="Baseline meets coarse criteria: keep PI, no change recommended"
    old_vec=np.array([old.kp,old.ki]);target=old_vec*factors
    # scipy is the V1 bounded candidate calculation base; objective regularizes deviation.
    bounds=[(max(0.,v*.85),v*1.10) if v else (0.,0.) for v in old_vec]
    fit=minimize(lambda x:np.sum((x-target)**2)+.02*np.sum((x-old_vec)**2),
                 old_vec,bounds=bounds,method="L-BFGS-B")
    pi=PI(*[float(x) for x in fit.x])
    candidate=gains.set_loop(spec.loop,pi,spec.axis,spec.dq_mode)
    return Proposal(gains,candidate,metrics,reason,"heuristic / unvalidated on hardware",{})
