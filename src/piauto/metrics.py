"""Step metrics following explicit measured-target timestamps and ktune-style tracking statistics."""
from __future__ import annotations
import numpy as np
from .contracts import Trace, Metrics, Quality


def measure_step(trace: Trace, settle_band: float = 0.02) -> Metrics:
    t, r, y = trace.timestamp_s, trace.target, trace.actual
    reasons = []
    if len(t) < 12 or not np.isfinite(t).all() or not np.isfinite(r).all() or not np.isfinite(y).all():
        reasons.append("too_few_or_nonfinite_samples")
    if len(t) > 1 and (np.diff(t) <= 0).any():
        reasons.append("nonmonotonic_timestamps")
    if trace.dropped_frames > max(2, int(0.02*max(1, len(t)))):
        reasons.append("excessive_dropped_frames")
    if len(t) > 1 and (np.diff(t) > 0).all():
        dt = np.diff(t)
        fs = 1.0/np.median(dt)
        if np.max(dt) > 5*np.median(dt):
            reasons.append("large_sample_gap")
    else:
        fs = 0.0
    if len(t) and not (t[0] <= trace.command_time_s < t[-1]):
        reasons.append("missing_step_time")
    idx = int(np.searchsorted(t, trace.command_time_s, side="left")) if len(t) else 0
    before = y[:idx]
    after = y[idx:]
    if len(before) < 3 or len(after) < 8:
        reasons.append("missing_pre_or_post_step_samples")
    if len(before) and len(after):
        y0 = float(np.median(before))
        final_target = float(np.median(r[idx:]))
        step = final_target-y0
        if abs(step) < 1e-12:
            reasons.append("zero_or_unapplied_step")
        # Target must remain at its final level, else this is not a single step response.
        if np.ptp(r[idx:]) > max(1e-8, .01*abs(step)):
            reasons.append("target_not_constant_after_step")
    else:
        y0, final_target, step = 0.,0.,0.
    if reasons:
        q=Quality(False, tuple(dict.fromkeys(reasons)), fs)
        return Metrics(float("nan"),float("nan"),None,None,float("nan"),float("nan"),0,float("nan"),q)
    direction = np.sign(step)
    progress = direction*(after-y0)
    excursion = progress-abs(step)
    overshoot = float(max(0.,np.max(excursion)))
    overshoot_pct=100*overshoot/abs(step)
    times=t[idx:]-trace.command_time_s
    def crossing(threshold):
        hit=np.where(progress>=threshold*abs(step))[0]
        if not len(hit):return None
        k=int(hit[0])
        if k==0:return float(times[0])
        x0,x1=progress[k-1],progress[k]
        return float(times[k-1]+(threshold*abs(step)-x0)/(x1-x0)*(times[k]-times[k-1])) if x1!=x0 else float(times[k])
    t10,t90=crossing(.1),crossing(.9)
    rise=(t90-t10) if t10 is not None and t90 is not None else None
    within=np.abs(after-final_target)<=settle_band*abs(step)
    settling=None
    outside=np.where(~within)[0]
    if len(outside)==0:
        settling=float(times[0])
    elif outside[-1]+1<len(times):
        settling=float(times[outside[-1]+1])
    tail=max(3,int(len(after)*.1))
    steady=float(final_target-np.mean(after[-tail:]))
    rms=float(np.sqrt(np.mean((r[idx:]-after)**2)))
    err=after-final_target
    deadband=max(.001*abs(step),1e-10)
    signs=np.where(err>deadband,1,np.where(err < -deadband,-1,0))
    significant=signs[signs!=0]
    crossings=int(np.count_nonzero(np.diff(significant)!=0)) if len(significant) else 0
    peak=float(after[np.argmax(progress)])
    return Metrics(overshoot,overshoot_pct,rise,settling,steady,rms,crossings,peak,Quality(True,(),fs))
