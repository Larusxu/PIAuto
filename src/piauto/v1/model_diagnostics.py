"""Time-domain model diagnostics. No physical PI gain mapping is inferred."""
from __future__ import annotations
from ..contracts import Trace
from ..modeling import fit_closed_loop


def diagnose_step(trace:Trace)->dict:
    y=trace.actual;t=trace.timestamp_s
    pre=y[t<trace.command_time_s]
    post=trace.target[t>=trace.command_time_s]
    if len(pre)<3 or len(post)<8:raise ValueError("insufficient trace")
    return fit_closed_loop(t,y,start=float(pre.mean()),step=float(post[-1]-pre.mean()),command_s=trace.command_time_s)
