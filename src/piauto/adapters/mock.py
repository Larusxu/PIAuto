"""Deterministic mock servo, not a guarantee for hardware gain response."""
from __future__ import annotations
import numpy as np
from .base import MotorAdapter
from ..contracts import PI,Gains,Capabilities,Experiment,SafetyEnvelope,Trace,Loop

class MockMotorAdapter(MotorAdapter):
    def __init__(self):
        self.gains=Gains(position=PI(5.,.8),velocity=PI(3.,.5),d=PI(2.,.3),q=PI(2.,.3))
        self.stopped=True
        self._faults=[]
    def capabilities(self):
        return Capabilities(True,True,True,True,True,True,True,500.,"mock-synchronized",True,True)
    def read_pi(self):return self.gains
    def snapshot(self):return self.gains
    def faults(self):return list(self._faults)
    def safe_stop(self):self.stopped=True
    def apply_candidate(self,gains,*,approved):
        if not approved:raise PermissionError("candidate requires user approval")
        if self._faults:raise RuntimeError("hardware fault")
        self.stopped=True
        self.gains=gains
    def restore(self,snapshot):
        self.safe_stop()
        if not isinstance(snapshot,Gains):raise TypeError("invalid snapshot")
        self.gains=snapshot
    def run_step(self,spec,safety):
        safety.validate(spec)
        if self._faults:raise RuntimeError("simulated fault")
        self.stopped=False
        fs=500
        t=np.arange(-.12,spec.duration_s,1/fs)
        # critically damped reference can be made more/less oscillatory by mock PI
        gain=self.gains.loop_gain(spec.loop,spec.axis)
        zeta=np.clip(.3+.055*gain.kp-.075*gain.ki,.18,1.6)
        wn=np.clip(4+2*gain.kp,3,60)
        ts=np.maximum(t,0)
        if zeta<1:
            w=wn*np.sqrt(1-zeta*zeta)
            resp=1-np.exp(-zeta*wn*ts)*(np.cos(w*ts)+(zeta/np.sqrt(1-zeta*zeta))*np.sin(w*ts))
        elif zeta==1:
            resp=1-np.exp(-wn*ts)*(1+wn*ts)
        else:
            a=wn*(zeta-np.sqrt(zeta*zeta-1));b=wn*(zeta+np.sqrt(zeta*zeta-1))
            resp=1-(b*np.exp(-a*ts)-a*np.exp(-b*ts))/(b-a)
        y=spec.start+spec.step*resp
        target=np.where(t<0,spec.start,spec.start+spec.step)
        self.stopped=True
        return Trace(t,target,y,signal=spec.loop.value,units="arb",command_time_s=0.,metadata={"source":"mock"})
