"""Physics-shaped offline evidence adapter for V2 integration tests only.

This module does not impersonate a real chip or a production HPM transport.
"""
from __future__ import annotations
import numpy as np
from ..contracts import Loop,DQMode,PI,Gains,SafetyEnvelope
from .mock import MockMotorAdapter
from ..v2.designer import ModelEvidence
from ..v2.current_model import simulate_rl
from ..v2.sysid import _simulate

class ModelMockMotorAdapter(MockMotorAdapter):
    def __init__(self):
        super().__init__()
        self.evidence_requests=[]
        self.fail_readback=False
        self.fail_axis=None
    def collect_model_evidence(self,loop:Loop,axis:str,safety:SafetyEnvelope)->ModelEvidence:
        if self.faults():raise RuntimeError("simulated motor fault")
        self.evidence_requests.append((loop,axis))
        if loop==Loop.CURRENT:
            t=np.arange(0,.5,.001);u=np.where(t<.05,0.,.5)
            y=simulate_rl(t,u,2. if axis=="d" else 2.2,
                          .06 if axis=="d" else .065,0.)
            return ModelEvidence(loop,axis,t,u,y,"effective_voltage",("V","A"),True,
                                 {"stationary_verified":True,"decoupling_verified":True})
        t=np.arange(0,3.,.01)
        u=np.where(t<.2,0.,.5)
        if loop==Loop.VELOCITY:
            return ModelEvidence(loop,axis,t,u,_simulate(t,u,8.,.2,0.),
                                 "current_to_speed",("A","rad/s"),True)
        return ModelEvidence(loop,axis,t,u,_simulate(t,u,1.,.075,0.),
                             "velocity_command_to_velocity",("rad/s","rad/s"),True,
                             {"inner_loop_verified":True})
    def convert_physical_pi(self,gains:Gains,loop:Loop,axis:str,pi:PI,mode:DQMode)->Gains:
        return gains.set_loop(loop,pi,axis=axis,mode=mode)
    def verify_written_pi(self,gains:Gains)->bool:
        return not self.fail_readback and self.read_pi()==gains
