import numpy as np
import pytest
from piauto.adapters.mock import MockMotorAdapter
from piauto.contracts import Experiment,Loop,DQMode,SafetyEnvelope
from piauto.v2 import V2AutoTuner
from piauto.v2.sysid import fit_velocity_plant,imc_velocity_pi,_simulate

SAFE=SafetyEnvelope(5,100,2,6,-10,10)

def test_velocity_plant_and_physical_imc():
    t=np.arange(0,3,.01)
    u=np.where(t<.3,0.,.5)
    y=_simulate(t,u,8.,.2,0.)
    model=fit_velocity_plant(t,u,y,open_loop_current_excitation=True)
    assert model.fit_quality=="valid"
    assert model.gain==pytest.approx(8,rel=.04)
    assert model.tau_s==pytest.approx(.2,rel=.06)
    pi=imc_velocity_pi(model,lambda_s=.5)
    assert pi.kp==pytest.approx(.05,rel=.07)
    assert pi.ki==pytest.approx(.25,rel=.07)

def test_sysid_rejects_unproven_or_bad_input():
    t=np.linspace(0,1,100)
    with pytest.raises(ValueError):fit_velocity_plant(t,np.ones(100),t)
    with pytest.raises(ValueError):fit_velocity_plant(t,np.ones(100),t,open_loop_current_excitation=True)

def test_shared_current_measures_both_axes():
    class TrackingAdapter(MockMotorAdapter):
        def __init__(self):super().__init__();self.axes=[]
        def run_step(self,spec,safety):
            self.axes.append(spec.axis)
            return super().run_step(spec,safety)
    a=TrackingAdapter()
    spec=Experiment(loop=Loop.CURRENT,axis="q",dq_mode=DQMode.SHARED)
    V2AutoTuner(a).autotune(spec,SAFE,authorized=True,max_rounds=2)
    assert set(a.axes)=={"d","q"}

def test_fault_restores_initial_pi():
    class FailOnce(MockMotorAdapter):
        def __init__(self):super().__init__();self.calls=0
        def run_step(self,spec,safety):
            self.calls+=1
            if self.calls==2:raise RuntimeError("simulated motor fault")
            return super().run_step(spec,safety)
    a=FailOnce();before=a.read_pi()
    with pytest.raises(RuntimeError):
        V2AutoTuner(a).autotune(Experiment(),SAFE,authorized=True,max_rounds=2)
    assert a.read_pi()==before and a.stopped
