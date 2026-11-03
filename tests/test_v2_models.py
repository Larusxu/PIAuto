import numpy as np
import pytest
from piauto.v2.current_model import fit_rl_current, current_pi_from_bandwidth, simulate_rl
from piauto.v2.position_model import fit_inner_velocity_tracking, optimize_position_pi
from piauto.v2.sysid import _simulate
from piauto.v2.model_optimizer import optimize_velocity_pi
from piauto.v2.sysid import VelocityPlant, PhysicalPI
from piauto.v2.model_workflow import analyze_and_optimize_velocity

def test_current_dq_model_and_pi():
    t=np.arange(0,.5,.001)
    v=np.where(t<.05,0.,1.)
    for axis in ("d","q"):
        actual=simulate_rl(t,v,2.,.06,0.)
        p=fit_rl_current(t,v,actual,axis=axis,stationary_verified=True,decoupling_verified=True)
        assert p.fit_quality=="valid"
        assert p.resistance_ohm==pytest.approx(2.,rel=.05)
        assert p.inductance_h==pytest.approx(.06,rel=.08)
        pi=current_pi_from_bandwidth(p,15.,pwm_hz=20000.)
        assert pi.kp>0 and pi.ki>0

def test_current_unsafe_data_rejected():
    t=np.arange(0,.5,.001);v=np.where(t<.05,0.,1.)
    with pytest.raises(PermissionError):
        fit_rl_current(t,v,np.zeros_like(t),axis="d")
    with pytest.raises(ValueError):
        current_pi_from_bandwidth(type("P",(),{"fit_quality":"valid","inductance_h":.06,"resistance_ohm":2.})(),
                                  500.,pwm_hz=1000.)

def test_position_model_and_optimization():
    pytest.importorskip("control")
    t=np.arange(0,2,.002)
    u=np.where(t<.15,0.,1.)
    y=_simulate(t,u,1.,.075,0.)
    p=fit_inner_velocity_tracking(t,u,y,inner_loop_verified=True)
    assert p.fit_quality=="valid"
    assert p.velocity_tau_s==pytest.approx(.075,rel=.1)
    out=optimize_position_pi(p,baseline_kp=3.,baseline_ki=.1)
    assert out.kp>0 and out.ki>=0 and np.isfinite(out.model_score)

def test_velocity_model_search():
    pytest.importorskip("control")
    plant=VelocityPlant(8.,.2,0.,0.,0.,"valid")
    proposal=optimize_velocity_pi(plant,PhysicalPI(.04,.15,.5),max_gain_factor=2)
    assert proposal.candidate.kp>0 and proposal.candidate.ki>0
    assert proposal.candidate_cost<proposal.baseline_cost

def test_full_velocity_offline_pipeline():
    pytest.importorskip("control")
    t=np.arange(0,3,.01);u=np.where(t<.3,0.,.5);y=_simulate(t,u,8.,.2,0.)
    result=analyze_and_optimize_velocity(t,u,y,PhysicalPI(.04,.15,.5),excitation_confirmed=True)
    assert result.plant.fit_quality=="valid"
    assert result.optimization.candidate_cost<result.optimization.baseline_cost
