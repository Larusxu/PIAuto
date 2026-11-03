import numpy as np
import pytest
from piauto.contracts import Trace,PI,Experiment,SafetyEnvelope,Loop,DQMode
from piauto.metrics import measure_step
from piauto.adapters.mock import MockMotorAdapter
from piauto.v1 import V1SemiAutoTuner
from piauto.v2 import V2AutoTuner
from piauto.adapters.tmc6460 import decode_pi_word,encode_pi_word

SAFE=SafetyEnvelope(5,100,2,6,-10,10)
def test_metrics_overshoot_and_negative_motion():
    t=np.linspace(-.1,1,301)
    for direction in (1,-1):
        x=np.maximum(t,0)
        y=direction*(1-np.exp(-8*x)*(np.cos(20*x)+.4*np.sin(20*x)))
        z=Trace(t,np.where(t<0,0,direction),y,command_time_s=0)
        m=measure_step(z)
        assert m.quality.valid
        assert m.overshoot_percent>1
        assert m.rms_error>0

def test_metrics_reject_bad_timestamps():
    t=np.linspace(-.1,1,301);t[100]=t[99]
    m=measure_step(Trace(t,np.where(t<0,0,1),np.where(t<0,0,1)))
    assert not m.quality.valid

def test_dq_shared_and_independent():
    g=MockMotorAdapter().read_pi()
    both=g.set_loop(Loop.CURRENT,PI(6,2),mode=DQMode.SHARED)
    assert both.d==both.q==PI(6,2)
    one=g.set_loop(Loop.CURRENT,PI(6,2),axis="d",mode=DQMode.INDEPENDENT)
    assert one.d==PI(6,2) and one.q==g.q

def test_v1_mock_approval_and_rollback():
    adapter=MockMotorAdapter();tuner=V1SemiAutoTuner(adapter)
    baseline=adapter.read_pi()
    proposal=tuner.analyze_and_propose(Experiment(step=1),SAFE)
    assert proposal.baseline==baseline
    with pytest.raises(PermissionError):tuner.verify_approved_candidate(approved=False)
    output=tuner.verify_approved_candidate(approved=True)
    assert output.state.value in ('accepted','rolled_back')
    tuner.rollback()
    assert adapter.read_pi()==baseline

def test_v2_requires_authorization():
    t=V2AutoTuner(MockMotorAdapter())
    with pytest.raises(PermissionError):t.autotune(Experiment(),SAFE)

def test_v2_mock_run():
    output=V2AutoTuner(MockMotorAdapter()).autotune(Experiment(),SAFE,authorized=True,max_rounds=2)
    assert output.rounds<=2

def test_tmc_pi_codec():
    for kp,ki in ((-1,100),(32767,-32768),(0,0)):
        assert decode_pi_word(encode_pi_word(kp,ki))==(kp,ki)
    with pytest.raises(ValueError):encode_pi_word(32768,0)
