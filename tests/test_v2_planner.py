import pytest
from piauto.adapters.mock import MockMotorAdapter
from piauto.contracts import Loop,DQMode,SafetyEnvelope
from piauto.v2.planner import choose_cascade,choose_chirp

def _safety():
    return SafetyEnvelope(5.,100.,2.,5.,-10.,10.)

def test_choose_only_unverified_cascade_layers():
    caps=MockMotorAdapter().capabilities()
    plan=choose_cascade(target=Loop.POSITION,validated=(Loop.CURRENT,),
                        capabilities=caps,
                        envelopes={Loop.VELOCITY:_safety(),Loop.POSITION:_safety()})
    assert [s.experiment.loop for s in plan.stages]==[Loop.VELOCITY,Loop.POSITION]
    assert plan.skipped_verified==("current",)

def test_choose_three_layers_shared():
    cap=MockMotorAdapter().capabilities()
    plan=choose_cascade(target=Loop.POSITION,validated=(),
                        capabilities=cap,envelopes={loop:_safety() for loop in Loop},
                        dq_mode=DQMode.SHARED,max_rounds=2)
    assert len(plan.stages)==3
    assert plan.stages[0].experiment.dq_mode==DQMode.SHARED

def test_choose_safe_chirp():
    cap=MockMotorAdapter().capabilities()
    p,signal=choose_chirp(capabilities=cap,envelope=_safety(),center=0,
                          duration_s=1.0,f_min_hz=2,f_requested_hz=12)
    assert len(signal.timestamp_s)>50
    assert abs(signal.position_target).max()<=p.amplitude+1e-12
    assert p.f_end_hz<=cap.telemetry_hz/20

def test_choose_chirp_reject_bad_center():
    cap=MockMotorAdapter().capabilities()
    with pytest.raises(ValueError):
        choose_chirp(capabilities=cap,envelope=_safety(),center=12,duration_s=1)
