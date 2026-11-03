import pytest
from piauto.adapters.model_mock import ModelMockMotorAdapter
from piauto.contracts import Experiment,SafetyEnvelope,Loop,DQMode
from piauto.v2.designer import design_current,design_velocity,design_position
from piauto.v2.full_auto import V2ModelAutoTuner,ModelSettings
from piauto.v2.staged import TuningStage

SAFE=SafetyEnvelope(5,100,2,6,-10,10)

def test_all_three_loop_models_generate_physical_pi():
    a=ModelMockMotorAdapter()
    current={axis:a.collect_model_evidence(Loop.CURRENT,axis,SAFE) for axis in ("d","q")}
    design=design_current(current,mode=DQMode.SHARED,bandwidth_hz=15,pwm_hz=20000)
    assert design.suggested["d"]==design.suggested["q"]
    assert design.suggested["q"].ki>0
    d=design_current({"d":current["d"]},mode=DQMode.INDEPENDENT,
                     selected_axis="d",bandwidth_hz=15,pwm_hz=20000)
    assert list(d.suggested)==["d"]
    evidence=a.collect_model_evidence(Loop.VELOCITY,"q",SAFE)
    v=design_velocity(evidence,lambda_s=.5)
    assert v.suggested["velocity"].kp==pytest.approx(.05,rel=.1)
    p=design_position(a.collect_model_evidence(Loop.POSITION,"q",SAFE),
                      baseline=a.read_pi().position)
    assert p.suggested["position"].kp>0

def test_evidence_provenance_enforced():
    a=ModelMockMotorAdapter()
    evidence=a.collect_model_evidence(Loop.VELOCITY,"q",SAFE)
    with pytest.raises(PermissionError):
        design_current({"d":evidence},mode=DQMode.INDEPENDENT,
                       selected_axis="d",bandwidth_hz=10,pwm_hz=10000)

def test_end_to_end_all_layers_shared():
    a=ModelMockMotorAdapter()
    stages=[TuningStage(Experiment(loop=loop,dq_mode=DQMode.SHARED),SAFE,1)
            for loop in (Loop.CURRENT,Loop.VELOCITY,Loop.POSITION)]
    out=V2ModelAutoTuner(a).run(stages,authorized=True)
    assert out.status=="completed"
    assert len(out.stages)==3
    assert (Loop.CURRENT,"d") in a.evidence_requests
    assert (Loop.CURRENT,"q") in a.evidence_requests
    assert a.stopped

def test_missing_authorization_is_safe():
    a=ModelMockMotorAdapter()
    with pytest.raises(PermissionError):
        V2ModelAutoTuner(a).run([TuningStage(Experiment(),SAFE,1)])
    assert a.stopped

def test_pi_readback_failure_rolls_back():
    a=ModelMockMotorAdapter();before=a.read_pi();a.fail_readback=True
    with pytest.raises(RuntimeError,match="readback"):
        V2ModelAutoTuner(a).run([TuningStage(Experiment(loop=Loop.CURRENT),SAFE,1)],
                                authorized=True)
    assert a.read_pi()==before and a.stopped
