from piauto.adapters.model_mock import ModelMockMotorAdapter
from piauto.contracts import Loop,DQMode,SafetyEnvelope
from piauto.v2.planner import choose_cascade
from piauto.v2.full_auto import V2ModelAutoTuner
from piauto.v2.staged import TuningStage
from piauto.contracts import Experiment

SAFE=SafetyEnvelope(5,100,2,6,-10,10)

def test_independent_current_d_and_q_in_one_cascade():
    adapter=ModelMockMotorAdapter()
    plan=choose_cascade(target=Loop.POSITION,validated=(),
                        envelopes={loop:SAFE for loop in Loop},
                        capabilities=adapter.capabilities(),
                        dq_mode=DQMode.INDEPENDENT,max_rounds=1)
    assert [(s.experiment.loop,s.experiment.axis) for s in plan.stages]==[
        (Loop.CURRENT,"d"),(Loop.CURRENT,"q"),
        (Loop.VELOCITY,"q"),(Loop.POSITION,"q")]
    result=V2ModelAutoTuner(adapter).run(plan.stages,authorized=True)
    assert result.status=="completed"
    assert len(result.stages)==4
    assert set(x for x in adapter.evidence_requests if x[0]==Loop.CURRENT)=={
        (Loop.CURRENT,"d"),(Loop.CURRENT,"q")}

def test_duplicate_current_axis_refused():
    a=ModelMockMotorAdapter()
    one=TuningStage(Experiment(loop=Loop.CURRENT,axis="d",dq_mode=DQMode.INDEPENDENT),SAFE,1)
    import pytest
    with pytest.raises(ValueError,match="duplicate"):
        V2ModelAutoTuner(a).run([one,one],authorized=True)
