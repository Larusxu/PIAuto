import pytest
from piauto.adapters.mock import MockMotorAdapter
from piauto.contracts import Experiment,SafetyEnvelope,Loop,DQMode
from piauto.v2.staged import TuningStage,V2StagedTuner
SAFE=SafetyEnvelope(5,100,2,6,-10,10)

def test_staged_cascade_mock():
    motor=MockMotorAdapter()
    stages=[
      TuningStage(Experiment(loop=Loop.CURRENT,dq_mode=DQMode.SHARED),SAFE,max_rounds=1),
      TuningStage(Experiment(loop=Loop.VELOCITY),SAFE,max_rounds=1),
      TuningStage(Experiment(loop=Loop.POSITION),SAFE,max_rounds=1)]
    result=V2StagedTuner(motor).autotune(stages,authorized=True)
    assert result.status=="completed"
    assert len(result.stages)==3 and motor.stopped

def test_invalid_stage_order_rejected():
    motor=MockMotorAdapter()
    stages=[TuningStage(Experiment(loop=Loop.POSITION),SAFE),
            TuningStage(Experiment(loop=Loop.CURRENT),SAFE)]
    with pytest.raises(ValueError):
        V2StagedTuner(motor).autotune(stages,authorized=True)
