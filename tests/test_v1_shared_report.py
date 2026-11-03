import json
import numpy as np
import pytest
from piauto.adapters.mock import MockMotorAdapter
from piauto.contracts import Experiment,SafetyEnvelope,Loop,DQMode
from piauto.v1 import V1SemiAutoTuner
from piauto.reporting import TuningSessionReport,trace_csv

SAFE=SafetyEnvelope(5,100,2,6,-10,10)

def test_v1_shared_requires_both_axes_live():
    class Track(MockMotorAdapter):
        def __init__(self):super().__init__();self.axes=[]
        def run_step(self,spec,safety):
            self.axes.append(spec.axis)
            return super().run_step(spec,safety)
    a=Track()
    spec=Experiment(loop=Loop.CURRENT,dq_mode=DQMode.SHARED)
    v=V1SemiAutoTuner(a)
    v.analyze_and_propose(spec,SAFE)
    assert a.axes==["d","q"]
    v.verify_approved_candidate(approved=True)
    assert a.axes==["d","q","d","q"]

def test_shared_offline_single_axis_refused():
    a=MockMotorAdapter()
    tr=a.run_step(Experiment(),SAFE)
    with pytest.raises(ValueError,match="both axes"):
        V1SemiAutoTuner(a).analyze_and_propose(
            Experiment(loop=Loop.CURRENT,dq_mode=DQMode.SHARED),SAFE,offline_trace=tr)

def test_report_is_structured_json_and_trace_csv():
    a=MockMotorAdapter();spec=Experiment()
    trace=a.run_step(spec,SAFE)
    report=TuningSessionReport("V1","device1","M9","accepted",
                               {"score":1.2},a.snapshot(),{"dropped":0})
    parsed=json.loads(report.to_json())
    assert parsed["config_snapshot"]["q"]["kp"]>0
    assert "timestamp_s,target,actual" in trace_csv(trace)
