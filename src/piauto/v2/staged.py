"""V2 cascade planner: current -> velocity -> position, with atomic rollback.
Each stage needs its OWN safety envelope/physical units and verified MCU
capabilities. Skipping prevalidated inner loops is allowed by the caller.
"""
from dataclasses import dataclass
from typing import Sequence
from ..contracts import Experiment,SafetyEnvelope,Loop
from ..adapters.base import MotorAdapter
from .workflow import V2AutoTuner,AutoResult

_ORDER={Loop.CURRENT:0,Loop.VELOCITY:1,Loop.POSITION:2}

@dataclass(frozen=True)
class TuningStage:
    experiment:Experiment
    safety:SafetyEnvelope
    max_rounds:int=3

@dataclass(frozen=True)
class StagedResult:
    stages:tuple[AutoResult,...]
    status:str

class V2StagedTuner:
    def __init__(self,adapter:MotorAdapter):self.adapter=adapter

    def autotune(self,stages:Sequence[TuningStage],*,authorized:bool=False)->StagedResult:
        if not authorized:raise PermissionError("explicit V2 full-stage authorization required")
        if not stages:raise ValueError("no stages requested")
        order=[_ORDER[s.experiment.loop] for s in stages]
        if order!=sorted(order):
            raise ValueError("stage order must be current -> velocity -> position")
        keys=[(st.experiment.loop,st.experiment.axis if st.experiment.loop==Loop.CURRENT and
               st.experiment.dq_mode==DQMode.INDEPENDENT else "*") for st in stages]
        if len(set(keys))!=len(keys):
            raise ValueError("duplicate tuning stage or repeated current axis")
        if self.adapter.faults():raise RuntimeError("hardware faults")
        for stage in stages:
            stage.safety.validate(stage.experiment)
            if not 1<=stage.max_rounds<=8:raise ValueError("invalid max_rounds")
        initial=self.adapter.snapshot()
        results=[]
        try:
            for stage in stages:
                result=V2AutoTuner(self.adapter).autotune(stage.experiment,stage.safety,
                                                         authorized=True,max_rounds=stage.max_rounds)
                results.append(result)
            return StagedResult(tuple(results),"completed")
        except BaseException:
            self.adapter.safe_stop();self.adapter.restore(initial)
            raise
        finally:
            self.adapter.safe_stop()
