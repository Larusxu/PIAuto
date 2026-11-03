import numpy as np
import pytest
from piauto.adapters.bridge import StreamingMotorBridge
from piauto.adapters.streaming import ExperimentHandle,SampleBatch
from piauto.adapters.mock import MockMotorAdapter
from piauto.contracts import Experiment,SafetyEnvelope
from piauto.metrics import measure_step

SAFE=SafetyEnvelope(5,100,2,6,-10,10)

class Backend:
    def __init__(self):
        self.motor=MockMotorAdapter()
        self.stopped=False
        self.cursor=0
        self.bad_clock=False
        self.bad_readback=False
    def capabilities(self):return self.motor.capabilities()
    def read_pi(self):return self.motor.read_pi()
    def snapshot(self):return self.motor.snapshot()
    def apply_pi(self,g):return self.motor.apply_candidate(g,approved=True)
    def verify_pi(self,g):return self.motor.read_pi()==g and not self.bad_readback
    def restore(self,snap):return self.motor.restore(snap)
    def safe_stop(self):self.stopped=True;self.motor.safe_stop()
    def faults(self):return self.motor.faults()
    def prepare_experiment(self,spec,safety,signals):
        self.spec=spec;self.safety=safety
        return ExperimentHandle("test-a",0.)
    def start_experiment(self,handle):
        self.trace=self.motor.run_step(self.spec,self.safety)
        self.cursor=0;self.stopped=False
    def read_samples(self,handle,timeout_s):
        start=self.cursor;end=min(start+250,len(self.trace.timestamp_s))
        self.cursor=end
        t=self.trace.timestamp_s[start:end]
        return SampleBatch("dev1","motor1",handle.test_id,t,
                           {"target":self.trace.target[start:end],"actual":self.trace.actual[start:end]},
                           {"target":"rad","actual":"rad"},0,
                           "host_receive" if self.bad_clock else "device_clock",
                           not self.bad_clock,
                           {"command_time_s":0.,"complete":end==len(self.trace.timestamp_s)})
    def stop_experiment(self,handle):
        self.stopped=True

def test_complete_streaming_capture_and_metrics():
    b=Backend();bridge=StreamingMotorBridge(b)
    trace=bridge.run_step(Experiment(),SAFE)
    assert trace.metadata["test_id"]=="test-a"
    assert trace.command_time_s==0.
    assert measure_step(trace).quality.valid
    assert b.stopped

def test_reject_unverified_clock_even_for_mock():
    b=Backend();b.bad_clock=True
    with pytest.raises(RuntimeError,match="clock"):
        StreamingMotorBridge(b).run_step(Experiment(),SAFE)
    assert b.stopped

def test_bridge_pi_readback_check():
    b=Backend();bridge=StreamingMotorBridge(b)
    before=bridge.read_pi()
    bridge.apply_candidate(before,approved=True)
    b.bad_readback=True
    with pytest.raises(RuntimeError,match="readback"):
        bridge.apply_candidate(before,approved=True)
    assert b.stopped

def test_unapproved_candidate_cannot_write():
    b=Backend();bridge=StreamingMotorBridge(b)
    with pytest.raises(PermissionError):
        bridge.apply_candidate(bridge.read_pi(),approved=False)
