"""Reusable streaming device bridge: any chip backend -> normalized Tuning Trace.

The backend is the ONLY component allowed to know UART/RTMI framing, device
registers, timing windows and MCU motion limits. This bridge cannot enable a
motor and will reject unknown telemetry timing provenance and partial captures.
"""
from __future__ import annotations
import time
from typing import Protocol,runtime_checkable
import numpy as np
from .base import MotorAdapter
from .streaming import SignalSpec, SampleBatch, ExperimentHandle,validate_batch
from ..contracts import Gains,Capabilities,Experiment,SafetyEnvelope,Trace

@runtime_checkable
class StreamingBackend(Protocol):
    def capabilities(self)->Capabilities: ...
    def read_pi(self)->Gains: ...
    def snapshot(self)->object: ...
    def apply_pi(self,gains:Gains)->None: ...
    def verify_pi(self,gains:Gains)->bool: ...
    def restore(self,snapshot:object)->None: ...
    def safe_stop(self)->None: ...
    def faults(self)->list[str]: ...
    def prepare_experiment(self,spec:Experiment,safety:SafetyEnvelope,
                           signals:tuple[SignalSpec,...])->ExperimentHandle: ...
    def start_experiment(self,handle:ExperimentHandle)->None: ...
    def read_samples(self,handle:ExperimentHandle,timeout_s:float)->SampleBatch: ...
    def stop_experiment(self,handle:ExperimentHandle)->None: ...

class StreamingMotorBridge(MotorAdapter):
    """Adapter for scheduled full-batch or streaming experiments.

    For a test, the backend shall return batches containing both target and
    actual sampled on a COMMON device clock and a target-application timestamp.
    RTMI serial receive time on the host alone does not meet this requirement.
    """
    def __init__(self,backend:StreamingBackend,*,max_drop_fraction:float=.02,
                 timeout_margin_s:float=1.0):
        if not 0<=max_drop_fraction<=.1 or timeout_margin_s<=0:
            raise ValueError("invalid data bridge quality settings")
        self.backend=backend
        self.max_drop_fraction=max_drop_fraction
        self.timeout_margin_s=timeout_margin_s
    def capabilities(self):return self.backend.capabilities()
    def read_pi(self):return self.backend.read_pi()
    def snapshot(self):return self.backend.snapshot()
    def safe_stop(self):return self.backend.safe_stop()
    def faults(self):return self.backend.faults()
    def apply_candidate(self,gains:Gains,*,approved:bool)->None:
        if not approved:raise PermissionError("explicit approval required")
        cap=self.backend.capabilities()
        if not (cap.safe_stop and cap.rollback and cap.verified_command_scheduling):
            raise RuntimeError("unsafe backend")
        if self.backend.faults():raise RuntimeError("hardware fault")
        self.backend.safe_stop()
        self.backend.apply_pi(gains)
        if not self.backend.verify_pi(gains):
            self.backend.safe_stop()
            raise RuntimeError("PI readback mismatch")
    def verify_written_pi(self,gains:Gains)->bool:
        return self.backend.verify_pi(gains)
    def restore(self,snapshot):
        self.backend.safe_stop()
        self.backend.restore(snapshot)

    def run_step(self,spec:Experiment,safety:SafetyEnvelope)->Trace:
        safety.validate(spec)
        cap=self.backend.capabilities()
        if not (cap.supports(spec.loop,spec.axis) and cap.safe_stop and
                cap.rollback and cap.verified_telemetry and cap.verified_command_scheduling):
            raise RuntimeError("step scheduling / MCU safety / telemetry not verified")
        if self.backend.faults():raise RuntimeError("hardware fault before test")
        signals=(SignalSpec("target","device"),SignalSpec("actual","device"))
        handle=self.backend.prepare_experiment(spec,safety,signals)
        if not isinstance(handle,ExperimentHandle) or not handle.test_id:
            raise RuntimeError("backend returned invalid experiment handle")
        start=time.monotonic()
        deadline=start+spec.duration_s+self.timeout_margin_s
        batches=[];dropped=0;last=None;command_time=None
        units=None
        try:
            self.backend.start_experiment(handle)
            while time.monotonic()<deadline:
                if self.backend.faults():raise RuntimeError("hardware fault during experiment")
                remaining=max(.01,deadline-time.monotonic())
                batch=self.backend.read_samples(handle,min(remaining,.5))
                if batch.test_id!=handle.test_id:
                    raise ValueError("test ID mismatch")
                info=validate_batch(batch,("target","actual"),
                                    max_drop_fraction=self.max_drop_fraction)
                if not info["synchronized"] or info["timestamp_source"] not in (
                    "device_clock","mcu_clock","synchronized_source"):
                    raise RuntimeError("telemetry lacks verified synchronized device/MCU clock")
                if last is not None and batch.timestamp_s[0]<=last:
                    raise ValueError("out-of-order/duplicate sample batches")
                if units is None:units={key:batch.units.get(key,"") for key in ("target","actual")}
                if units["target"]!=batch.units.get("target","") or units["actual"]!=batch.units.get("actual",""):
                    raise ValueError("inconsistent units across batches")
                last=float(batch.timestamp_s[-1])
                if "command_time_s" in batch.metadata:
                    v=float(batch.metadata["command_time_s"])
                    if command_time is not None and abs(v-command_time)>1e-9:
                        raise ValueError("inconsistent command application timestamp")
                    command_time=v
                dropped+=batch.dropped_frames
                batches.append(batch)
                # The backend explicitly marks the final complete capture.
                if batch.metadata.get("complete") is True:
                    break
            else:
                raise TimeoutError("capture incomplete before deadline")
            if not batches or not batches[-1].metadata.get("complete"):
                raise RuntimeError("capture incomplete")
            if command_time is None:
                raise ValueError("missing device-confirmed command application time")
            t=np.concatenate([b.timestamp_s for b in batches])
            target=np.concatenate([b.signals["target"] for b in batches])
            actual=np.concatenate([b.signals["actual"] for b in batches])
            fraction=dropped/max(len(t)+dropped,1)
            if fraction>self.max_drop_fraction:
                raise ValueError("excessive total dropped samples")
            trace=Trace(t,target,actual,signal=spec.loop.value,
                        units=units["actual"],command_time_s=command_time,
                        dropped_frames=dropped,
                        metadata={"source":"streaming_bridge",
                                  "timestamp_source":batches[0].timestamp_source,
                                  "synchronized":True,
                                  "test_id":handle.test_id,
                                  "device_id":batches[0].device_id,
                                  "motor_id":batches[0].motor_id})
            return trace
        finally:
            try:self.backend.stop_experiment(handle)
            finally:self.backend.safe_stop()
