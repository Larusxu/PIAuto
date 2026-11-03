"""Device-neutral structured experiment reporting and trace export."""
from __future__ import annotations
import csv
import io
import json
from dataclasses import dataclass,asdict,is_dataclass
from enum import Enum
from datetime import datetime,timezone
import numpy as np
from .contracts import Trace

def _jsonable(v):
    if is_dataclass(v):return _jsonable(asdict(v))
    if isinstance(v,Enum):return v.value
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    if isinstance(v,dict):return {str(k):_jsonable(value) for k,value in v.items()}
    if isinstance(v,(list,tuple)):return [_jsonable(item) for item in v]
    if isinstance(v,(str,float,int,bool)) or v is None:return v
    return str(v)

@dataclass(frozen=True)
class TuningSessionReport:
    version: str
    device_id: str
    motor_id: str
    status: str
    output: object
    config_snapshot: object
    evidence_quality: object
    notes: tuple[str,...]=()
    created_at_utc: str=""

    def to_dict(self):
        data=_jsonable(self)
        if not data["created_at_utc"]:
            data["created_at_utc"]=datetime.now(timezone.utc).isoformat()
        return data

    def to_json(self)->str:
        return json.dumps(self.to_dict(),ensure_ascii=False,indent=2,allow_nan=False)

def trace_csv(trace:Trace)->str:
    if not (len(trace.timestamp_s)==len(trace.target)==len(trace.actual)):
        raise ValueError("trace size mismatch")
    stream=io.StringIO()
    writer=csv.writer(stream)
    writer.writerow(["timestamp_s","target","actual"])
    for t,r,y in zip(trace.timestamp_s,trace.target,trace.actual):
        if not np.isfinite([t,r,y]).all():
            raise ValueError("cannot export nonfinite telemetry")
        writer.writerow([f"{float(t):.9g}",f"{float(r):.12g}",f"{float(y):.12g}"])
    return stream.getvalue()
