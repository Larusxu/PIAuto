"""Import chip-neutral exported telemetry CSV without issuing motor commands."""
from __future__ import annotations
import csv
import json
from pathlib import Path
import numpy as np
from .base import ReadOnlyAdapter
from ..contracts import Trace, Gains, Capabilities

class OfflineAdapter(ReadOnlyAdapter):
    def __init__(self, trace: Trace, gains: Gains):
        self.trace,self.gains=trace,gains
    def capabilities(self):
        return Capabilities(timestamp_quality="imported",telemetry_hz=0)
    def read_pi(self):return self.gains
    def snapshot(self):return self.gains

def load_csv(path:str|Path,*,command_time_s:float=0,metadata_path=None)->Trace:
    """Required columns: timestamp_s,target,actual; optional dropped_frames per row."""
    with open(path,newline="",encoding="utf-8-sig") as f:
        rows=list(csv.DictReader(f))
    if not rows:raise ValueError("empty CSV")
    for k in ("timestamp_s","target","actual"):
        if k not in rows[0]:raise ValueError(f"missing CSV column: {k}")
    meta=json.loads(Path(metadata_path).read_text(encoding="utf-8")) if metadata_path else {}
    return Trace(np.array([float(x["timestamp_s"]) for x in rows]),
                 np.array([float(x["target"]) for x in rows]),
                 np.array([float(x["actual"]) for x in rows]),
                 signal=str(meta.get("signal","position")),units=str(meta.get("units","rad")),
                 command_time_s=float(meta.get("command_time_s",command_time_s)),
                 dropped_frames=int(meta.get("dropped_frames",0)),metadata=meta)
