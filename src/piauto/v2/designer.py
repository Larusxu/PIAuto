"""V2 model identification planner for all PI loops, independent of chip drivers.

Each evidence record carries input provenance; it must be established by the
adapter, not assumed from a CSV column title. Output is continuous physical PI
only. A real device must quantize, read back, verify and rerun experiments.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Mapping
import numpy as np
from ..contracts import Loop, DQMode, PI
from .current_model import fit_rl_current, current_pi_from_bandwidth
from .position_model import fit_inner_velocity_tracking, optimize_position_pi
from .sysid import fit_velocity_plant, imc_velocity_pi, PhysicalPI

@dataclass(frozen=True)
class ModelEvidence:
    loop: Loop
    axis: str
    timestamp_s: np.ndarray
    stimulus: np.ndarray
    response: np.ndarray
    stimulus_kind: str
    units: tuple[str, str]
    verified: bool
    metadata: Mapping[str, object] = field(default_factory=dict)

@dataclass(frozen=True)
class LoopDesign:
    loop: Loop
    mode: DQMode
    suggested: Mapping[str, PI]
    models: Mapping[str, object]
    method: str
    note: str = "physical PI; device adaptation, readback and experiment verification required"

def _validate_record(e: ModelEvidence, loop: Loop, kind: str, axis: str) -> None:
    if e.loop!=loop or e.axis!=axis or e.stimulus_kind!=kind or not e.verified:
        raise PermissionError(f"{loop.value}/{axis} requires verified {kind} model evidence")
    t,u,y=[np.asarray(a,dtype=float) for a in (e.timestamp_s,e.stimulus,e.response)]
    if len(t)<40 or len(t)!=len(u) or len(t)!=len(y) or not all(np.isfinite(a).all() for a in (t,u,y)):
        raise ValueError("invalid model evidence")
    dt=np.diff(t)
    if np.any(dt<=0) or np.max(dt)>2.5*np.median(dt):
        raise ValueError("unreliable model evidence timestamps")

def design_current(evidence: Mapping[str,ModelEvidence], *,
                   mode: DQMode, bandwidth_hz: float,
                   pwm_hz: float, selected_axis: str = "q") -> LoopDesign:
    axes=("d","q") if mode==DQMode.SHARED else (selected_axis,)
    if any(a not in ("d","q") for a in axes):
        raise ValueError("invalid current axis")
    designed={};models={}
    for a in axes:
        e=evidence[a]
        _validate_record(e,Loop.CURRENT,"effective_voltage",a)
        plant=fit_rl_current(e.timestamp_s,e.stimulus,e.response,axis=a,
                             stationary_verified=bool(e.metadata.get("stationary_verified")),
                             decoupling_verified=bool(e.metadata.get("decoupling_verified")))
        if plant.fit_quality!="valid":raise ValueError(f"{a} RL model rejected")
        pi=current_pi_from_bandwidth(plant,bandwidth_hz,pwm_hz=pwm_hz)
        models[a]=plant;designed[a]=PI(pi.kp,pi.ki)
    if mode==DQMode.SHARED:
        # Conservative gains across both axes, with mandatory separate validation.
        kp=min(designed["d"].kp,designed["q"].kp)
        ki=min(designed["d"].ki,designed["q"].ki)
        designed={"d":PI(kp,ki),"q":PI(kp,ki)}
    return LoopDesign(Loop.CURRENT,mode,designed,models,
                      "RL voltage-to-current identification; bandwidth PI")

def design_velocity(e: ModelEvidence, *, lambda_s: float) -> LoopDesign:
    _validate_record(e,Loop.VELOCITY,"current_to_speed",e.axis)
    p=fit_velocity_plant(e.timestamp_s,e.stimulus,e.response,open_loop_current_excitation=True)
    if p.fit_quality!="valid":raise ValueError("velocity plant rejected")
    pi=imc_velocity_pi(p,lambda_s=lambda_s)
    return LoopDesign(Loop.VELOCITY,DQMode.INDEPENDENT,{"velocity":PI(pi.kp,pi.ki)},
                      {"velocity":p},"current-to-speed first order identification + IMC")

def design_position(e: ModelEvidence, *, baseline: PI,
                    horizon_s: float = 1.5) -> LoopDesign:
    _validate_record(e,Loop.POSITION,"velocity_command_to_velocity",e.axis)
    p=fit_inner_velocity_tracking(e.timestamp_s,e.stimulus,e.response,
                                  inner_loop_verified=bool(e.metadata.get("inner_loop_verified")))
    if p.fit_quality!="valid":raise ValueError("inner velocity tracking plant rejected")
    out=optimize_position_pi(p,baseline.kp,baseline.ki,horizon_s=horizon_s)
    return LoopDesign(Loop.POSITION,DQMode.INDEPENDENT,{"position":PI(out.kp,out.ki)},
                      {"position":p},"validated inner velocity transfer + position PI model optimization")
