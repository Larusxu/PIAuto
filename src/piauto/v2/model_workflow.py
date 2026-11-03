"""Offline model-assisted V2 workflow for velocity-loop PI design.

This workflow deliberately stops at a proposal. Real-device application must go
through V1's explicit approval/verification path and a hardware-specific adapter.
"""
from dataclasses import dataclass
import numpy as np
from .sysid import fit_velocity_plant, PhysicalPI
from .model_optimizer import optimize_velocity_pi, ModelOptimization

@dataclass(frozen=True)
class ModelTuningResult:
    plant: object
    optimization: ModelOptimization
    sample_count: int
    sampling_hz: float

def analyze_and_optimize_velocity(timestamp_s, current_command, speed_actual,
                                  baseline: PhysicalPI, *, excitation_confirmed: bool,
                                  max_gain_factor: float = 2.0) -> ModelTuningResult:
    t = np.asarray(timestamp_s, dtype=float)
    if len(t) < 24 or np.any(np.diff(t) <= 0):
        raise ValueError("at least 24 monotonically timestamped samples are required")
    plant = fit_velocity_plant(t, current_command, speed_actual,
                               open_loop_current_excitation=excitation_confirmed)
    if plant.fit_quality != "valid":
        raise ValueError("identified plant failed quality gates; no PI proposal")
    optimized = optimize_velocity_pi(plant, baseline, max_gain_factor=max_gain_factor)
    return ModelTuningResult(plant, optimized, len(t), float(1 / np.median(np.diff(t))))
