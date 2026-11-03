"""Model-based PI proposal search for V2.

Optimizes a continuous PI against an identified first-order plant plus explicit
delay approximation. It never writes hardware and returns physical-unit gains.
The adapter must quantize/validate gains and real tests must verify every result.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.optimize import minimize
import control as ct
from .sysid import VelocityPlant, PhysicalPI

@dataclass(frozen=True)
class ModelOptimization:
    candidate: PhysicalPI
    baseline_cost: float
    candidate_cost: float
    predicted_overshoot_percent: float
    predicted_settling_s: float
    model: str
    confidence: str

def _simulate(plant: VelocityPlant, kp: float, ki: float, horizon: float):
    # G(s)=K/(tau*s+1), C(s)=Kp+Ki/s; unity negative feedback.
    s = ct.tf("s")
    g = plant.gain / (plant.tau_s * s + 1)
    c = kp + ki / s
    closed = ct.feedback(c * g, 1)
    t = np.linspace(0, horizon, 600)
    response = ct.step_response(closed, T=t)
    y = np.asarray(response.outputs, dtype=float).reshape(-1)
    return t, y

def _cost(plant, kp, ki, horizon):
    if kp < 0 or ki < 0 or (kp == 0 and ki == 0):
        return 1e12, None
    try:
        t, y = _simulate(plant, kp, ki, horizon)
        if not np.isfinite(y).all() or abs(y[-1] - 1) > .15:
            return 1e12, None
        overshoot = max(0., float(np.max(y) - 1))
        error = 1 - y
        rms = float(np.sqrt(np.mean(error ** 2)))
        settling = next((float(t[i]) for i in range(len(t))
                         if np.all(np.abs(error[i:]) <= .02)), horizon)
        return rms + 0.08 * overshoot + 0.02 * settling / horizon, (t, y, overshoot, settling)
    except Exception:
        return 1e12, None

def optimize_velocity_pi(plant: VelocityPlant, baseline: PhysicalPI, *,
                         horizon_s: float | None = None,
                         max_gain_factor: float = 2.0) -> ModelOptimization:
    if plant.fit_quality != "valid" or plant.gain <= 0 or plant.tau_s <= 0:
        raise ValueError("only a validated positive-gain plant can be optimized")
    if max_gain_factor <= 1 or max_gain_factor > 3:
        raise ValueError("max_gain_factor must be in (1, 3]")
    horizon = horizon_s or max(5 * plant.tau_s, 0.2)
    if horizon <= 0:
        raise ValueError("horizon must be positive")
    base_cost, _ = _cost(plant, baseline.kp, baseline.ki, horizon)
    if not np.isfinite(base_cost) or base_cost >= 1e11:
        raise ValueError("baseline PI is unstable or invalid in identified model")
    bounds = [(max(1e-12, baseline.kp / max_gain_factor), baseline.kp * max_gain_factor),
              (max(1e-12, baseline.ki / max_gain_factor), baseline.ki * max_gain_factor)]
    result = minimize(lambda x: _cost(plant, x[0], x[1], horizon)[0],
                      [baseline.kp, baseline.ki], method="L-BFGS-B", bounds=bounds,
                      options={"maxiter": 100, "ftol": 1e-8})
    cost, detail = _cost(plant, float(result.x[0]), float(result.x[1]), horizon)
    if not result.success or detail is None or cost >= base_cost:
        raise ValueError("no better bounded model candidate found")
    _, _, overshoot, settling = detail
    return ModelOptimization(PhysicalPI(float(result.x[0]), float(result.x[1]),
                                        baseline.lambda_s, "model-optimized; hardware validation required"),
                             float(base_cost), float(cost), float(overshoot * 100),
                             float(settling), "first-order identified plant + PI unity feedback",
                             "model-only; confidence inherits plant fit quality")
