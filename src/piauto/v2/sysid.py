"""Open-loop speed plant identification, inspired by foc-sysid and SciPy.
A measured CURRENT/torque-mode step is required. Closed-loop position or speed
target data MUST NOT be fed to this estimator. Outputs are physical continuous
PI values; the chip adapter alone can convert/quantize them.
"""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares

@dataclass(frozen=True)
class VelocityPlant:
    gain: float
    tau_s: float
    offset: float
    rmse: float
    normalized_rmse: float
    fit_quality: str

@dataclass(frozen=True)
class PhysicalPI:
    kp: float
    ki: float
    lambda_s: float
    note: str = "continuous parallel PI; hardware conversion and testing required"

def _simulate(t,u,gain,tau,offset):
    y=np.empty_like(t,dtype=float)
    y[0]=offset
    for i in range(1,len(t)):
        alpha=-np.expm1(-(t[i]-t[i-1])/tau)
        y[i]=y[i-1]+alpha*(offset+gain*(u[i-1]-u[0])-y[i-1])
    return y

def fit_velocity_plant(t,current_command,speed_actual,*,open_loop_current_excitation=False):
    """Fit K/(tau*s+1) near a verified current-control operating point.
    Caller must independently verify actual command provenance and timebase.
    """
    if not open_loop_current_excitation:
        raise ValueError("requires confirmed torque/current excitation, not closed-loop target")
    t,u,y=[np.asarray(v,dtype=float) for v in (t,current_command,speed_actual)]
    if not (len(t)>=24 and len(t)==len(u)==len(y)):
        raise ValueError("insufficient same-length samples")
    if not all(np.isfinite(v).all() for v in (t,u,y)) or np.any(np.diff(t)<=0):
        raise ValueError("nonfinite data or nonmonotonic time")
    dt=np.diff(t)
    if max(dt)>2.5*np.median(dt):
        raise ValueError("irregular timestamps")
    du,dy=np.ptp(u),np.ptp(y)
    if du<=1e-12 or dy<=1e-12:
        raise ValueError("insufficient excitation/response")
    transitions=np.count_nonzero(abs(np.diff(u))>.025*du)
    if transitions>5:
        raise ValueError("expected one current step, not a chirp/multi-step")
    tau_min=max(float(np.median(dt)),1e-5)
    tau_max=max(float(t[-1]-t[0])*5,tau_min*2)
    scale=max(dy,1e-12)
    est_k=(y[-1]-y[0])/(u[-1]-u[0]) if abs(u[-1]-u[0])>.25*du else dy/du
    opt=least_squares(lambda x:(_simulate(t,u,x[0],x[1],x[2])-y)/scale,
      [est_k,max(tau_min*2,(t[-1]-t[0])*.2),float(y[0])],
      bounds=([-np.inf,tau_min,y[0]-dy],[np.inf,tau_max,y[0]+dy]),
      loss="soft_l1",max_nfev=200)
    gain,tau,offset=map(float,opt.x)
    fitted=_simulate(t,u,gain,tau,offset)
    rmse=float(np.sqrt(np.mean((fitted-y)**2)))
    nrmse=rmse/dy
    quality="valid" if opt.success and nrmse<.12 and tau<.85*tau_max and abs(gain)>1e-12 else "rejected"
    return VelocityPlant(gain,tau,offset,rmse,nrmse,quality)

def imc_velocity_pi(plant:VelocityPlant,*,lambda_s:float)->PhysicalPI:
    """Nominal no-delay first-order model: Kp=tau/(K lambda), Ki=1/(K lambda).
    No direct TMC registers! Real hardware may have saturation, delay or backlash.
    """
    if plant.fit_quality!="valid" or plant.gain<=0 or lambda_s<max(plant.tau_s,1e-6):
        raise ValueError("invalid or too aggressive IMC design")
    return PhysicalPI(plant.tau_s/(plant.gain*lambda_s),1/(plant.gain*lambda_s),lambda_s)
