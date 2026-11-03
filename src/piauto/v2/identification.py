"""Closed-loop frequency response with coherence quality gates.
Inspired by foc-sysid; not an open-loop margin estimate.
"""
import numpy as np
from scipy.signal import csd,welch,coherence


def closed_loop_frequency_response(t,target,actual,*,min_coherence=.8,band_hz=None):
    t=np.asarray(t,dtype=float);u=np.asarray(target,dtype=float);y=np.asarray(actual,dtype=float)
    if len(t)<128 or len(u)!=len(t) or len(y)!=len(t) or np.any(np.diff(t)<=0):
        raise ValueError("insufficient monotonic data")
    dt=np.diff(t);fs=1/np.median(dt)
    if np.max(dt)>2.5*np.median(dt):raise ValueError("irregular timebase: resample explicitly")
    nperseg=min(256,len(t)//2)
    f,syu=csd(u-np.mean(u),y-np.mean(y),fs=fs,nperseg=nperseg)
    _,suu=welch(u-np.mean(u),fs=fs,nperseg=nperseg)
    _,coh=coherence(u,y,fs=fs,nperseg=nperseg)
    h=syu/np.maximum(suu,1e-22)
    good=(f>0)&(coh>=min_coherence)&(suu>max(1e-20,float(np.max(suu))*1e-5))
    if band_hz is not None:good &= (f>=band_hz[0])&(f<=band_hz[1])
    if not np.any(good):raise ValueError("no reliable coherent frequency bins")
    return {"frequency_hz":f[good],"gain_db":20*np.log10(np.maximum(np.abs(h[good]),1e-20)),
            "phase_deg":np.rad2deg(np.unwrap(np.angle(h[good]))),"coherence":coh[good],
            "sampling_hz":fs,"transfer":"closed-loop measured response"}
