import pytest
from piauto.v2.excitation import ExcitationPlan,generate_waveform

def _plan(**kw):
    d=dict(kind="chirp",center=0,amplitude=.01,duration_s=2,sample_hz=500,
           f_start_hz=1,f_end_hz=10,position_min=-.1,position_max=.1,speed_max=1)
    d.update(kw)
    return ExcitationPlan(**d)

def test_chirp_and_sine_planning():
    for name in ("chirp","sine"):
        out=generate_waveform(_plan(kind=name))
        assert len(out.timestamp_s)==1000
        assert max(abs(out.position_target))<=.011

def test_unsafe_waveforms_rejected():
    with pytest.raises(ValueError):generate_waveform(_plan(speed_max=.01))
    with pytest.raises(ValueError):generate_waveform(_plan(position_max=.005))
    with pytest.raises(ValueError):generate_waveform(_plan(sample_hz=30))
