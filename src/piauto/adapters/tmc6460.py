"""TMC6460 codec and RTMI intake boundary (NO imaginary HPM command protocol).

Actual wiring requires implementation of the site's HPM serial transport and
validation against its firmware's RTMI frame format, CRC and bus scheduling.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
from ..contracts import PI, Gains, Capabilities
from .base import ReadOnlyAdapter

REG = {
    "pid_config": 0x140, "flux_pi":0x142, "torque_pi":0x143,
    "velocity_pi":0x145,"position_pi":0x146,
    "velocity_limit":0x14B, "velocity_target":0x150,
    "position_target":0x152,"velocity_actual":0x154,"position_actual":0x155,
    "uart_control":0x340,"rtmi_ch0":0x344,"rtmi_ch7":0x34B,
}

def decode_pi_word(word:int)->tuple[int,int]:
    def s16(u):return u-65536 if u&0x8000 else u
    return s16((word>>16)&65535),s16(word&65535)

def encode_pi_word(kp:int,ki:int)->int:
    if not all(isinstance(x,int) and -32768<=x<=32767 for x in (kp,ki)):
        raise ValueError("TMC PI coefficients must be signed int16 raw values")
    return ((kp&0xFFFF)<<16)|(ki&0xFFFF)

class TMCRegisterTransport(Protocol):
    def read_register(self,addr:int)->int: ...
    def write_register(self,addr:int,value:int)->None: ...

@dataclass(frozen=True)
class RTMIRecord:
    """Only normalized values; decoder must be configured per real firmware framing."""
    timestamp_s:float
    channel:int
    register:int
    raw_value:int
    crc_valid:bool|None

class TMC6460ReadAdapter(ReadOnlyAdapter):
    """Non-actuating register reader. Cannot enable motor, start tests or apply PI."""
    def __init__(self,transport:TMCRegisterTransport):self.transport=transport
    def capabilities(self):return Capabilities(timestamp_quality="unverified")
    def read_raw_pi(self)->dict:
        return {name:decode_pi_word(self.transport.read_register(REG[key])) for name,key in
                (("d","flux_pi"),("q","torque_pi"),("velocity","velocity_pi"),("position","position_pi"))}
    def read_pi(self)->Gains:
        raise RuntimeError("must decode 0x140 normalization and loop sampling configuration before physical gains can be supplied")
    def snapshot(self):return {key:self.transport.read_register(REG[key]) for key in ("pid_config","flux_pi","torque_pi","velocity_pi","position_pi")}
