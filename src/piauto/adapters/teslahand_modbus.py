"""Read-only TeslaHand UART1 Modbus FC04 monitor.

Protocol grounded in TSLHand application/TeslaHand/common/Modbus/MODBUS_PROTOCOL.md
(2026-10-29). This is NOT a TMC6460 register/RTMI transport; firmware does
not expose generic motor register writes over this Modbus interface.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import struct
import time

BAUD=460800
SLAVE=1
EXPECTED_MAGIC=0x5448
EXPECTED_VERSION=1
# Motor alias -> (UART, MAX20087 index, OUT number); 1-indexed.
EXPECTED_MOTOR_ROUTES=(
    (11,1,3),(7,1,1),(6,0,1),(5,0,3),(4,0,2),(2,0,4),
    (17,2,4),(16,2,2),(3,2,1),(15,2,3),(14,3,2),
    (10,3,4),(8,3,1),(9,3,3),(13,1,2),(12,1,4)
)

def modbus_crc(data:bytes)->int:
    crc=0xffff
    for b in data:
        crc^=b
        for _ in range(8):
            crc=(crc>>1) ^ (0xA001 if crc&1 else 0)
    return crc

class Stream(Protocol):
    def reset_input_buffer(self)->None: ...
    def write(self,data:bytes)->int: ...
    def read(self,size:int)->bytes: ...
    def flush(self)->None: ...

@dataclass(frozen=True)
class PowerRoute:
    motor_id:int
    uart:int
    chip_index:int
    out:int
    sys:int
    connector:int
    output_status:int
    status_valid:bool
    error:int

@dataclass(frozen=True)
class BoardStatus:
    slave:int
    protocol_version:int
    snapshot_seq:int
    snapshot_uptime_ms:int
    module_status:tuple[int,...]
    routes:tuple[PowerRoute,...]
    chips:tuple[tuple[int,...],...]
    stale_or_inconsistent:bool
    uptime_s:int

class TeslaHandModbusMonitor:
    def __init__(self,stream:Stream,*,slave:int=SLAVE,timeout_s:float=.5,min_gap_s:float=.01):
        if not 1<=slave<=247 or timeout_s<=0 or min_gap_s<0:
            raise ValueError("invalid Modbus monitor settings")
        self.stream=stream;self.slave=slave
        self.timeout_s=timeout_s;self.min_gap_s=min_gap_s
        self._last_tx=-float("inf")

    def read_input_registers(self,address:int,count:int)->tuple[int,...]:
        if not (0<=address<=65535 and 1<=count<=125 and address+count<=65536):
            raise ValueError("FC04 address/count out of range")
        gap=self.min_gap_s-(time.monotonic()-self._last_tx)
        if gap>0:time.sleep(gap)
        req=struct.pack(">BBHH",self.slave,4,address,count)
        packet=req+struct.pack("<H",modbus_crc(req))
        self.stream.reset_input_buffer()
        self.stream.write(packet);self.stream.flush()
        self._last_tx=time.monotonic()
        deadline=self._last_tx+self.timeout_s
        buf=bytearray()
        expected=None
        while time.monotonic()<deadline:
            chunk=self.stream.read(256)
            if chunk:buf.extend(chunk)
            if len(buf)>=3 and expected is None:
                if buf[0]!=self.slave:
                    raise RuntimeError("Modbus slave mismatch")
                if buf[1]==0x84:expected=5
                elif buf[1]==4:
                    expected=5+buf[2]
                    if expected!=5+2*count:raise RuntimeError("unexpected Modbus byte count")
                else:raise RuntimeError("unexpected Modbus function")
            if expected is not None and len(buf)>=expected:break
        if expected is None or len(buf)!=expected:
            raise TimeoutError("Modbus truncated or timed-out response")
        if modbus_crc(buf)!=0:
            raise RuntimeError("Modbus CRC mismatch")
        if buf[1]==0x84:
            raise RuntimeError(f"Modbus FC04 exception {buf[2]}")
        if buf[2]!=count*2:
            raise RuntimeError("Modbus unexpected response length")
        return struct.unpack(">"+str(count)+"H",buf[3:-2])

    def read_status(self)->BoardStatus:
        # Existing firmware snapshots change about every 500 ms. One consistent
        # read needs a before/after sequence check; never infer live 5 V voltage.
        header=self.read_input_registers(0,80)
        if header[0]!=EXPECTED_MAGIC or header[1]!=EXPECTED_VERSION:
            raise RuntimeError("unsupported TeslaHand firmware protocol")
        chips=self.read_input_registers(80,32)
        a=self.read_input_registers(128,64)
        b=self.read_input_registers(192,64)
        tail=self.read_input_registers(4,4)
        seq=(header[4]<<16)|header[5]
        seq_after=(tail[0]<<16)|tail[1]
        timestamp=(header[6]<<16)|header[7]
        routes=[]
        motors=a+b
        for i,expected in enumerate(EXPECTED_MOTOR_ROUTES):
            fields=motors[i*8:(i+1)*8]
            if len(fields)!=8 or tuple(fields[:3])!=expected:
                raise RuntimeError(f"unexpected motor mapping at M{i+1}")
            error=fields[7] if fields[7]<32768 else fields[7]-65536
            routes.append(PowerRoute(i+1,*fields[:5],fields[5],
                                     bool(fields[6]),error))
        return BoardStatus(self.slave,header[1],seq,timestamp,
                           tuple(header[64:70]),tuple(routes),
                           tuple(tuple(chips[j:j+8]) for j in range(0,32,8)),
                           seq!=seq_after,int(header[3]))

def open_monitor(port:str,*,baud:int=BAUD,slave:int=SLAVE):
    """Optional pyserial factory. Never sends FC03/FC06 or motion commands."""
    try:
        import serial
    except ImportError as e:
        raise RuntimeError("pip install pyserial to use UART monitor") from e
    stream=serial.Serial(port,baudrate=baud,bytesize=8,parity="N",stopbits=1,timeout=.1)
    return stream,TeslaHandModbusMonitor(stream,slave=slave)
