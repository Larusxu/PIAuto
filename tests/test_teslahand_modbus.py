import struct
import pytest
from piauto.adapters.teslahand_modbus import (
    TeslaHandModbusMonitor,modbus_crc,EXPECTED_MOTOR_ROUTES,BAUD)

class FakePort:
    def __init__(self):
        self.response=bytearray()
        self.corrupt_crc=False
        self.bad_route=False
        self.commands=[]
    def reset_input_buffer(self):
        self.response.clear()
    def flush(self):pass
    def write(self,data):
        self.commands.append(bytes(data))
        assert data[1]==4
        assert modbus_crc(data)==0
        _,_,start,count=struct.unpack(">BBHH",data[:-2])
        table=[0]*256
        table[:4]=[0x5448,1,1,1]
        table[4:8]=[0,7,0,500]
        table[64:70]=[1]*6
        for i,route in enumerate(EXPECTED_MOTOR_ROUTES):
            uart,chip,out=route
            table[128+i*8:128+i*8+8]=[
                uart if not self.bad_route or i!=0 else 99,
                chip,out,12,17+i,0,1,0]
        body=bytes([1,4,count*2])+struct.pack(">"+str(count)+"H",*table[start:start+count])
        crc=modbus_crc(body)
        if self.corrupt_crc:crc^=0x2000
        self.response.extend(body+struct.pack("<H",crc))
        return len(data)
    def read(self,size):
        data=self.response[:size]
        del self.response[:size]
        return bytes(data)

def test_crc_standard_modbus_example():
    request=bytes.fromhex("01 04 00 00 00 03")
    assert modbus_crc(request)==0x0B00

def test_status_read_only_and_motor_routing():
    port=FakePort()
    monitor=TeslaHandModbusMonitor(port,min_gap_s=0)
    status=monitor.read_status()
    assert status.snapshot_seq==7
    assert status.module_status==(1,)*6
    assert len(status.routes)==16
    assert status.routes[8].motor_id==9
    assert status.routes[8].uart==3
    assert status.routes[8].output_status==0
    assert all(command[1]==4 for command in port.commands)
    assert not status.stale_or_inconsistent

def test_read_rejects_corruption_and_unsafe_operations():
    port=FakePort();port.corrupt_crc=True
    with pytest.raises(RuntimeError,match="CRC"):
        TeslaHandModbusMonitor(port,min_gap_s=0).read_input_registers(0,3)
    with pytest.raises(ValueError):
        TeslaHandModbusMonitor(port).read_input_registers(0,126)

def test_unexpected_motor_route_is_rejected():
    port=FakePort();port.bad_route=True
    with pytest.raises(RuntimeError,match="mapping"):
        TeslaHandModbusMonitor(port,min_gap_s=0).read_status()
