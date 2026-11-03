"""Read-only TeslaHand hardware discovery; never writes power/motion parameters."""
import argparse
import json
from dataclasses import asdict
from .teslahand_modbus import open_monitor,BAUD

def main(argv=None):
    p=argparse.ArgumentParser(description="TeslaHand FC04 read-only motor power/status monitor")
    p.add_argument("--port",required=True,help="e.g. COM10 or /dev/ttyUSB0")
    p.add_argument("--slave",type=int,default=1)
    p.add_argument("--baud",type=int,default=BAUD)
    a=p.parse_args(argv)
    stream=None
    try:
        stream,monitor=open_monitor(a.port,baud=a.baud,slave=a.slave)
        status=monitor.read_status()
        print(json.dumps(asdict(status),ensure_ascii=False,indent=2))
        if status.stale_or_inconsistent:
            p.exit(2,"snapshot sequence changed during read; retry when stable\n")
        return 0
    except (RuntimeError,OSError,ValueError,TimeoutError) as e:
        p.exit(2,"read-only TeslaHand status check failed: "+str(e)+"\n")
    finally:
        if stream is not None:stream.close()

if __name__=="__main__":raise SystemExit(main())
