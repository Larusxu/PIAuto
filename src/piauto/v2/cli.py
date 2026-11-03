"""Independent V2 offline CLI. No hardware command can be sent."""
from __future__ import annotations
import argparse,csv,json
from dataclasses import asdict
import numpy as np
from .sysid import fit_velocity_plant,imc_velocity_pi
from .identification import closed_loop_frequency_response

def _read(path,columns):
    with open(path,newline="",encoding="utf-8-sig") as stream:
        rows=list(csv.DictReader(stream))
    if not rows:raise ValueError("CSV is empty")
    try:
        return {name:np.asarray([float(row[name]) for row in rows]) for name in columns}
    except (KeyError,ValueError) as exc:
        raise ValueError("missing or nonnumeric columns: "+str(columns)) from exc

def main(argv=None):
    p=argparse.ArgumentParser(description="PIAuto V2 offline model analysis: no motor writes")
    sub=p.add_subparsers(dest="command",required=True)
    plant=sub.add_parser("sysid-velocity",help="current step to rotor velocity")
    plant.add_argument("--csv",required=True)
    plant.add_argument("--confirmed-current-mode",action="store_true")
    plant.add_argument("--lambda-s",type=float)
    bode=sub.add_parser("bode",help="closed-loop frequency response, not stability margin")
    bode.add_argument("--csv",required=True)
    bode.add_argument("--min-coherence",type=float,default=.8)
    a=p.parse_args(argv)
    try:
        if a.command=="sysid-velocity":
            x=_read(a.csv,("timestamp_s","current_command","speed_actual"))
            model=fit_velocity_plant(x["timestamp_s"],x["current_command"],x["speed_actual"],
                                     open_loop_current_excitation=a.confirmed_current_mode)
            out={"plant":asdict(model),"physical_pi":None}
            if a.lambda_s is not None:
                out["physical_pi"]=asdict(imc_velocity_pi(model,lambda_s=a.lambda_s))
        else:
            x=_read(a.csv,("timestamp_s","target","actual"))
            response=closed_loop_frequency_response(x["timestamp_s"],x["target"],x["actual"],min_coherence=a.min_coherence)
            out={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in response.items()}
        print(json.dumps(out,ensure_ascii=False,indent=2))
        return 0
    except (ValueError,OSError) as exc:
        p.error(str(exc))
if __name__=="__main__":raise SystemExit(main())
