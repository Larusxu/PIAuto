"""CLI defaults to mock/offline; never opens a real motor implicitly."""
from __future__ import annotations
import argparse,json
from dataclasses import asdict
from .contracts import Experiment,SafetyEnvelope,Loop,DQMode,PI,Gains
from .adapters.mock import MockMotorAdapter
from .adapters.offline import OfflineAdapter,load_csv
from .v1 import V1SemiAutoTuner

SAFETY=SafetyEnvelope(max_step=5,max_speed=100,max_current=2,max_duration_s=6,min_position=-10,max_position=10)

def main(argv=None):
    p=argparse.ArgumentParser(description="PIAuto: offline/Mock tuner (never auto-enables hardware)")
    p.add_argument("command",choices=["v1-mock","v1-csv","v2-mock"])
    p.add_argument("--csv")
    p.add_argument("--loop",choices=[x.value for x in Loop],default="position")
    p.add_argument("--dq",choices=[x.value for x in DQMode],default="shared")
    p.add_argument("--axis",choices=["d","q"],default="q")
    p.add_argument("--step",type=float,default=1)
    p.add_argument("--approve",action="store_true",help="mock-only candidate approval")
    args=p.parse_args(argv)
    spec=Experiment(loop=Loop(args.loop),axis=args.axis,dq_mode=DQMode(args.dq),step=args.step)
    adapter=MockMotorAdapter()
    if args.command=="v1-csv":
        if not args.csv:p.error("--csv is required")
        trace=load_csv(args.csv)
        adapter=OfflineAdapter(trace,adapter.read_pi())
        proposal=V1SemiAutoTuner(adapter).analyze_and_propose(spec,SAFETY,offline_trace=trace)
        print(json.dumps(asdict(proposal),ensure_ascii=False,indent=2));return 0
    if args.command=="v1-mock":
        workflow=V1SemiAutoTuner(adapter)
        proposal=workflow.analyze_and_propose(spec,SAFETY)
        print(json.dumps(asdict(proposal),ensure_ascii=False,indent=2))
        if args.approve:
            result=workflow.verify_approved_candidate(approved=True)
            print(json.dumps(asdict(result),ensure_ascii=False,indent=2,default=str))
        return 0
    from .v2 import V2AutoTuner
    if not args.approve:
        p.error("v2-mock requires --approve (simulation only)")
    result=V2AutoTuner(adapter).autotune(spec,SAFETY,authorized=True,max_rounds=3)
    print(json.dumps(asdict(result),ensure_ascii=False,indent=2,default=str));return 0

if __name__=="__main__":raise SystemExit(main())
