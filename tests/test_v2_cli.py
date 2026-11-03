from pathlib import Path
import csv,json
import numpy as np
import pytest
from piauto.v2.cli import main
from piauto.v2.sysid import _simulate

def test_sysid_cli_and_provenance_requirement(tmp_path,capsys):
    t=np.arange(0,3,.01)
    u=np.where(t<.3,0.,.5)
    y=_simulate(t,u,8,.2,0)
    fp=tmp_path/"current_step.csv"
    with fp.open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f);w.writerow(["timestamp_s","current_command","speed_actual"])
        w.writerows(zip(t,u,y))
    assert main(["sysid-velocity","--csv",str(fp),"--confirmed-current-mode","--lambda-s",".5"])==0
    result=json.loads(capsys.readouterr().out)
    assert result["plant"]["fit_quality"]=="valid"
    assert result["physical_pi"]["kp"]==pytest.approx(.05,rel=.08)
    with pytest.raises(SystemExit):
        main(["sysid-velocity","--csv",str(fp),"--lambda-s",".5"])
