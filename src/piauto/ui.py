"""Optional Streamlit standalone PI-tuning tab prototype; does not connect to hardware.

Host application can reuse the framework-neutral workflow/services instead.
"""
from __future__ import annotations
from dataclasses import asdict
import numpy as np

def run():
    import streamlit as st
    import pandas as pd
    from piauto.adapters.mock import MockMotorAdapter
    from piauto.contracts import Loop,DQMode,Experiment,SafetyEnvelope
    from piauto.v1 import V1SemiAutoTuner
    from piauto.v2 import V2AutoTuner
    from piauto.metrics import measure_step
    st.set_page_config(page_title="PIAuto · PI 整定",layout="wide")
    st.title("PI 整定")
    st.caption("独立演示：Mock 数据，不写真实芯片，不加载默认 PI。后续由上位机集成服务接口。")
    version=st.radio("整定版本",["V1 · 半自动","V2 · 自动"],horizontal=True)
    a,b=st.columns([1,2.4],gap="large")
    with a:
        loop=Loop(st.selectbox("控制环",[x.value for x in Loop]))
        mode=DQMode.SHARED;axis="q"
        if loop==Loop.CURRENT:
            mode=DQMode(st.radio("d/q PI",[x.value for x in DQMode],format_func=lambda x:"共用粗调" if x=="shared" else "独立细调"))
            if mode==DQMode.INDEPENDENT:axis=st.radio("电流轴",["d","q"],horizontal=True)
        step=st.number_input("安全测试阶跃幅度（Mock）",min_value=.1,max_value=5.,value=1.,step=.1)
        duration=st.slider("测试持续时间（s）",1.0,6.0,2.0,0.5)
        run_now=st.button("运行模拟整定",type="primary")
    if "history" not in st.session_state:st.session_state.history=[]
    if "pending_v1" not in st.session_state:st.session_state.pending_v1=None
    if run_now:
        adapter=MockMotorAdapter()
        spec=Experiment(loop=loop,axis=axis,dq_mode=mode,step=step,duration_s=duration)
        safety=SafetyEnvelope(5,100,2,6,-10,10)
        if version.startswith("V1"):
            workflow=V1SemiAutoTuner(adapter)
            proposal=workflow.analyze_and_propose(spec,safety)
            trace=workflow.baseline_trace
            result={"version":"V1","proposal":asdict(proposal),"trace":trace}
            st.session_state.pending_v1=workflow
        else:
            trace=adapter.run_step(spec,safety)
            output=V2AutoTuner(adapter).autotune(spec,safety,authorized=True)
            result={"version":"V2","result":asdict(output),"trace":trace}
        st.session_state.history.append(result)
    with b:
        if st.session_state.history:
            result=st.session_state.history[-1];tr=result["trace"]
            st.subheader("阶跃响应")
            df=pd.DataFrame({"目标":tr.target,"实际":tr.actual},index=tr.timestamp_s)
            st.line_chart(df,x_label="时间 (s)")
            m=measure_step(tr)
            c1,c2,c3=st.columns(3)
            c1.metric("超调率",f"{m.overshoot_percent:.2f}%")
            c2.metric("RMS",f"{m.rms_error:.4f}")
            c3.metric("稳定时间", "未知" if m.settling_time_s is None else f"{m.settling_time_s:.3f}s")
            with st.expander("候选参数 / 迭代详情",expanded=True):
                display={k:v for k,v in result.items() if k!="trace"}
                st.json(display)
            st.download_button("导出曲线 CSV",df.to_csv(index_label="timestamp_s"),"response.csv","text/csv")
            if result["version"] == "V1" and st.session_state.pending_v1 is not None:
                st.warning("候选 PI 仅为经验性小范围试调建议，须进行独立验证。以下操作只作用于 Mock。")
                left,right=st.columns(2)
                with left:
                    if st.button("批准试用并验证（仅 Mock）"):
                        try:
                            verification=st.session_state.pending_v1.verify_approved_candidate(approved=True)
                            st.session_state.last_verification=asdict(verification)
                        except Exception as ex:st.error(str(ex))
                with right:
                    if st.button("恢复基线 PI（仅 Mock）"):
                        st.session_state.pending_v1.rollback()
                        st.session_state.last_verification={"state":"rolled_back"}
                if "last_verification" in st.session_state:st.json(st.session_state.last_verification)
            st.caption("Mock 仅验证页面和算法状态机，不代表 TMC6460 硬件整定已经完成。")
        else:
            st.info("选择控制环并运行 Mock 整定。设备实测接口尚未接入。")

if __name__=="__main__":run()
