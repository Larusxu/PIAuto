"""Optional V2 Streamlit offline analysis panel; never sends motor commands."""
from dataclasses import asdict

def render_offline_v2():
    import streamlit as st
    import pandas as pd
    from .identification import closed_loop_frequency_response
    from .sysid import fit_velocity_plant,imc_velocity_pi
    st.divider()
    st.subheader("V2 · 离线辨识与频率响应")
    st.caption("仅分析 CSV，不连接硬件、不修改 PI；闭环响应不是开环稳定裕度。")
    mode=st.radio("分析工具",["闭环频响（Bode）","电流→速度对象辨识"],horizontal=True,key="v2_analysis")
    uploaded=st.file_uploader("上传实验 CSV",type=["csv"],key="v2_file")
    if uploaded is None:return
    try:
        data=pd.read_csv(uploaded)
        if mode.startswith("闭环"):
            required={"timestamp_s","target","actual"}
            if not required.issubset(data.columns):raise ValueError("CSV 需要 timestamp_s,target,actual")
            result=closed_loop_frequency_response(data.timestamp_s,data.target,data.actual,min_coherence=.8)
            freq=result["frequency_hz"]
            st.caption(f"采样率 {result['sampling_hz']:.1f} Hz，相干度门限 0.8")
            st.line_chart(pd.DataFrame({"增益 (dB)":result["gain_db"]},index=freq),x_label="Hz")
            st.line_chart(pd.DataFrame({"相位 (deg)":result["phase_deg"]},index=freq),x_label="Hz")
            st.line_chart(pd.DataFrame({"相干度":result["coherence"]},index=freq),x_label="Hz")
            st.info("仅为目标→实际的闭环频响，不代表开环增益或相位裕度。")
        else:
            required={"timestamp_s","current_command","speed_actual"}
            if not required.issubset(data.columns):raise ValueError("CSV 需要 timestamp_s,current_command,speed_actual")
            confirmed=st.checkbox("确认数据来自电流/转矩激励，而非位置/速度闭环目标",key="v2_provenance")
            lam=st.number_input("IMC 时间常数 λ (秒)",min_value=.001,max_value=30.,value=.5,step=.05,key="v2_lambda")
            if not confirmed:
                st.warning("未确认激励来源，不能做物理对象辨识")
                return
            plant=fit_velocity_plant(data.timestamp_s,data.current_command,data.speed_actual,open_loop_current_excitation=True)
            st.json(asdict(plant))
            if plant.fit_quality=="valid":
                try:st.json(asdict(imc_velocity_pi(plant,lambda_s=lam)))
                except ValueError as exc:st.warning(str(exc))
            else:st.warning("拟合未通过可靠性门限，不提供 PI 候选")
            st.info("连续域 PI 不等于芯片寄存器值，需要专用单位换算及真实安全验证。")
    except (ValueError,KeyError,RuntimeError,TypeError) as exc:
        st.error("输入数据不符合要求："+str(exc))
