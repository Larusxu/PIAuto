# PIAuto · 通用电机 PI 整定工具

- **V1**：半自动候选生成、受控阶跃、指标计算、显式确认、验证与回滚。
- **V2**：独立入口、受约束自动迭代、闭环频响辨识计算；真实电机仅在安全能力明确验证后开放。
- **两层**：`piauto.v1/v2/metrics/modeling` 芯片无关；`piauto.adapters` 负责硬件、RTMI、单位与寄存器转换。
- **不处理固件默认 PI 初始化。** 仓库创建时为空，未提供 HPM 485 指令协议；真实硬件写入及其时隙调度没有被假定已实现。

## 快速运行

```bash
python -m pip install -e '.[test]'
pytest -q
piauto v1-mock
piauto v1-mock --approve
piauto v2-mock --approve
piauto v1-csv --csv measured.csv
```

UI Mock 版：`pip install -e '.[ui]'`，然后 `streamlit run streamlit_app.py`（如果作为模块运行受限，可在上位机中导入 `piauto.ui.run()`）。

CSV 列：`timestamp_s,target,actual`；命令触发时刻默认 `0s`，可在 `load_csv` 中指定；应包含触发前至少三个样本和足够长的触发后稳定段。目标、实际值需使用同一物理单位。

## 核心边界

- `python-control` 提供模型阶跃/响应；`SciPy` 提供优化与频响信号处理。
- 当前 V1 候选是 **有边界的启发式试验值**，并不是基于一次位置阶跃准确反推的硬件 PI。只有验证实测改善后才算通过。
- `TMC6460ReadAdapter` 只支持原始 PI 寄存器读取；`0x140` 的归一化与采样/移位、HPM 传输帧、RTMI 分帧/CRC、RS485 半双工时隙尚待接入。禁止在未知协议条件下通过它下发真实控制命令。
- V2 已具有独立自动迭代框架和离线闭环频域分析，但 **不等于已经完成真实电机的全自动辨识及模型参数求解**；这部分需要实际驱动接口与可信采样。闭环拟合模型不得用来假装预测修改 PI 后的硬件动态。
- `MotorAdapter.capabilities` 中的 `verified_telemetry`, `verified_command_scheduling`, `safe_stop` 等能力必须源于实测，不可仅通过配置布尔值绕过 MCU 物理保护。
- 受控停止、机械限位、电流保护、链路超时由 HPM 独立执行；上位机不能作为唯一停机保护。

## 开源参考

- https://github.com/nfreq/ktune (`metrics.py`, `plots.py`, `cli/command.py`)
- https://github.com/python-control/python-control
- https://github.com/scipy/scipy
- https://github.com/jnzim/foc-sysid (`py-script/closed_pos_bode_plot.py`)
- https://github.com/odriverobotics/ODrive (`Firmware/MotorControl/controller.cpp`)
- https://github.com/simplefoc/Arduino-FOC

详细实施边界见 `docs/实施方案.md`，界面建议见 `docs/界面方案.md`。
