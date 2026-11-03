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


## V2 独立离线工具（不依赖 V1）

```bash
pip install -e '.[test]'
piauto-v2 sysid-velocity --csv current_step.csv --confirmed-current-mode --lambda-s 0.5
piauto-v2 bode --csv position_chirp.csv --min-coherence 0.8
pytest -q
```

- `sysid-velocity` CSV：`timestamp_s,current_command,speed_actual`；要求**确认为当前电流/转矩控制激励**，否则拒绝拟合。拟合使用 SciPy `least_squares` 求一阶速度对象，基于 IMC 返回**物理连续域** PI 初值；这些数字**不是** TMC6460 的定点寄存器值。
- `bode` CSV：`timestamp_s,target,actual`；输出闭环增益、相位、相干度，不能直接宣称开环稳定裕度。频响按 foc-sysid 的 `csd/welch/coherence` 思路实现。
- `piauto.v2.excitation.generate_waveform`：按 ktune 的 Sine/Chirp 实验形式离线规划信号，并进行位置、速度、采样率的初步检查；**不负责在 HPM 上定时执行**。
- `piauto v2-mock --approve`：独立的 V2 Mock 自动候选循环，每轮测试后根据实测指标决定接受或回滚。共享 d/q 时两轴都要验证。

根目录的 [开发进度表](开发进度表.md) 记录已完成事项和真实硬件尚未实现的限制；[原始需求手册](原始需求手册.md) 和 [上位机界面原始需求](上位机界面原始需求.md) 保留原始要求。

**尚未完成的不是文档功能**：HPM6E8Y 实际 RTMI/RS485 帧接口、采样同步及安全停机验收、真实电机电流环激励、TMC 寄存器物理 PI 量化，以及实板端到端 V2 全自动验收。没有硬件协议时这些工作不得以 Mock 代替。


## V2 分层自动整定与激励规划

- `piauto-v2 staged-mock --approve --dq shared --max-rounds 1`：完整电流（d/q 联动）→速度→位置 Mock 测试；`--dq independent` 则仅优化所选轴的独立电流参数。
- `piauto.v2.staged.V2StagedTuner`：真实设备未来可按每层不同单位、安全边界创建 `TuningStage`；异常后回退整次会话的原始 PI。支持只选位置环，但必须确认内环已经验证。
- `piauto.v2.excitation.generate_waveform`：离线预生成正弦 / Chirp 测试信号；不会通过 UART 或 RS485 发送。**采样率、时钟和 MCU 停止能力未验收前，禁止将其直接用于实物运动。**
- Streamlit 的 V2 页新增 CSV 闭环 Bode 曲线（增益/相位/相干度）与电流→速度对象辨识工具，明确标注物理 PI 与寄存器值的区别。
- 当前本地 Python 核心测试：`16 passed`。真实联机部分请查看 [开发进度表](开发进度表.md) 中的未完成项。
