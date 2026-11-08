# AI 自主学习阶段一：可复现闭环与 EV3 接触模型

日期：2026-11-08。所有结果均为离线仿真，**不是实机成功率**。

## 1. 结论先行

本轮建立了可训练环境、专家数据收集、GRU 行为克隆、循环 PPO、配对真值评估、模型导出和纯 MicroPython 推理原型。直接让小型 GRU 每 200 ms 控制底盘的端到端方案在独立测试中为 **0/24**，不能替代 Final；失败结果和权重均保留。

有实际增益的成果是分层的 769 参数 `ContactGRU`：它从电机负载、里程计增量和动作历史学习“物体是否稳定进入推板”。在独立物理随机化测试中，相比固定负载阈值，整局虚报由 **37/50 降至 2/50**，召回由 **10.3% 提升到 54.0%**。它已导出为 8.6 KB 的纯 Python 权重，可由 Pybricks/MicroPython 运行，但尚未实机验收。

## 2. 学习闭环

代码入口：

```text
learning/env.py             POMDP 环境、领域随机化、世界真值奖励/评分
learning/baseline.py        原样运行 Final、收集专家时序
learning/train.py           GRU 行为克隆 + recurrent PPO
learning/evaluate.py        Final/GRU 同场景配对评估
learning/contact_train.py   接触 GRU 数据生成、训练、阈值选择、独立测试
learning/export_contact.py  PyTorch -> 纯 Python 权重 + 数值对拍/性能测量
src/contact_inference.py    无第三方依赖的 GRUCell 推理
src/learned_capture.py      带 RIGHT 启动门、白边制动和超时的实机原型
```

低层策略输入严格限制为实机可得量：单次 RGB、左右电机负载、里程计累计/增量、时间和上一动作。终点记忆由“RGB 命中色标时的里程计坐标”计算。物体位置、真实接触、场地真位姿和成功标志只进入训练标签、奖励与评估器，不进入模型观测。

动作是前进、左右弧线、左右急转、倒车、制动、慢进和主动释放。释放动作后才用世界真值判断成功；错误释放记为虚报。

物理随机化包含物体半径/负载/位置、推板摩擦、负载噪声、颜色噪声、加速度、白带宽度、传感器前伸以及距离/转角里程计误差。`train`、`val`、`test` 使用互不重叠的固定种子段。

## 3. 端到端实验与失败分析

专家集从 Final 的 120 次随机尝试中只保留世界真值可接受的 29 条轨迹，共 25,007 个控制样本；没有把失败轨迹包装成专家。行为克隆的批内动作准确率达到约 90%，但闭环出现分布漂移。

尝试记录：

| 实验 | 结果 | 主要现象 |
|---|---:|---|
| 24 维观测，行为克隆 | 0/24 | 终点只存在 GRU 长期记忆中，常未找到色标 |
| 加里程计终点外部记忆，行为克隆 | 0/24 | 终点记忆 24/24 建立，但捕获不稳定 |
| recurrent PPO，第一轮 65 updates | 验证 0/12 | 收敛到停住以规避越界 |
| recurrent PPO，分层 scout 后 35 updates | 验证 0/12 | 搜索/接触的低层信用分配仍不足，已停止 |

正式对比使用 24 个全新测试场景、相同起点/物体/摩擦/噪声/里程计误差、相同 300 s 总预算，成功完全依据仿真物体真值。学习侧使用现有 Final 的 `calib` 安全 scout 后才交给 GRU，因此是明确的混合架构，而不是宣称全神经控制。

| 策略 | 真值成功 | 可接受成功 | 轮越白带 | 物体出界 | 虚报 |
|---|---:|---:|---:|---:|---:|
| Final（当前 R2/Final） | 7/24 | 6/24 | 7/24 | 10/24 | 3/24 |
| scout + 低层 GRU | 0/24 | 0/24 | 12/24 | 2/24 | 0/24 |

逐种子原始记录见 `docs/ai_full_eval.json`。GRU 更少把物体推出场外，但代价是没有完成任务；它不是改进版。

## 4. 接触序列模型

`ContactGRU` 输入每 100 ms 的 7 维序列：左右负载、相对空载的总负载增量、距离/转角增量、指令速度和转向率。训练集 240 局；阈值与连续确认长度只在 60 局验证集选择；最终结果来自另 80 局测试集。对象负载 6–45 mNm、摩擦 0.08–0.65、负载噪声 1.5–6.0 mNm，并包含倒车释放/重新接触。

| 检测器 | Precision | Recall | F1 | 无捕获局虚报 | 轻物体召回 |
|---|---:|---:|---:|---:|---:|
| 固定负载增量 >15 mNm，连续 3 次 | 35.0% | 10.3% | 15.9% | 37/50 | 1.1% |
| ContactGRU，验证集选择阈值 | 60.6% | 54.0% | 57.1% | 2/50 | 4.8% |

轻物体仍是硬问题：高置信阈值大幅减少毁灭性的虚报，也漏掉多数 6–14 mNm 物体。真实负载分布只能由 `probe_load.py` 实测决定是否可接受。

## 5. 部署测量与边界

- GRU：7 输入、12 隐状态、769 参数；导出权重模块 8,558 bytes。
- PyTorch 与纯 Python 推理的 100 步最大概率误差：`3.22e-7`。
- 当前云端 CPython：约 `30.7 µs/step`（仅用于回归，不代表 EV3）。
- 环境没有 `micropython`/`mpy-cross`，也没有 EV3，因此本轮不能诚实给出 EV3 延迟、堆内存或实机准确率。
- `src/learned_capture.py` 与已实机通过的 `demo_edge.py` 完全隔离；没有修改冻结巡线控制器。它必须 RIGHT 按下再松开才运动，CENTER 可急停，见白制动，30 s 超时，并只报告 `CONTACT CANDIDATE (unverified)`。

## 6. 复现

```bash
python -m venv .venv
.venv/bin/pip install -r requirements-ai.txt

# 原有回归
python tests/run_all.py

# 重新生成专家数据、行为克隆和 PPO（计算量较大）
.venv/bin/python -m learning.train --collect-expert --expert-episodes 30 --updates 150

# 24 个配对真值场景
.venv/bin/python -m learning.evaluate --episodes 24

# 推荐的接触模型训练、测试、导出
.venv/bin/python -m learning.contact_train
.venv/bin/python -m learning.export_contact
```

训练随机种子、划分偏移、阈值、训练耗时和逐项指标保存在 `models/*_metrics.json` 与 `docs/ai_full_eval.json`。

## 7. 下一次实机计划

1. 先跑原 `demo_edge.py` 回归，确认本轮没有影响巡线（代码层已隔离）。
2. 按现有计划运行 `probe_load.py`，空载、轻/重物体各至少 10 段，保存逐样本负载；不先运行学习运动脚本。
3. 在车轮架空状态运行 `learned_capture.py`，测 100 ms 周期是否超时、记录 EV3 单步延迟和内存。
4. 落地低速直线测试：无物体 10 次，要求 0 次候选；重物体 10 次后再测轻物体。所有结果目视标注，模型输出不能作为真值。
5. 只有上述门槛通过，才把 `ContactInference` 接到 `mission.py` 的 `tap_confirm` 旁做影子判断；先记录分歧，不立即取代 Final。
6. 下一研究迭代应采用“安全运动原语 + 学习高层 option”的半马尔可夫策略，而不是继续让 769–几千参数网络直接学习全部 200 ms 电机控制。Transformer 只有在积累真实轨迹并证明 GRU 记忆不足后才值得尝试。

