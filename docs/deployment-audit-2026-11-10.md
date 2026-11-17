# 部署核查报告 — 2026-11-10

回答问题：**"跑在 EV3 上的，到底是不是我们仓库里的 Final 算法和 ContactGRU？"**

基线：`koi-commits/ev3-treasure-bot` main @ `4c3224d`，本地快照提交 `db65b41`。

---

## 1. 上传到 EV3 的代码集合（实测，非推断）

`pb run` 的实际行为（`D:\python\Lib\site-packages\pb_cli\commands\program.py` 的 `_run`）：

```python
roots = await project.module_roots(script)
compiled = await pb_client.compile_program(script, roots=roots)
program = compiled.pack(legacy_main=...)
await service.download(program, ...)
```

即**入口脚本 import 的本地模块会被一起打包下载**，不是只传一个文件。
把同一步在 PC 上复现，得到实际字节（`tools/deploy_check.py`，日志 `logs/deploy_check_1.txt`）：

| 入口 | 打进 hub 的模块 | 字节 |
|---|---|---|
| `src/mission.py` | mission 11365 + config 3047 + common 793 + fsm 4478 + hardware 433 + sensing 1367 | 21483 |
| `src/probe_tap_shadow.py` | probe_tap_shadow 6306 + config 3047 + **tap_probe_weights 13440** + common 793 + **contact_features 467** + **contact_inference 780** + fsm 4478 + hardware 433 | 29744 |

结论：EV3 上运行的是仓库里的 `mission.py` / `fsm.py` / `sensing.py` / `config.py` / `hardware.py`，
**没有**任何本机另写的替代实现（本机新增的 `_check_model.py` / `_smoke_motor.py` / `_probe_marker.py`
是诊断脚本，不进 mission 的依赖图）。

## 2. 模型对应关系（`models/README.md` 官方说明）

| 训练产物 | MicroPython 导出 | 用途 | 是否部署 |
|---|---|---|---|
| `tap_probe_gru_v1.pt` | `src/tap_probe_weights.py`（841 参数，IN 9 / HID 12 / 阈值 0.85 / 防抖 1） | **当前唯一推荐、仅影子模式** | ✅ 由 `probe_tap_shadow.py` 加载 |
| `contact_gru.pt` | `src/contact_weights.py`（7 输入 / 12 隐层 / 阈值 0.95 / 防抖 3） | 遗留负结果，仅 `learned_capture.py` 与单元测试 | ⚠️ 不在 Final 路径 |
| `contact_gru_event_v2.pt` | — | 负结果（首触发 2/4） | ❌ |
| `gru_policy.pt` | — | 行为克隆负结果，README 明确 "do not deploy" | ❌ |
| `final_expert.npz` | — | 行为克隆训练数据 | ❌（PC 训练用） |

**重要事实**：`mission.py`（Final 算法）**不 import 任何 GRU**。
它 import 的是 `config / common / fsm / hardware / sensing / pybricks`。
Final 的捕获判定走 `fsm.LoadMonitor`（负载阈值），这是仓库原本的设计
（`models/README.md`：contact GRU "do not use it to trigger delivery"）。
GRU 在本项目里的定位是 **shadow-only**，由 `probe_tap_shadow.py` 采集验证。

所以"ContactGRU 影子模式"要验证的入口就是 `src/probe_tap_shadow.py`，不是 mission。

## 3. 发现的部署阻断 bug（已修）

`src/contact_inference.py` 原文：

```python
try:
    from umath import exp, tanh
except ImportError:
    from math import exp, tanh
```

在 EV3 上实跑：

```
Traceback (most recent call last):
  File "_check_model.py", line 23, in <module>
  File "contact_inference.py", line 6, in <module>
ImportError: can't import name tanh
```

原因：Pybricks 的 MicroPython 构建里 `math` **没有 `tanh`**（标准 MicroPython 也没有）。
CPython 有 `math.tanh`，所以 `tests/test_learning_runtime.py` 一直是绿的 ——
**模型从来没有在 EV3 上真正跑起来过**。

修法（适配层，未改模型与数值语义）：只在 `math.tanh` 缺失时补纯 Python 实现
（`2*sigmoid(2x)-1` 形式，|x|≥15 饱和），并保留 `umath` 优先路径。

## 4. 修复后的实机证据

**PC 参考**（`tools/model_parity.py` → `logs/model_parity_pc_2.txt`）
**EV3 实机**（`src/_check_model.py` → `logs/model_check_ev3_2.txt`）

同一段确定性特征序列，逐位对比：

| 步 | PC 概率 | EV3 概率 | 差 |
|---|---|---|---|
| 0 | 0.600529 | 0.600529 | 0 |
| 5 | 0.700268 | 0.700268 | 0 |
| 10 | 0.945961 | 0.945961 | 0 |
| 15 | 0.979383 | 0.979384 | 1e-6 |
| 20 | 0.982961 | 0.982961 | 0 |
| 25 | 0.976901 | 0.976901 | 0 |
| 30 | 0.985293 | 0.985293 | 0 |
| 35 | 0.988985 | 0.988985 | 0 |
| 39 | 0.982627 | 0.982627 | 0 |

- 参数量 841，与 `models/tap_probe_gru_v1_export.json` 记录的 `"parameters": 841` 一致。
- 内存：`mem_free_before=964816` → `mem_free_after=729888`（剩余约 713 KB）。
- **推理耗时：40 步 / 2435 ms，其中 2000 ms 是 `wait(50)` → 约 11 ms/步**（导出 json 记的 CPython 是 31 µs/步）。
  `SAMPLE_PERIOD_MS = 50`，即单步推理占周期的 ~22%，可行但需注意不要再多加开销。

## 5. 相对上游基线改了什么（`git diff --name-status db65b41`）

```
M  pbrun.cmd                    hub 名自动探测（Hamster -> Platypus），无 hub 时明确报错
M  src/config.py                端口/方向/黑白阈值/几何/SENSE_MODE/签名/RED_RULE/FOLLOW_MAX_MS
M  src/fsm.py                   新增 is_red() 比例判据（红标兜底），不改状态机
M  src/sensing.py               分类器后接 RED_RULE 兜底
M  src/contact_inference.py     修 EV3 缺 tanh（部署阻断）
M  src/probe_timing.py          修 MemoryError（Track 累计统计，只留前 400 样本）
M  tests/test_mission_sim.py    3 个用例显式传"未标定"配置，不再依赖出厂默认值
A  src/_check_model.py          模型部署核查（诊断）
A  src/_probe_marker.py         色标实测取样（诊断）
A  src/_smoke_motor.py          电机冒烟（诊断）
A  tools/deploy_check.py        编译产物模块清单核查
A  tools/model_parity.py        PC/EV3 数值对齐参考
```

**未改动**：`mission.py`、`edge.py`、`probe_tap_shadow.py`、`learned_capture.py`、
`contact_weights.py`、`tap_probe_weights.py`、`fsm.py` 的状态机部分、`learning/`、`models/`。

## 6. 尚未在实机验证的部分

- `probe_tap_shadow.py` 12 次试验（EMPTY / CENTER_HEAVY / SIDE_CONTACT / CENTER_LIGHT 各 3 次）
  需要 V 形口袋 + 物体 + 每次动作的 RIGHT 授权，未开始。
- 该脚本会打印 `SHADOW_DATA ... p=...` 与 `SHADOW_RESULT ... max_infer_ms=... max_cycle_ms=...`，
  这是"推理在真实传感器数据上运行"的最终证据。
