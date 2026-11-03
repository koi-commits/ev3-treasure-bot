# 下周二（2026-11-10）实机测试方案

> 执行人：DSH　|　预计 2.5–3 h　|　所有新功能均为**离线开发、待实机验证**
> 原则：**按顺序做，每步有通过标准；不通过就停在那一步记录现象，不要跳步。**
> 每次运行都把输出存成日志：`pbrun.cmd src\xxx.py > logs\xxx_N.txt 2>&1`，跑完 `type logs\xxx_N.txt`。
> 急停：EV3 **CENTER 键**。车跑起来时一只手扶着 USB 线，防止被拖拽。

---

## 0. 准备（15 min，不开车）

**带齐**：充满的电池（≥ 7400 mV）、最终比赛用的**终点色标实物**、目标物体、卷尺/直尺、胶带、笔。

**PC 侧离线自检**（确认拷过去的工程是好的）：

```bat
cd <PROJ>
D:\python\python.exe tests\run_all.py
```

期望最后一行 `OK`（37 个测试，几秒钟）。

**用尺子量，填进 `docs/calibration.md` 的"待测量"表**（这些数字直接决定算法安全边界）：

| # | 测量项 | 用途 | 关键阈值 |
|---|---|---|---|
| M1 | 光斑中心 → 两轮轴线中点的前向距离 | `SENSOR_OFFSET_MM` | **> 80 mm 要报告**（拐角原地转时光斑会扫出白边） |
| M2 | 传感器底面离地高度 | `SENSOR_HEIGHT_MM` | 白纸反射率只有 11 可能是装太高 |
| M3 | 白色边界宽度 | 判断越界余量 | **< 40 mm 要报告** |
| M4 | 白边外侧是什么（桌面颜色/地面/悬空） | 出界后传感器读到什么 | 深色 = 危险组合 |
| M5 | 场地内部长 × 宽，算对角线 | `ARENA_DIAG_MM = 对角线 × 1.1` | 必须改 config |
| M6 | 终点色标：颜色、尺寸、位置（贴边/场内）、材质 | 色标方案 | 灰/黑/白色标不可用 |
| M7 | 目标物：尺寸、重量、形状 | 推板与负载判定 | |

改 `src/config.py`：`ARENA_DIAG_MM = <M5 × 1.1>`。

---

## 1. 基线回归：`demo_edge.py`（10 min）

确认硬件与 11-03 状态一致（**demo_edge 是冻结的已验证版本**）。

```bat
pbrun.cmd src\demo_edge.py > logs\demo_edge_1.txt 2>&1
```

车放场地内部、车头朝白边、距离 ≥ 20 cm，按 RIGHT。

**通过**：3 段都 `edge found`；每段 `white_frac` 0.05–0.3、`edge_twists` 3–6；目视不越界、拐角不抖。
**不通过** → 停。先查电池、传感器是否被碰歪，重跑 `calibrate.py`。后面所有步骤都依赖这一步。

---

## 2. 颜色传感器模式切换：`probe_timing.py`（5 min，不开车）

```bat
pbrun.cmd src\probe_timing.py > logs\probe_timing_1.txt 2>&1
```

听到每声"嘀"后，**整段 6 s 里一直用手把车在黑地面和白边之间慢慢来回推**。

记录 5 个 PHASE 的 `ms_per_loop`、`distinct`、`longest_same`。判读：

| 现象 | 结论 | 后续 |
|---|---|---|
| A、B 读数有变化，D 读数几乎不变 | **H1 成立**：模式来回切换导致读不到新值（11-03 色标失败的主因） | 新代码已按单模式设计，继续 |
| A、B、D 都有变化 | 切换不是问题 | 继续；若 C 的 `ms_per_loop` < 30 可考虑 `SENSE_MODE="dual"` |
| A 有变化、B（rgb）不变 | rgb 模式本身有问题 | 跳到第 3 步的 color() 段看看，记录后报告 |
| 全部不变 | 传感器/接线问题 | 停，换端口/传感器 |

E 段的 `ms_per_loop` 是巡线循环真实周期（期望约 50–60 ms），记下来。

---

## 3. 色标可见性：`color_probe.py`（10 min，不开车）

**把色标贴在比赛场地上**，用手推车让光斑依次压在：地面 → 白边 → 色标 → 其他颜色，每处停约 4 s。不要把色标举到传感器下面。

```bat
pbrun.cmd src\color_probe.py > logs\color_probe_1.txt 2>&1
```

**通过**：压到色标时 `rgb=` 与地面、白边明显不同，且 `cr/cg`（通道占比）与白边不同（白边 cr≈cg≈33；红色标 cr 明显 > 40）。
**不通过（色标 rgb 与地面/白边几乎一样）**：
1. 确认光斑真的在色标上（离色标边缘 ≥ 1 cm）。
2. 把传感器**降低一个 LEGO 单位（8 mm）**再试（只是挪位置，不加硬件）。**降低后必须重跑 `calibrate.py` + `probe_live.py`，更新 BLACK/WHITE，再重跑第 1 步 demo_edge。**
3. 仍不行 → 换高饱和度色标（红/蓝胶带最好），记录后报告。

---

## 4. 色标标定：`calibrate_marker.py`（10 min，不开车）

```bat
pbrun.cmd src\calibrate_marker.py > logs\calibrate_marker_1.txt 2>&1
```

按提示依次把车放在 **FLOOR → WHITE → FINISH**（EXTRA 不需要就等它超时跳过），每处按 RIGHT。**采样那 2–3 s 里用手把车在该表面上挪动 1–2 cm。**

**通过**：输出 `==== RESULT: PASS ====`。把打印出的这一段原样粘进 `src/config.py` 第 9 节，替换原来的三行：

```python
SENSE_MODE = "rgb"
RGB_EDGE_BRIGHT = ...
SIGNATURES = { ... }
```

然后看 LIVE 模式：手推车跨过各表面，屏幕标签应正确切换（地面绿灯、色标红灯、未识别橙灯），跨黑白边时**不应出现 `finish`**。CENTER 结束。

**FAIL** → 看 `PROBLEM:` 行：
- `overlap: floor ~ white` → 黑白在 rgb 下分不开，回到第 3 步检查高度。
- `floor/white mix ... looks like finish` → 色标颜色太接近灰色，换色标。

---

## 5. 新任务程序，先用已验证的传感模式：`mission.py` survey（10 min）

目的：只验证新的监督层（丢边/越界/堵转恢复），传感方式与 demo_edge 相同。

临时改 `config.py`：`SENSE_MODE = "reflection"`、`MISSION_MODE = "survey"`、`FOLLOW_MAX_MS = 60000`

```bat
pbrun.cmd src\mission.py > logs\mission_survey_1.txt 2>&1
```

**通过**：60 s 沿边不越界；结尾 `==== SUMMARY ====` 中 `white_frac` 0.05–0.3，`overruns` 很少（拐角偶发可以）。
记录 `lost / overruns / stalls / outside_returns` 的数字。

---

## 6. 主测：寻边 → 巡线 → 识别终点停车（30 min）

改回：`SENSE_MODE = "rgb"`（第 4 步的标定结果）、`MISSION_MODE = "edge_to_finish"`、`FOLLOW_MAX_MS = 120000`

**5 次**，每次换**随机起点与朝向**（规则：**车身和光斑都必须在深色场地内**），其中至少 1 次光斑压在白边上起步、1 次在角落附近起步：

```bat
pbrun.cmd src\mission.py > logs\mission_e2f_1.txt 2>&1
```

每次记录：起点描述、`result:`、`time_ms`、目视是否越界、停车时光斑是否在色标上。

**通过**：≥ 4/5 `result: PASS finish`，停在色标上，全程车身不出界。

| 现象 | 处理 |
|---|---|
| 沿边形态和 demo_edge 明显不同（rgb 模式下判边不稳） | 把 `logs\` 里 trace 行的 `rgb=` 发回分析；临时可试 `SENSE_MODE="dual"`（第 2 步 C 段 < 30 ms 时） |
| 经过色标不停 | 看日志里有无 `marker finish`；没有 → 标定问题，重做第 4 步并在色标上**开车**采样 |
| 没到色标就停（误报） | 记录停车位置的表面；可把 `MARKER_CONFIRM` 从 3 调到 5 |
| 整圈找不到 | `FOLLOW_MAX_MS` 不够绕一圈（看 `time_ms`），调大 |

---

## 7. 几何标定：`calibrate_geometry.py`（15 min）

需要 ≥ 1.2 m 直线空地。按脚本提示：直行 1000 mm 量实际距离、原地转 720° 看误差，按打印出的公式改 `WHEEL_DIAMETER_MM`、`AXLE_TRACK_MM`。各复测一次，误差 < 1% 则 `GEOMETRY_CONFIRMED = True`。

```bat
pbrun.cmd src\calibrate_geometry.py > logs\geometry_1.txt 2>&1
```

---

## 8.（有 V 形推板时）负载信号：`probe_load.py`（15 min）

推板要求见 `docs/object-strategy.md`。前方 ≥ 1 m 空地：

```bat
pbrun.cmd src\probe_load.py > logs\probe_load_1.txt 2>&1
```

三段：EMPTY（空推板）→ HOLD（物体放推板顶点）→ ONLINE（物体放车前约 30 cm）。
按输出 `SUGGEST:` 行改 `LOAD_ABS_MARGIN` 或把 `CAPTURE_MODE` 改为 `"off"`。
**通过**：HOLD 比 EMPTY 高出 > 4 倍 std，ONLINE 在约 300 mm 处触发。

---

## 9.（时间允许）完整任务：`mission.py` full（30 min）

`MISSION_MODE = "full"`。物体随机放在场地内，3 次。

```bat
pbrun.cmd src\mission.py > logs\mission_full_1.txt 2>&1
```

记录 `result:`、物体最终位置相对色标、是否有物体被推出场地。目标：≥ 2/3 物体进入色标区。

---

## 交回的东西

1. 整个 `logs\` 目录
2. 更新后的 `docs/calibration.md`（M1–M7 + 各步数字）和 `src/config.py`
3. 每步：通过 / 不通过 + 一句现象描述（尤其是目视看到、日志里看不到的东西）
