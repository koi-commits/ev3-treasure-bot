# EV3 Tap Shadow V2 实机操作卡

脚本：`src/probe_tap_shadow.py`（日志版本 `tap-shadow-v2.1`）。用途仅为采集和影子判断；模型不会控制 Final、捕获或交付。

## 运行前

1. 先运行 `demo_edge.py` 回归，确认已验证巡线仍正常。
2. 首次必须架空车轮运行一整轮，核对倒车、前进、左弧、右弧和 CENTER 急停。
3. 落地时使用深色、平整、至少前后各 40 cm 的区域；整个动作路径不得靠近白边、桌沿或人员。
4. 确认 V 形推板牢固、电池电压和目标物编号已写入实验记录；低于 7000 mV 时脚本会拒绝运动。
5. 启动并保留完整终端输出，例如：

```bat
pbrun.cmd src\probe_tap_shadow.py > logs\tap-shadow-YYYYMMDD-run01.txt
```

## 固定 12 次计划

每轮脚本依次提示：

- 1–3：`EMPTY`，推板和路径内无物体。
- 4–6：`CENTER_HEAVY`，重物体放在 V 口袋中心、与推板接触。
- 7–9：`SIDE_CONTACT`，物体只接触一侧斜臂，不算捕获。
- 10–12：`CENTER_LIGHT`，轻物体放在口袋中心。

每种状态要按目视事实放置，不要为了迎合模型调整位置。

## 每次操作

1. 按屏幕提示摆放物体，手离开车辆，按下并松开 RIGHT：只授权 tap。
2. 脚本制动后显示 `PRE_PROBE LEFT=0 RIGHT=1`：
   - RIGHT：物体中心已稳定处于 V 口袋内。
   - LEFT：无物体、侧擦、松动或中心仍在口袋外。
3. 观察并标注，**不要触碰车辆或物体**。
4. 手完全离开后，再次按下并松开 RIGHT：单独授权左右保持探针。
5. 探针制动后用同一标准标注 `POST_PROBE`。
6. 任意时刻出现危险立即按 CENTER。白边、卡滞、500 ms 无进展或阶段超时都会自动制动并写 `SHADOW_ABORT`/不完整结果。

若 LEFT 与 RIGHT 同时按下，标签无效，脚本会要求松开后重新选择；不要把这一提示当作一次标签。
每次 tap/探针授权均在提示前和松键后复查电池；任一次低于 7000 mV 都会制动并终止整轮。

`0to1` 表示探针使物体落座；`1to0` 表示探针导致脱落，是安全硬失败。无论试验是否完整，出现 `1to0` 都会写入 `SHADOW_SAFETY_FAIL` 并立即终止当前采集会话。

## 日志检查

- `SHADOW_SESSION`：版本、阈值、安全超时。
- `SHADOW_DATA`：50 ms 目标周期下的负载、里程计增量和影子概率。
- `SHADOW_LABEL`：探针前/后的人工真值。
- `SHADOW_RESULT`：完整性、转化、Final 规则、模型输出、电压、最大推理/采样周期。
- `SHADOW_ABORT`：未进入统计的安全中止；`SHADOW_FATAL` 表示异常后制动并终止整轮。
- `SHADOW_SAFETY_FAIL` / `SHADOW_SESSION_ABORT`：掉物或低电压触发的会话级安全熔断，出现后不得在同一轮继续。

汇总：

```bash
python tools/parse_shadow_logs.py logs/tap-shadow-YYYYMMDD-run01.txt
```

Final 规则必须与 `pre_probe` 比较，模型必须与 `post_probe` 比较；解析器已按此处理，并从准确率中排除不完整试验以及超过下述时延门槛的模型样本。安全转化则统计所有 `SHADOW_RESULT`，包括不完整试验；重点检查 `drop_safety_events` 和 `incomplete_drop_safety_events`。

## 验收门槛

先完成两轮架空测试，再累计至少五轮落地测试（60 次，每类 15 次）：

- 无任何未授权运动；RIGHT 必须按下再松开；CENTER 每次可立即停机。
- `max_infer_ms ≤ 20`；`max_cycle_ms ≤ 80`。超出则该轮不用于模型准确率统计。
- 所有中止单独复核，不得当作阴性样本。
- `1to0 = 0`；任何一次探针导致脱落都暂停落地试验并检查机械/速度。
- 至少 20 个 post 正样本和 20 个 post 负样本后，模型负样本误确认最多 1 次，正样本召回 ≥80%。
- 即使通过，模型仍保持影子模式；不得修改 Final 决策或宣称实机寻宝成功。

