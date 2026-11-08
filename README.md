# EV3 Treasure Bot — 自主寻宝与定点运输

> **过程存档**：[`开发全过程 / 版本时间线`](docs/development-timeline.md) · [`初版任务方案与五人分工`](docs/project-overview.md) · [`首次实机日志`](docs/session-2026-11-03.md) · [`两轮离线改进`](docs/offline-2026-11-03.md) · [`最终交接`](docs/final-handoff-2026-11-03.md)
>
> Git 提交中的早期版本为依据 ZIP 文件**事后导入的快照**，并非完整原始逐次提交历史。最新分支以 Final 版为准；**只有自动寻边与沿边巡线已经获得实机通过**，完整寻宝仍待测试。

> **AI 阶段一/接触复核（2026-11-08）**：端到端低层 GRU 在独立测试为 0/24，不能替代 Final；旧接触模型的 2/50 虚报口径已撤回。当前推荐实验是 Final tap+seat 后的保持探针 GRU，仍只允许影子运行。见 [`docs/ai-learning-2026-11-08.md`](docs/ai-learning-2026-11-08.md)、[`docs/ai-contact-event-2026-11-08.md`](docs/ai-contact-event-2026-11-08.md) 与可直接执行的 [`docs/tap-shadow-runbook.md`](docs/tap-shadow-runbook.md)。


LEGO MINDSTORMS EV3 双电机差速小车 + 1 个朝地颜色传感器，Pybricks 4.1.2 MicroPython。

**当前阶段**

- Phase 2：自动寻边与沿边巡线，已于 11-03 **实机通过**。
- Phase 3：完整任务（色标识别、搜索、确认、推送、交付），代码已完成，并经两轮离线开发（接触式物体仿真 + 真 MicroPython 执行），**全部待 11-10 实机验证**。
- 离线结论（仿真，不是实机成功率）：寻边 → 巡线 → 终点停车接近 100%；完整搬运在"物体在场内、转角已标定"时约 50–60%；物体靠边时约 25%。详见 `docs/offline-2026-11-03.md` 第二轮。

| 文档 | 内容 |
|---|---|
| [`docs/tuesday-test-plan.md`](docs/tuesday-test-plan.md) | ★ 下次实机的执行顺序、起点卡、通过标准、决策分支（v2） |
| [`docs/run-sheet.csv`](docs/run-sheet.csv) | 实机记录表（每次运行一行，含目视结果） |
| [`docs/offline-2026-11-03.md`](docs/offline-2026-11-03.md) | 离线开发：修了什么、测试结果、仍需实机回答的问题 |
| [`docs/object-strategy.md`](docs/object-strategy.md) | 目标物搜索/推送/交付方案与 V 形推板设计要求 |
| [`docs/session-2026-11-03.md`](docs/session-2026-11-03.md) | 11-03 实机日志（环境打通、三状态巡线的由来） |
| [`docs/calibration.md`](docs/calibration.md) | 实物参数记录 + 待测量清单 |

## 功能状态

| 功能 | 脚本 | 状态 |
|---|---|---|
| 设备识别、端口、电机方向 | `check_env.py` / `motor_id.py` / `test_motors.py` | ✅ 实机 |
| 黑白反射率标定 | `calibrate.py` / `probe_live.py` | ✅ 实机（阈值 6.35） |
| **寻边 + 三状态沿边巡线** | **`demo_edge.py`**（冻结基线） | ✅ 实机 |
| 颜色模式切换诊断 | `probe_timing.py` | 🧪 待实机 |
| 终点色标识别与标定 | `color_probe.py` / `calibrate_marker.py` | 🧪 待实机（离线通过） |
| 完整任务：寻边→巡线→识别终点停车 | `mission.py`（`edge_to_finish`） | 🧪 待实机（仿真 20/20） |
| 异常恢复：丢边/越界/堵转/出界/起步压白 | `mission.py` 监督层 | 🧪 待实机（仿真） |
| 场上转角自标定 + 色标定位 | `mission.py`（`calib`） | 🧪 待实机 |
| 搜索 + 负载嫌疑 + tap 确认 | `mission.py`（`capture`） | 🧪 待实机，需推板 |
| 搜索 + 推送 + 交付 | `mission.py`（`full`） | 🧪 待实机，需带侧板的 V 形推板 |
| 推物体负载信号 | `probe_load.py` | 🧪 待实机 |
| 轮径/轮距标定 | `calibrate_geometry.py` | 🧪 待实机 |

## 目录

```
ev3-treasure-bot/
├── pb.cmd / pbrun.cmd        # pb CLI 沙箱包装（见下）
├── docs/                     # 见上表
├── logs/                     # 实机运行日志（tools/parse_logs.py 汇总）
├── tools/parse_logs.py       # 把 logs\*.txt 汇总成 Markdown 表
├── src/                      # 全部在 EV3 上运行
│   ├── config.py             # 唯一参数来源（[TBD] = 未实测）
│   ├── hardware.py           # 唯一设备构造点（Pybricks 4.x 必须模块顶层构造）
│   ├── fsm.py                # 纯逻辑：三状态巡线机、色标分类、负载监测、里程计
│   ├── sensing.py            # 传感模式 reflection / rgb / dual -> 白边 + 色标
│   ├── calib.py              # 纯逻辑：色标标定分析
│   ├── edge.py               # 巡线封装（follow_edge_bangbang 调用 fsm.EdgeFollower）
│   ├── mission.py            # ★ 完整任务状态机
│   ├── demo_edge.py          # ★ 已验证基线（冻结）
│   ├── calibrate_marker.py / probe_timing.py / color_probe.py / probe_load.py / calibrate_geometry.py
│   ├── calibrate.py / probe_live.py / check_env.py / test_motors.py / motor_id.py ...
│   └── common.py
└── tests/                    # PC 离线测试（不上机）
    ├── run_all.py            # python tests/run_all.py
    ├── report_scenarios.py   # 仿真敏感性矩阵（地面真值评判）
    ├── evaluate.py           # 起点/物体生成 + 地面真值评判（成功 / 可接受 / 越线 / 物体出界 / 虚报）
    ├── sim.py / harness.py   # 2D 场地仿真（接触式物体、滑行停车、负载漂移）+ 原样运行 src 脚本
    ├── viz2.py               # 画一次运行的车/物体轨迹
    ├── mock/pybricks/        # Pybricks API 模拟
    ├── reference_20261103.py # 11-03 实测版巡线状态机（对拍基准）
    ├── test_fsm.py / test_calib.py / test_mission_sim.py
    └── mpy_logic_check.py / mpy_sim_check.py   # 在真正的 MicroPython 下执行
```

## 任务流程（`mission.py`）

```
start_check ─► seek_edge ─► follow（已验证三状态机 + 每 tick 色标检测）─► 终点色标 ─► 停车      [edge_to_finish]
                                                                     │
full：────────────────────────────────────────────────────────────────┘
  calibrate_and_map：离开边 → 原地转 720° → 回到同一条边沿边走（经过色标）→ 测边方向
                     → 转角比例 k + 航向 + 色标中心 + 交付目标点
  bounce：沿场地对角线方向直行到边（碰边入射角 ≥ 30°）→ 负载嫌疑 → tap + seat 确认
  deliver：纯追踪（只走前进弧线）→ 场内待命点 → 正对色标推进，让物体中心到目标点 → 倒车放下
  推送中负载回落 → 判定物体丢失 → 回到搜索（不虚报）
```

监督层：丢边 → 重新寻边；越界 → 倒车重置；堵转 → 后退转开；直线超过对角线不见白 → 判定在场外并返回；起步光斑在白线上 → 先倒车。

`MISSION_MODE`：`edge_to_finish` / `survey` / `calib` / `capture` / `full`
`SENSE_MODE`：`reflection`（基线，无色标）/ `rgb` / `dual`
`FOLLOW_PROFILE`：`validated`（实机通过）/ `steep`（仿真：轮子压线更浅）

## 快速开始

```bat
cd <PROJ>
D:\python\python.exe tests\run_all.py          rem 离线自检，期望 OK
pbrun.cmd src\demo_edge.py                      rem 基线回归
pbrun.cmd src\mission.py                        rem 完整任务（先按测试方案标定）
```

正常起点规则：**车身和光斑都在深色场地内**，朝向随机。B1 属于单独的白边起步异常恢复测试，不计入正常起点成功率。

**运行 `mission.py`、`test_motors.py` 和 `probe_load.py` 必须按 RIGHT 才会启动相应电机动作，不会等待超时后自行启动。** CENTER 急停。

## 为什么用 .cmd 包装脚本

`pb` 默认走 Windows 命名管道，并使用 `%LOCALAPPDATA%\pybricks`、`%TEMP%\pybricks`。这两条路径在本机沙箱下都会报 `PermissionError [WinError 5]`。

`pb.cmd` / `pbrun.cmd` 利用 `pb_core/paths.py` 的官方覆盖点，把运行时目录、状态目录和 `USERPROFILE` 都指到项目内的 `.pb\`，并设置 `PB_ENDPOINT=tcp://127.0.0.1:47823` 绕开命名管道。

`run` 之前会先执行 `pb stop --hub Hamster`，清掉上次中断后残留在 hub 上的程序。

## Pybricks 4.x 硬约束（实机踩过）

1. 设备对象必须在**模块顶层**构造（`run_task()` 之前），统一放在 `hardware.py`。
2. MicroPython 异常没有 `__name__`，用 `repr(e)`。
3. `ColorSensor` 没有 `lights_on/lights_off`。
4. `hsv().s/.v`、`ambient()` 是 float，不能用 `{:d}` 格式化。
5. 带 `await` 的 API（`straight`/`turn`/`wait`/`reflection`/`rgb`/`beep`…）在 async 里**必须 await**。不 await 的话，紧接着的下一条驱动命令会立刻覆盖它。若需要"启动后轮询"，用 `drive.straight(mm, wait=False)` 加 `drive.done()`。
6. **同一循环里不要轮流读不同的颜色模式**（reflection/rgb/color/hsv），见 `probe_timing.py`。
7. 在白线处停车用 `drive.brake()`：`stop()` 是滑行，会多冲出一段。
8. 巡线结束后离开边要"原地转向场内再前进"，不能直接倒车（车头斜朝场内时，倒车会往白线走）。

## 不要做的事

- 不要把巡线改回纯比例控制。实测会"扫到白边→有转向→照样冲出白边"。
- 不要修改 `demo_edge.py` 的行为。它是硬件状态的回归基准。
- 未经实测，不要把 `[TBD]` 参数当成测量值，也不要把仿真结果当作实机通过。
- 不要相信 `DELIVERED`：机器人看不见物体，交付结果必须目视判定。

## Docs

- Pybricks API：https://docs.pybricks.com/en/stable/ （v4.1.x）
