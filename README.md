# EV3 Treasure Bot — 自主寻宝与定点运输

LEGO MINDSTORMS EV3 双电机差速小车 + 1 个朝地颜色传感器，Pybricks v4.1.x MicroPython。

**当前阶段：Phase 2 — 自动寻边 + 沿边巡线已实机跑通**（色标识别与推送运输尚未完成）

> 详细工作日志见 [`docs/session-2026-11-03.md`](docs/session-2026-11-03.md)。

## 已实机验证的功能

| 功能 | 脚本 | 状态 |
|---|---|---|
| 设备识别、端口探测 | `src/check_env.py` / `src/scan_ports.py` / `src/diag.py` | ✅ |
| 左右电机归属与方向 | `src/motor_id.py` / `src/test_motors.py` | ✅ A=左, D=右, 均 CLOCKWISE |
| 黑白反射率标定 | `src/calibrate.py` | ✅ BLACK 1.0 / WHITE 11.2 |
| 真实场地反射率复核 | `src/probe_live.py` | ✅ low20 0.7 / high20 11.5 |
| **自动寻边 + 沿边巡线** | **`src/demo_edge.py`** | ✅ 用户目视确认轨迹正确 |
| 终点色标识别 | `src/color_probe.py` | ❌ 传感器未能区分颜色，待处理 |
| 探索状态机 | `src/mission.py` | ⚠️ 仍是旧版逻辑，待接入三状态机 |

## Directory

```
ev3-treasure-bot/
├── README.md
├── pb.cmd                    # 单条 pb 命令包装（已处理沙箱环境变量）
├── pbrun.cmd                 # 一条命令内完成 start→connect→run→stop
├── docs/
│   ├── calibration.md        # 实物参数记录表
│   └── session-2026-11-03.md # 本次会话完整工作日志
└── src/
    ├── config.py             # 端口/方向/标定/速度/阈值，唯一参数来源
    ├── hardware.py           # 唯一设备构造点（Pybricks 4.x 要求模块顶层构造）
    ├── common.py             # log / 按键等待 / 电池检查
    ├── edge.py               # 边界检测 + 三状态机沿边巡线
    ├── demo_edge.py          # ★ 自动寻边 + 沿边巡线（最小可跑版本）
    ├── mission.py            # 自主寻宝状态机（旧版，待重构）
    ├── check_env.py          # 环境与端口自检
    ├── test_motors.py        # 电机方向与基础运动
    ├── test_color.py         # 颜色传感器连续读数
    ├── calibrate.py          # 按键门控的黑白标定
    ├── motor_id.py           # 逐个转电机，人工确认左右归属
    ├── color_probe.py        # 色标识别能力评估
    ├── scan_ports.py         # 全端口类型探测
    ├── probe_live.py         # 手推过边界，记录真实场地反射率
    ├── probe_dark.py         # 三段诊断（遮住/白纸/见光）
    ├── probe_sensor.py / probe_lights.py / introspect.py / diag.py
```

## 快速开始

```bat
cd <PROJ>

rem 确认连接（hub 名 Hamster，USB，固件 4.1.2）
D:\python\Scripts\pb.exe scan

rem 硬件自检
pbrun.cmd src\check_env.py

rem 自动寻边 + 沿边巡线
pbrun.cmd src\demo_edge.py
```

`demo_edge.py` 运行方式：把小车放在场地**内部**、车头朝向白边、距离 ≥ 20 cm，
然后按 EV3 上的**右键（→）**启动。

## 为什么用 .cmd 包装脚本

`pb` 默认走 Windows 命名管道 + `%LOCALAPPDATA%\pybricks`、`%TEMP%\pybricks`，
这两条路径在本机沙箱下会 `PermissionError [WinError 5]`。
`pb.cmd` / `pbrun.cmd` 用 `pb_core/paths.py` 的官方覆盖点把运行时目录、状态目录、
`USERPROFILE` 都指到项目内 `.pb\`，并用 `PB_ENDPOINT=tcp://127.0.0.1:47823` 绕开命名管道。

另外脚本会在 `run` 之前先 `pb stop`（清掉上次中断残留的程序），
且 `stop` / `run` 都显式带 `--hub Hamster`（否则会清空 hub 选择导致 `No hub selected`）。

## Pybricks 4.x 的硬约束

1. **设备对象必须在模块顶层构造**（`run_task()` 之前）。
   在 `async` 里 `Motor(port)` 会抛
   `RuntimeError('This can only be called before multitasking starts.')`。
   → 所有设备统一在 `src/hardware.py` 构造。
2. MicroPython 异常**没有 `__name__`**，用 `repr(e)`。
3. `ColorSensor` 在 4.1.2 上**没有** `lights_on/lights_off`。
4. `hsv().s/.v`、`ambient()` 是 float，`"{:5d}".format()` 会抛 `ValueError`。
5. 带 `await` 的 API：`run_angle` / `straight` / `turn` / `wait` / `reflection` / `color` / `hsv` / `speaker.beep`。
   非 awaitable：`run` / `stop` / `drive` / `angle` / `distance` / `buttons.pressed` / `screen.print`。

## 沿边巡线算法

`src/edge.py` 的 `follow_edge_bangbang()` 是三状态机：

| 状态 | 行为 |
|---|---|
| `CRUISE` | 60 mm/s 前进 + 朝白边偏 22 °/s，斜向逼近边界 |
| `ESCAPE` | 见白立刻**原地**转 130 °/s，固定 350 ms（约 45°），转完仍在白区就再续（最多 2 次） |
| `CLEAR` | 逃逸后盲走 400 ms，**忽略传感器**，切断抖振反馈回路 |

净效果是沿边界走"锯齿"：不越界，又持续沿边前进。

> 不要改回纯比例控制。`drive.drive(speed, steer)` 保持前进速度，
> 等传感器读到白色时车头已经深入白区，会"扫到白边→有转向→照样冲出白边"。

## 标定规则

- `config.py` 中 `*_CONFIRMED` 为 `False` 的项仍未在实物上确认。
- 实测后：改数值 → 把对应 `*_CONFIRMED` 设为 `True` → 抄进 `docs/calibration.md`。
- 几何修正：`straight(1000)` 走不够 → 轮径调小；`turn(360)` 转不够 → 轮距调大。先轮径后轮距。

## Docs

- Pybricks API：https://docs.pybricks.com/en/stable/ （v4.1.1）
- 相关模块：`ev3devices` / `hubs.EV3Brick` / `robotics.DriveBase` / `tools`
