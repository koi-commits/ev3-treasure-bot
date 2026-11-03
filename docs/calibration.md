# Calibration Record（实物参数记录）

每次改动 `src/config.py` 后在此追加一行，记录日期与测量方法。

## Environment

| 项目 | 值 | 日期 |
|---|---|---|
| Pybricks 固件版本 | **4.1.2** | 2026-11-03 |
| Pybricks Code 网址（stable/beta） | beta.pybricks.com（本次改用 `pb` CLI 直连） | 2026-11-03 |
| Hub 名称 | **Hamster** | 2026-11-03 |
| 电池电压 (mV) | **7486–7487** | 2026-11-03 |
| 连接方式 | USB（Windows 侧 `COM5` / `\Device\USBSER000`） | 2026-11-03 |
| pybricks-cli | 1.0.0b1（`D:\python\Scripts\pb.exe`） | 2026-11-03 |

## Hardware

| 参数 | config 名称 | 初始量测 | 修正后 | 方法 / 备注 |
|---|---|---|---|---|
| 左电机端口 | `LEFT_MOTOR_PORT` | `Port.B` | **`Port.A`** | `diag.py` / `scan_ports.py` 扫描，B、C 为空 |
| 右电机端口 | `RIGHT_MOTOR_PORT` | `Port.C` | **`Port.D`** | 同上 |
| 颜色传感器端口 | `COLOR_SENSOR_PORT` | `Port.S3` | **`Port.S1`** | 同上，S2/S3/S4 为空 |
| 左电机方向 | `LEFT_MOTOR_DIRECTION` | `CLOCKWISE` | `CLOCKWISE`（确认） | `motor_id.py`，用户架空观察确认 |
| 右电机方向 | `RIGHT_MOTOR_DIRECTION` | `CLOCKWISE` | `CLOCKWISE`（确认） | 同上 |
| 齿轮组 | `GEARS` | `None` | `None`（轮子直装） | 目视 |
| 轮径 (mm) | `WHEEL_DIAMETER_MM` | 56.0 | **仍是占位值** | 未测量；只影响距离精度 |
| 轮距 (mm) | `AXLE_TRACK_MM` | 114.0 | **仍是占位值** | 未测量；只影响转向角度精度 |
| 传感器离地 (mm) | `SENSOR_HEIGHT_MM` | — | **未测量** | 反射率绝对量程偏低，怀疑偏高 |

`GEOMETRY_CONFIRMED = False`。

## Color（场地实测）

| 表面 | mean | min | max | std | 光照条件 |
|---|---|---|---|---|---|
| 黑色场地（标定用） | **1.0** | 0.9 | 1.1 | <0.2 | 室内，传感器朝下 |
| 白色边界（标定用白纸） | **11.2** | 11.2 | 11.3 | <0.1 | 同上 |
| 黑色场地（真实场地手推 30s，low20） | **0.7** | 0.0 | — | — | 实战场地，深色地面 |
| 白色边界（真实场地手推 30s，high20） | **11.5** | — | **14.5** | — | 实战场地，白色边界 |

阈值 = (0.7 + 12.0) / 2 = **6.35**（`config.edge_threshold()` 自动计算）

`config.py` 当前值：

```python
BLACK_REFLECTION = 0.7
WHITE_REFLECTION = 12.0
COLOR_CONFIRMED = True
```

> 绝对量程偏低（EV3 颜色传感器贴白纸通常能到 ~70），但黑白有 **约 16 倍分离**、噪声 < 0.2，判定可用。
> 黑白过渡在 0.5 s 内完成（很陡），因此 `EDGE_CONFIRM_SAMPLES = 2`（50 ms × 2 = 100 ms，
> 70 mm/s 时越界量约 7 mm）。

## 颜色识别（未完成）

`color_probe.py` 20 s 实测：

```
rgb 恒为 (25,21,21)    color 恒为 Color.NONE    hsv.s ≈ 13–20    hsv.v ≈ 5
```

要求依次放置红/绿/蓝/黄物体，读数**全程无变化**。
`rgb=(25,21,21)` 比场地深色的 `(5,5,5)` 亮，说明传感器在工作但没看到被测物体。

→ `TARGET_COLOR` / `FINISH_COLOR` 仍为 `None`。下一步需确认物体确实位于传感器正下方
（1–2 cm、贴住光斑）；若仍无变化，改用 RGB 通道比值 + 反射率联合判据。

## Calibration runs

| 日期 | 测试 | 期望 | 实测 | 调整 |
|---|---|---|---|---|
| 2026-11-03 | `straight(1000)` | 1000 mm | 未测 | — |
| 2026-11-03 | `turn(360)` | 360° | 未测 | — |
| 2026-11-03 | 沿边巡线 v1（比例控制） | 沿边走 | **冲出白边** | 废弃，改 bang-bang |
| 2026-11-03 | 沿边巡线 v2（bang-bang） | 沿边走 | 不冲出，但**转角抖振** | 加 CLEAR 盲走状态 |
| 2026-11-03 | 沿边巡线 v3（三状态机） | 沿边走 | **通过**，white_frac 0.10–0.17 | 定版 |
| 2026-11-03 | 一块颜色传感器 | 读数随表面变化 | 恒为 `refl=0.0 rgb=(1,0,0)` | **判定硬件故障，已更换** |
