# Calibration Record（实物参数记录）

每次改动 `src/config.py` 后在此追加一行，记录日期与测量方法。

## Environment

| 项目 | 值 | 日期 |
|---|---|---|
| Pybricks 固件版本 | | |
| Pybricks Code 网址（stable/beta） | | |
| Hub 名称 | | |
| 电池电压 (mV) | | |

## Hardware

| 参数 | config 名称 | 初始量测 | 修正后 | 方法 / 备注 |
|---|---|---|---|---|
| 左电机端口 | `LEFT_MOTOR_PORT` | | | check_env.py |
| 右电机端口 | `RIGHT_MOTOR_PORT` | | | check_env.py |
| 颜色传感器端口 | `COLOR_SENSOR_PORT` | | | check_env.py |
| 左电机方向 | `LEFT_MOTOR_DIRECTION` | | | test_motors P1 |
| 右电机方向 | `RIGHT_MOTOR_DIRECTION` | | | test_motors P1 |
| 齿轮组 | `GEARS` | | | 数齿 |
| 轮径 (mm) | `WHEEL_DIAMETER_MM` | | | 尺子/胎侧印字 → straight(1000) |
| 轮距 (mm) | `AXLE_TRACK_MM` | | | 两轮中线距 → turn(360) |
| 传感器离地 (mm) | `SENSOR_HEIGHT_MM` | | | |

## Color（场地实测）

| 表面 | mean | min | max | std | 光照条件 |
|---|---|---|---|---|---|
| 黑色场地 | | | | | |
| 白色边界 | | | | | |
| 终点标记色（待定） | | | | | |

阈值 = (black_mean + white_mean) / 2 = ____

## Calibration runs

| 日期 | 测试 | 期望 | 实测 | 调整 |
|---|---|---|---|---|
| | straight(1000) | 1000 mm | | |
| | turn(360) | 360° | | |
