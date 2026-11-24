r"""
hardware.py — 唯一的设备构造点。

Pybricks 4.x 的硬性约束：Motor / ColorSensor / DriveBase 必须在**模块顶层**构造，
也就是在任何 run_task() 启动多任务之前。在 async 函数里 new 设备会抛
RuntimeError("This can only be called before multitasking starts.")。

所以所有脚本统一 `from hardware import ev3, drive, color, left, right`。

2026-11-17 硬件 v3：新增 TouchSensor（V 形推板顶点）与 GyroSensor。
这两个是可选的：端口没接 / 传感器没被识别时降级成 None 并打印警告，
让不依赖它们的旧脚本（demo_edge / calibrate / mission 的 reflection 模式）照常能跑。
"""

import config

from pybricks.ev3devices import ColorSensor, GyroSensor, Motor, TouchSensor
from pybricks.hubs import EV3Brick
from pybricks.robotics import DriveBase

ev3 = EV3Brick()

left = Motor(config.LEFT_MOTOR_PORT, config.LEFT_MOTOR_DIRECTION, config.GEARS)
right = Motor(config.RIGHT_MOTOR_PORT, config.RIGHT_MOTOR_DIRECTION, config.GEARS)

color = ColorSensor(config.COLOR_SENSOR_PORT)

color_right = None                      # 右侧朝下颜色传感器（2026-11-24 新增）
if config.USE_RIGHT_SENSOR:
    try:
        color_right = ColorSensor(config.COLOR_SENSOR_RIGHT_PORT)
    except Exception as e:              # 端口空 / 传感器不识别
        color_right = None
        print("WARN: no right color sensor on", config.COLOR_SENSOR_RIGHT_PORT, "->", repr(e))

try:
    touch = TouchSensor(config.TOUCH_SENSOR_PORT)
except Exception as e:                  # 端口空 / 传感器不识别
    touch = None
    print("WARN: no touch sensor on", config.TOUCH_SENSOR_PORT, "->", repr(e))

gyro = None                             # HEADING_SOURCE = "odometry"：不构造、不读陀螺
if config.HEADING_SOURCE == "gyro":
    try:
        gyro = GyroSensor(config.GYRO_SENSOR_PORT)
    except Exception as e:
        gyro = None
        print("WARN: no gyro sensor on", config.GYRO_SENSOR_PORT, "->", repr(e))

drive = DriveBase(
    left,
    right,
    config.WHEEL_DIAMETER_MM,
    config.AXLE_TRACK_MM,
)
