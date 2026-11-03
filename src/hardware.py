r"""
hardware.py — 唯一的设备构造点。

Pybricks 4.x 的硬性约束：Motor / ColorSensor / DriveBase 必须在**模块顶层**构造，
也就是在任何 run_task() 启动多任务之前。在 async 函数里 new 设备会抛
RuntimeError("This can only be called before multitasking starts.")。

所以所有脚本统一 `from hardware import ev3, drive, color, left, right`。
"""

import config

from pybricks.ev3devices import ColorSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.robotics import DriveBase

ev3 = EV3Brick()

left = Motor(config.LEFT_MOTOR_PORT, config.LEFT_MOTOR_DIRECTION, config.GEARS)
right = Motor(config.RIGHT_MOTOR_PORT, config.RIGHT_MOTOR_DIRECTION, config.GEARS)

color = ColorSensor(config.COLOR_SENSOR_PORT)

drive = DriveBase(
    left,
    right,
    config.WHEEL_DIAMETER_MM,
    config.AXLE_TRACK_MM,
)
