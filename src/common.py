r"""
common.py — 测试脚本共用的小工具（只依赖 pybricks 官方模块）

Button 约定 (all scripts):
  RIGHT  = 下一步 / confirm
  CENTER = 紧急停止 (Pybricks 默认 stop button，不要占用)
"""

import config

from pybricks.parameters import Button, Color
from pybricks.tools import wait


def log(ev3, *args):
    """同时输出到电脑终端 (print) 和 EV3 屏幕。"""
    print(*args)
    ev3.screen.print(*args)


async def wait_for_button(ev3, button=Button.RIGHT, prompt="RIGHT = next"):
    """等待按下并松开某个按键。async，内部用 await wait()。"""
    log(ev3, prompt)
    ev3.light.on(Color.ORANGE)
    while button not in ev3.buttons.pressed():
        await wait(20)
    while button in ev3.buttons.pressed():
        await wait(20)
    ev3.light.on(Color.GREEN)


async def wait_button_or_timeout(ev3, timeout_ms=5000,
                                 button=Button.RIGHT, prompt=None):
    """等按键；超时自动继续。返回 True 表示是按键触发。

    这样脚本既能人工操作，也能从电脑端无人值守地跑完。
    """
    if prompt:
        log(ev3, prompt)
    ev3.light.on(Color.ORANGE)
    t = 0
    pressed = False
    while t < timeout_ms:
        if button in ev3.buttons.pressed():
            pressed = True
            break
        await wait(25)
        t += 25
    if pressed:
        while button in ev3.buttons.pressed():
            await wait(25)
    ev3.light.on(Color.GREEN)
    return pressed


def battery_check(ev3, low_mv):
    mv = ev3.battery.voltage()
    log(ev3, "Battery:", mv, "mV")
    if mv < low_mv:
        log(ev3, "WARN: low battery")
    return mv


def warn_unconfirmed(ev3, cfg):
    items = cfg.unconfirmed_items()
    if items:
        log(ev3, "TBD:", ",".join(items))


async def aread(value):
    """读传感器：Pybricks 4.x 的读值可能返回 awaitable，也可能直接是数字。

    ColorSensor.reflection()/rgb() 已实机确认必须 await；Motor.load() 已实机确认不用 await。
    TouchSensor.pressed() / GyroSensor.angle()/speed() 是否 awaitable 尚未实机确认
    （[TBD - confirm on hardware]，probe_new_hw.py b 段会打印原始类型），
    所以统一走这里：数字 / bool / None 原样返回，其余 await 一次。
    """
    if value is None or isinstance(value, (int, float, bool)):
        return value
    return await value


def require_gyro_sign():
    """陀螺仪符号硬闸门：没测出来就报错，绝不默认 +1。

    GYRO_POSITIVE_DIRECTION 只能在实机 probe_new_hw.py c 段测出后回填 config；
    悄悄按 +1 处理只会让基于陀螺仪的转向反着走。
    """
    sign = config.GYRO_POSITIVE_DIRECTION
    if sign is None:
        raise ValueError("GYRO_POSITIVE_DIRECTION is None [TBD]: "
                         "run probe_new_hw.py section c on the robot, then set it in config.py")
    return sign
