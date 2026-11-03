r"""
check_env.py — 阶段一：环境自检（不驱动电机，安全）。

    pbrun.cmd src\check_env.py

检查内容：
  * 固件/连接状态、电池电压
  * 端口表（A/D 电机，S1 颜色传感器）
  * 颜色传感器当前读数
  * config 里哪些参数还没在实物上确认
  * 按 RIGHT 键做一次屏幕/按键/蜂鸣器自检
"""

import config
from common import battery_check, log, warn_unconfirmed
from hardware import color, ev3, left, right

from pybricks.parameters import Color, Port
from pybricks.tools import run_task, wait


async def main():
    ev3.screen.clear()

    log(ev3, "=== check_env ===")

    mv = battery_check(ev3, config.LOW_BATTERY_MV)

    info = ev3.system.info()
    log(ev3, "hub:", info.get("name"))
    log(ev3, "usb:", info.get("host_connected_usb"),
        "ble:", info.get("host_connected_ble"))

    log(ev3, "--- ports ---")
    log(ev3, "L:", config.LEFT_MOTOR_PORT, "R:", config.RIGHT_MOTOR_PORT,
        "C:", config.COLOR_SENSOR_PORT)

    log(ev3, "angle L/R:", left.angle(), right.angle())

    r = await color.reflection()
    c = await color.color()
    amb = await color.ambient()
    log(ev3, "color refl/amb:", r, amb)
    log(ev3, "color class:", c)

    if r < 3:
        log(ev3, "WARN: no reflection -> sensor sees nothing")

    warn_unconfirmed(ev3, config)

    if mv < config.LOW_BATTERY_MV:
        ev3.light.on(Color.RED)
    else:
        ev3.light.on(Color.GREEN)

    log(ev3, "press RIGHT to beep")
    # 最多等 3 秒，避免从电脑端无人值守运行时卡住
    waited = 0
    while waited < 3000:
        if ev3.buttons.pressed():
            break
        await wait(50)
        waited += 50

    ev3.speaker.beep(600, 200)
    log(ev3, "=== OK ===")


run_task(main())
