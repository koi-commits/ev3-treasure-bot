"""
common.py — 测试脚本共用的小工具（只依赖 pybricks 官方模块）

Button 约定 (all test scripts):
  RIGHT  = 下一步 / confirm
  CENTER = 紧急停止 (Pybricks 默认 stop button，不要占用)
"""

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
