r"""
calibrate.py — 颜色标定：黑面 / 白面反射率，算出边界阈值。

    pbrun.cmd src\calibrate.py

需要人工配合（按 EV3 砖上的按键）：
  1. 把机器人放到**黑色**表面上（黑胶带/深色桌面），按 RIGHT
  2. 把机器人放到**白色**表面上（白纸/白桌边），按 RIGHT
  3. 程序打印 black/white/threshold，并给出行内建议

注意：颜色传感器距地面 5–10 mm 时读数最稳。太高会一直读到 0。
"""

import config
from common import log
from hardware import color, ev3

from pybricks.parameters import Button, Color
from pybricks.tools import run_task, wait


async def wait_right(prompt):
    log(ev3, prompt)
    ev3.light.on(Color.ORANGE)
    # 先等按键按下
    while Button.RIGHT not in ev3.buttons.pressed():
        await wait(30)
    # 再等松开
    while Button.RIGHT in ev3.buttons.pressed():
        await wait(30)
    ev3.light.on(Color.GREEN)
    await wait(250)


async def sample_block(tag, n):
    vals = []
    for _ in range(n):
        vals.append(await color.reflection())
        await wait(config.SAMPLE_PERIOD_MS)
    lo = min(vals)
    hi = max(vals)
    avg = sum(vals) / len(vals)
    print("{}: n={} min={:.1f} max={:.1f} avg={:.1f}".format(tag, n, lo, hi, avg))
    log(ev3, tag, "avg=", int(avg))
    return avg, lo, hi


async def main():
    ev3.screen.clear()
    log(ev3, "CALIBRATE")
    log(ev3, "1 black, 2 white")

    await wait_right("put on BLACK, press RIGHT")
    b_avg, b_lo, b_hi = await sample_block("BLACK", config.CAL_SAMPLES)
    ev3.speaker.beep(700, 200)

    await wait_right("put on WHITE, press RIGHT")
    w_avg, w_lo, w_hi = await sample_block("WHITE", config.CAL_SAMPLES)
    ev3.speaker.beep(1100, 250)

    print("--- result ---")
    print("BLACK_REFLECTION = {:.1f}".format(b_avg))
    print("WHITE_REFLECTION = {:.1f}".format(w_avg))

    if w_avg - b_avg < 8:
        print("WARN: black/white contrast too small ({:.1f}).".format(w_avg - b_avg))
        print("      -> sensor too high, or surface not really white/black.")
    else:
        thr = (b_avg + w_avg) / 2
        print("threshold = {:.1f}".format(thr))

    log(ev3, "see terminal for values")
    await wait(2000)


run_task(main())
