r"""
test_color.py — 颜色传感器读数与黑白区分度检查。不驱动电机。

    pbrun.cmd src\test_color.py

流程：
  1. 连续采样 10 秒（每 100 ms 一次），打印 reflection / ambient / rgb / color
  2. 打印最小/最大/均值，判断黑白对比够不够
  3. 期间请用手把小车从黑面慢慢推到白面，或直接改放在不同表面上

正式的阈值标定用 calibrate.py（按键确认，采样更规范）。
"""

import config
from hardware import color, ev3

from pybricks.tools import run_task, wait

DURATION_MS = 10000
PERIOD_MS = 100


async def main():
    ev3.screen.clear()
    print("test_color: sampling {} ms, move robot over black/white".format(DURATION_MS))

    vals = []
    t = 0
    while t < DURATION_MS:
        r = await color.reflection()
        amb = await color.ambient()
        rgb = await color.rgb()
        c = await color.color()
        vals.append(r)
        print("{:5d}  refl={:5.1f} amb={} rgb={} color={}".format(t, r, amb, rgb, c))
        await wait(PERIOD_MS)
        t += PERIOD_MS

    lo = min(vals)
    hi = max(vals)
    avg = sum(vals) / len(vals)
    print("--- stats ---")
    print("min={:.1f} max={:.1f} avg={:.1f} span={:.1f}".format(lo, hi, avg, hi - lo))

    if hi - lo < 10:
        print("WARN: span < 10 -> 传感器可能没有贴近表面，或表面太单一")
    elif hi - lo > 40:
        print("GOOD: 黑白对比明显，可以跑 calibrate.py 记录阈值")
    else:
        print("OK: 有一定对比，建议再测一次更白/更黑的表面")

    ev3.speaker.beep(800, 200)
    print("done")


run_task(main())
