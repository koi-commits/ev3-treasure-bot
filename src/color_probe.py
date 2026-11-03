r"""
color_probe.py — 识别色标用：不驱动电机，连续打印颜色读数。

    pbrun.cmd src\color_probe.py

跑 20 秒，每 300 ms 一行：`hsv  rgb  color`。
开始会响 3 声（3-2-1）。请依次把**红 / 绿 / 蓝 / 黄**的物体放到传感器正下方
（贴地、挡住 LED 光斑），每种停 3 秒左右。最后会统计每种 Color 的出现次数，
用来判断哪种颜色在这个光照条件下能被稳定区分。
"""

import config
from hardware import color, ev3

from pybricks.tools import run_task, wait

DURATION_MS = 20000
PRINT_MS = 300


async def main():
    ev3.screen.clear()
    print("=== color_probe: show RED, GREEN, BLUE, YELLOW under the sensor ===")
    print("threshold", config.edge_threshold())

    for n in (3, 2, 1):
        ev3.screen.clear()
        ev3.screen.print(str(n))
        ev3.speaker.beep(600, 120)
        await wait(1000)
    ev3.speaker.beep(1200, 400)

    seen = {}
    t = 0
    next_print = 0
    while t < DURATION_MS:
        hsv = await color.hsv()
        rgb = await color.rgb()
        c = await color.color()
        key = str(c)
        seen[key] = seen.get(key, 0) + 1
        ev3.screen.clear()
        ev3.screen.print(key)
        ev3.screen.print("h{} s{} v{}".format(hsv.h, hsv.s, hsv.v))
        if t >= next_print:
            print("{:5.1f}s  hsv=({:3d},{:3d},{:3d})  rgb=({:3d},{:3d},{:3d})  color={}".format(
                t / 1000.0, hsv.h, hsv.s, hsv.v, rgb[0], rgb[1], rgb[2], key))
            next_print += PRINT_MS
        await wait(100)
        t += 100

    print("--- color counts ---")
    for k, v in seen.items():
        print("  {} : {}".format(k, v))
    ev3.speaker.beep(800, 300)
    print("done")


run_task(main())
