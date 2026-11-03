r"""
color_probe.py — 色标可见性快速检查（2026-11-03 晚重写：单模式读取）。不驱动电机。

    pbrun.cmd src\color_probe.py

旧版每轮连调 hsv()/rgb()/color() 三种模式，读数可能是切换造成的旧值（见 probe_timing.py）。
新版分两段，每段只用一种模式：
  段 1（20 s）rgb() 单模式：依次把 地面 / 白边 / 终点色标 / 其他颜色 放到光斑正下方，每种约 4 s
  段 2（10 s）color() 单模式：同样操作，看 Pybricks 内置颜色枚举是否可用（仅供参考）

**色标要贴地放在比赛场地上，让小车开上去**（光斑高度与比赛时一致），不要拿在手里举到传感器下。
"""

from hardware import color, ev3

from pybricks.tools import StopWatch, run_task, wait

PRINT_MS = 250


async def rgb_phase(ms):
    ev3.speaker.beep(1200, 200)
    print("=== rgb() only ===")
    sw = StopWatch()
    nxt = 0
    lo = [999, 999, 999]
    hi = [-1, -1, -1]
    while sw.time() < ms:
        rgb = await color.rgb()
        for i in range(3):
            lo[i] = min(lo[i], rgb[i])
            hi[i] = max(hi[i], rgb[i])
        if sw.time() >= nxt:
            s = rgb[0] + rgb[1] + rgb[2]
            ch = "" if s <= 0 else "cr={:.0f} cg={:.0f}".format(100 * rgb[0] / s, 100 * rgb[1] / s)
            print("{:5.1f}s rgb={} bright={:.1f} {}".format(sw.time() / 1000, rgb, s / 3, ch))
            ev3.screen.clear()
            ev3.screen.print(str(rgb))
            nxt += PRINT_MS
        await wait(20)
    print("rgb range: r {}..{}  g {}..{}  b {}..{}".format(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))


async def color_phase(ms):
    ev3.speaker.beep(800, 200)
    print("=== color() only ===")
    sw = StopWatch()
    seen = {}
    nxt = 0
    while sw.time() < ms:
        c = str(await color.color())
        seen[c] = seen.get(c, 0) + 1
        if sw.time() >= nxt:
            print("{:5.1f}s color={}".format(sw.time() / 1000, c))
            ev3.screen.clear()
            ev3.screen.print(c)
            nxt += PRINT_MS
        await wait(20)
    print("color counts:", seen)


async def main():
    ev3.screen.clear()
    ev3.screen.print("color_probe")
    for n in (3, 2, 1):
        ev3.screen.print(str(n))
        await wait(1000)
    await rgb_phase(20000)
    await color_phase(10000)
    ev3.speaker.beep(500, 400)
    print("done")


run_task(main())
