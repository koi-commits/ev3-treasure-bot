r"""
probe_dark.py — 诊断：颜色传感器为什么一直读到 0。

    pbrun.cmd src\probe_dark.py

屏幕会依次提示三个阶段，每一步按 EV3 上的 **RIGHT（右键）** 开始：
  1. COVER  : 用手完全遮住传感器镜头（贴住）
  2. WHITE  : 把传感器镜头贴近白纸（越近越好，1 cm 内）
  3. LIGHT  : 把小车抬起来，让传感器朝上对着灯/窗户

每个阶段采 3 秒。三段的数如果全都接近 0，说明传感器/线缆有问题；
如果 WHITE 或 LIGHT 明显变大，说明传感器正常，只是安装高度太高。
"""

import config
from hardware import color, ev3

from pybricks.parameters import Button
from pybricks.tools import run_task, wait

SAMPLE_MS = 3000
PERIOD_MS = 100


async def wait_right(prompt_line):
    ev3.screen.clear()
    ev3.screen.print(prompt_line)
    ev3.screen.print("press RIGHT")
    print(">>> " + prompt_line + " ... press RIGHT")
    while Button.RIGHT not in ev3.buttons.pressed():
        await wait(30)
    while Button.RIGHT in ev3.buttons.pressed():
        await wait(30)
    await wait(300)


async def measure(tag):
    vals = []
    rgbs = []
    ambs = []
    t = 0
    while t < SAMPLE_MS:
        vals.append(await color.reflection())
        rgbs.append(await color.rgb())
        ambs.append(await color.ambient())
        await wait(PERIOD_MS)
        t += PERIOD_MS

    rmin = min(vals)
    rmax = max(vals)
    rsum = sum(vals) / len(vals)
    asum = sum(ambs) / len(ambs)
    print("{:6s} refl avg={:6.1f} min={:6.1f} max={:6.1f}   amb avg={:6.1f}".format(
        tag, rsum, rmin, rmax, asum))
    print("       rgb samples:", rgbs[0], rgbs[len(rgbs) // 2], rgbs[-1])
    return rsum


async def main():
    ev3.screen.clear()
    print("=== sensor introspection ===")
    print("type:", type(color))
    try:
        print("dir:", dir(color))
    except Exception as e:
        print("dir failed:", repr(e))

    await wait_right("1 COVER lens")
    cover = await measure("COVER")

    await wait_right("2 WHITE paper")
    white = await measure("WHITE")

    await wait_right("3 LIGHT up")
    light = await measure("LIGHT")

    print("=== verdict ===")
    print("cover={:.1f} white={:.1f} light={:.1f}".format(cover, white, light))
    if white > 20 and white - cover > 10:
        print("SENSOR OK -> problem is mounting height / surface")
    elif light - cover > 10:
        print("SENSOR partly OK -> responds to light only; check LED/mounting")
    else:
        print("SENSOR DEAD or not really a color sensor -> check cable/port")

    ev3.speaker.beep(800, 400)


run_task(main())
