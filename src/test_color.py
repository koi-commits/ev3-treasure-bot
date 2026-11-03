"""
test_color.py — 阶段一：颜色传感器黑白读数稳定性
Pybricks v4.1.x, EV3. ColorSensor 的 reflection()/color()/hsv() 都是 awaitable。

流程 Flow：
  1. 标定 BLACK：把传感器放在黑色场地上（不动），RIGHT -> 采 30 个样
  2. 标定 WHITE：放在白色边界上，RIGHT -> 采 30 个样
  3. 打印统计（mean/min/max/std）、阈值、分离度，给出 PASS/FAIL
  4. Live：持续打印 reflection、color()、hsv()、BLACK/WHITE 判定 + 滑动窗口噪声
     手推小车慢慢跨过黑白边界，观察判定是否干净地切换。CENTER 结束。

不移动电机。标定结果需要你手动抄进 config.py（本脚本不写存储）。
"""

from umath import sqrt

from pybricks.ev3devices import ColorSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color
from pybricks.tools import StopWatch, run_task, wait

import config as cfg
from common import battery_check, log, wait_for_button, warn_unconfirmed

# 验收门槛 Acceptance thresholds（阶段一用，后续可调）
MIN_SEPARATION = 20     # white_min - black_max 至少 20 个百分点
MAX_STD = 3.0           # 静止时标准差不超过 3%

ev3 = EV3Brick()
sensor = ColorSensor(cfg.COLOR_SENSOR_PORT)
# 阶段一只关心黑/白；限定可选颜色能提高 color() 稳定性
sensor.detectable_colors([Color.BLACK, Color.WHITE, Color.NONE])


def stats(values):
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return mean, min(values), max(values), sqrt(var)


async def sample(n):
    vals = []
    for _ in range(n):
        vals.append(await sensor.reflection())
        await wait(cfg.SAMPLE_PERIOD_MS)
    return vals


async def calibrate(label):
    log(ev3, "Place on", label)
    await wait_for_button(ev3)
    vals = await sample(cfg.CAL_SAMPLES)
    mean, lo, hi, std = stats(vals)
    log(ev3, label, "m={:.1f}".format(mean))
    log(ev3, " min={} max={}".format(lo, hi))
    log(ev3, " std={:.2f}".format(std))
    print(label, "raw:", vals)
    return mean, lo, hi, std


async def live(threshold):
    log(ev3, "LIVE (CENTER=quit)")
    window = []
    watch = StopWatch()
    last_print = 0
    while True:
        r = await sensor.reflection()
        window.append(r)
        if len(window) > cfg.STATS_WINDOW:
            window.pop(0)

        # 屏幕刷新慢，每 500 ms 汇总一次；终端每个样本都打印，方便导出
        c = await sensor.color()
        h = await sensor.hsv()
        surface = "WHITE" if r > threshold else "BLACK"
        print("t={} r={} color={} hsv=({},{},{}) -> {}".format(
            watch.time(), r, c, h.h, h.s, h.v, surface))

        if watch.time() - last_print >= 500 and len(window) >= 2:
            _, lo, hi, std = stats(window)
            ev3.screen.clear()
            ev3.screen.print("r =", r)
            ev3.screen.print(surface)
            ev3.screen.print("color", c)
            ev3.screen.print("win {}-{}".format(lo, hi))
            ev3.screen.print("std {:.2f}".format(std))
            ev3.light.on(Color.GREEN if surface == "WHITE" else Color.RED)
            last_print = watch.time()

        await wait(cfg.SAMPLE_PERIOD_MS)


async def main():
    ev3.screen.clear()
    battery_check(ev3, cfg.LOW_BATTERY_MV)
    warn_unconfirmed(ev3, cfg)

    b_mean, b_lo, b_hi, b_std = await calibrate("BLACK")
    w_mean, w_lo, w_hi, w_std = await calibrate("WHITE")

    threshold = (b_mean + w_mean) / 2
    separation = w_lo - b_hi
    ok = separation >= MIN_SEPARATION and b_std <= MAX_STD and w_std <= MAX_STD

    ev3.screen.clear()
    log(ev3, "thr={:.1f}".format(threshold))
    log(ev3, "sep={}".format(separation))
    log(ev3, "PASS" if ok else "FAIL")
    print("Copy into config.py: BLACK_REFLECTION = {:.0f}, WHITE_REFLECTION = {:.0f}"
          .format(b_mean, w_mean))
    if not ok:
        print("Hints: adjust sensor height (try 5-10 mm), shield ambient light, check port.")

    await wait_for_button(ev3, prompt="RIGHT = live")
    await live(threshold)


run_task(main())
