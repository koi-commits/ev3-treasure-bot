r"""
probe_live.py — 不驱动电机：把小车**用手推**过真实场地的边界，记录反射率。

    pbrun.cmd src\probe_live.py

跑 30 秒。开始会响 3 声（3-2-1），屏幕实时显示当前反射率。
期间请用手把小车从场地内部慢慢推到白色边界上，再推回场地内部，来回 2-3 次。
结束后给出 low20/high20 与建议阈值。
"""

import config
from hardware import color, ev3

from pybricks.tools import run_task, wait

DURATION_MS = 30000
PRINT_MS = 500
SAMPLE_MS = 100


async def main():
    ev3.screen.clear()
    print("=== probe_live: PUSH the robot by hand across the boundary ===")
    print("duration", DURATION_MS, "ms")

    # 3-2-1 倒计时，用户好知道什么时候开始推
    for n in (3, 2, 1):
        ev3.screen.clear()
        ev3.screen.print(str(n))
        ev3.speaker.beep(600, 120)
        await wait(1000)
    ev3.speaker.beep(1200, 400)

    vals = []
    t = 0
    next_print = 0
    while t < DURATION_MS:
        r = await color.reflection()
        vals.append(r)
        ev3.screen.clear()
        ev3.screen.print("refl {:.1f}".format(r))
        ev3.screen.print("t {:.1f}s".format(t / 1000.0))
        if t >= next_print:
            rgb = await color.rgb()
            print("{:6.1f}s  refl={:6.1f}  rgb={}".format(t / 1000.0, r, rgb))
            next_print += PRINT_MS
        await wait(SAMPLE_MS)
        t += SAMPLE_MS

    lo = min(vals)
    hi = max(vals)
    avg = sum(vals) / len(vals)
    n = len(vals)

    s = sorted(vals)
    k = max(1, n // 5)
    low_mean = sum(s[:k]) / k
    high_mean = sum(s[-k:]) / k

    print("--- stats ---")
    print("n={} min={:.1f} max={:.1f} mean={:.1f}".format(n, lo, hi, avg))
    print("low20={:.1f}  high20={:.1f}  span={:.1f}".format(
        low_mean, high_mean, high_mean - low_mean))
    if hi < 2.5:
        print("VERDICT: 全程都很暗 —— 这次也没到白边（或白边反射率极低）")
    elif high_mean > low_mean * 2.0:
        print("suggest BLACK={:.1f} WHITE={:.1f} threshold={:.1f}".format(
            low_mean, high_mean, (low_mean + high_mean) / 2))
    else:
        print("VERDICT: 对比不足，黑白区分不明显")
    print("current config: BLACK={} WHITE={} threshold={}".format(
        config.BLACK_REFLECTION, config.WHITE_REFLECTION, config.edge_threshold()))

    ev3.speaker.beep(800, 300)
    print("done")


run_task(main())
