r"""
probe_load.py — 判断"推着物体"能否从电机负载里看出来（V 形推板方案的关键前提）。

    pbrun.cmd src\probe_load.py

场地上直行（遇白边自动停，最长 RUN_MM），每 tick 记录 |left.load()| + |right.load()|（mNm）。
  阶段 1 EMPTY  ：推板里没东西，前方 ≥ 1 m 空地。RIGHT 开始。
  阶段 2 HOLD   ：把物体放进推板顶点处，再 RIGHT。
  阶段 3 ONLINE ：物体放在车前方约 30 cm 的路径上（推板正对它），RIGHT。
                  用 mission 同款 LoadMonitor 在线判定，打印触发时行驶的距离。

判读：
  * EMPTY 与 HOLD 的均值差 >> EMPTY 的 std（建议 > 4 倍）-> 负载方案可行，
    把打印的 LOAD_ABS_MARGIN 建议值抄进 config.py
  * 差别不明显 -> 物体太轻；CAPTURE_MODE 改 "off"，或给物体加配重/换更重的目标物（需确认规则）
  * ONLINE 触发距离 ≈ 300 mm（+推板深度）为理想；远早于 300 mm = 误报；不触发 = 漏报
"""

import config
from common import battery_check, log, wait_button_or_timeout
from fsm import LoadMonitor
from hardware import color, drive, ev3, left, right

from pybricks.tools import run_task, wait

RUN_MM = 800
SPEED = config.SWEEP_SPEED


def load_sum():
    return abs(left.load()) + abs(right.load())


def stats(v):
    n = len(v)
    m = sum(v) / n
    sd = (sum((x - m) ** 2 for x in v) / n) ** 0.5
    return m, sd, min(v), max(v)


async def run_leg(tag, monitor=None):
    thr = config.edge_threshold()
    drive.reset()
    vals = []
    trig = None
    drive.drive(SPEED, 0)
    n = 0
    while abs(drive.distance()) < RUN_MM:
        if await color.reflection() > thr:
            log(ev3, "white -> stop")
            break
        x = load_sum()
        n += 1
        if n > config.LOAD_SKIP_N:
            vals.append(x)
        if monitor is not None and trig is None and monitor.push(x):
            trig = drive.distance()
            ev3.speaker.beep(1500, 100)
        await wait(config.SAMPLE_PERIOD_MS)
    drive.stop()
    dist = drive.distance()
    if len(vals) < 5:
        print(tag, "too few samples")
        return None
    m, sd, lo, hi = stats(vals)
    print("{:6s} dist={}mm n={} load mean={:.1f} std={:.1f} min={} max={}".format(
        tag, dist, len(vals), m, sd, lo, hi))
    print("   raw:", vals)
    if monitor is not None:
        print("   LoadMonitor trigger at", trig, "mm (None = never)",
              "baseline", monitor.mean, "std", monitor.std)
    await drive.straight(-min(abs(dist), 300))
    return m, sd


async def main():
    ev3.screen.clear()
    log(ev3, "probe_load")
    battery_check(ev3, config.LOW_BATTERY_MV)
    try:
        load_sum()
    except Exception as e:
        log(ev3, "load() failed", repr(e))
        return

    await wait_button_or_timeout(ev3, 60000, prompt="EMPTY: RIGHT")
    e = await run_leg("EMPTY")
    await wait_button_or_timeout(ev3, 60000, prompt="HOLD obj: RIGHT")
    h = await run_leg("HOLD")
    await wait_button_or_timeout(ev3, 60000, prompt="obj 30cm: RIGHT")
    mon = LoadMonitor(config.LOAD_SKIP_N, config.LOAD_LEARN_N, config.LOAD_WINDOW,
                      config.LOAD_K, config.LOAD_ABS_MARGIN, config.LOAD_HOLD_N)
    await run_leg("ONLINE", mon)

    if e and h:
        diff = h[0] - e[0]
        print("difference HOLD-EMPTY = {:.1f} mNm = {:.1f} x empty std".format(
            diff, diff / e[1] if e[1] > 0 else 999))
        if diff > 4 * e[1] and diff > 6:
            print("SUGGEST: LOAD_ABS_MARGIN = {:.0f}   (CAPTURE_MODE = 'load')".format(diff / 2))
        else:
            print("SUGGEST: load signal too weak -> CAPTURE_MODE = 'off' (see docs/object-strategy.md)")
    ev3.speaker.beep(500, 300)


run_task(main())
