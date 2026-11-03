r"""
demo_edge.py — 最小可跑：自动寻边 + 沿边巡线（带遥测）。

    pbrun.cmd src\demo_edge.py

行为：
  1. 慢速直行，直到颜色传感器连续看到白色（= 走到场地边界）
  2. 后退一点，让传感器从"全白"退到"边界"附近
  3. 沿边巡线 N 秒（比例控制，白线保持在传感器一侧）
  4. 共 3 段，段间转向

遥测：巡线过程中每 500 ms 打一行 `t refl steer`，段末打印
  white_frac = 读到白色的样本比例（理想 ~0.5，说明正压在边界上；
                            ~1.0 说明已经整个跑到白色区域里去了；
                            ~0.0 说明完全退回深色场地，没在巡线）
  refl_min / refl_max / refl_mean

安全：看到白色就后退转向；但请先把小车放在场地**内部**、离边至少 20 cm。
"""

import config
import edge
from hardware import drive, ev3

from pybricks.parameters import Button, Color
from pybricks.tools import run_task, wait

CRUISE_SPEED = 90        # mm/s 找边时的前进速度
SEARCH_MS = 15000        # 找边超时
FOLLOW_MS = 8000         # 每段沿边时长
SEGMENTS = 3
TRACE_MS = 500           # 遥测打印周期


class Tele:
    """巡线过程中的统计。"""

    def __init__(self, threshold):
        self.threshold = threshold
        self.n = 0
        self.white = 0
        self.lo = 999.0
        self.hi = -999.0
        self.sum = 0.0
        self.next_print = 0

    def tick(self, t, refl, steer):
        self.n += 1
        self.sum += refl
        if refl > self.threshold:
            self.white += 1
        if refl < self.lo:
            self.lo = refl
        if refl > self.hi:
            self.hi = refl
        if t >= self.next_print:
            print("  t={:5d} refl={:5.1f} steer={:5.0f}".format(t, refl, steer))
            self.next_print = t + TRACE_MS

    def report(self):
        if self.n == 0:
            print("  (no samples)")
            return
        print("  white_frac={:.2f}  refl min={:.1f} max={:.1f} mean={:.1f}  n={}".format(
            self.white / self.n, self.lo, self.hi, self.sum / self.n, self.n))


async def wait_start():
    ev3.screen.clear()
    print("demo_edge: put robot inside the arena, then press RIGHT")
    ev3.screen.print("demo_edge")
    ev3.light.on(Color.ORANGE)
    while Button.RIGHT not in ev3.buttons.pressed():
        await wait(30)
    while Button.RIGHT in ev3.buttons.pressed():
        await wait(30)
    ev3.light.on(Color.GREEN)
    await wait(400)


async def find_edge(threshold):
    """慢速前进直到看到白边。返回 True 表示找到。"""
    drive.reset()
    t = 0
    period = config.SAMPLE_PERIOD_MS
    drive.drive(CRUISE_SPEED, 0)
    while t < SEARCH_MS:
        r = await edge.read_reflection()
        if r > threshold:
            if await edge.hit_edge(threshold):
                drive.stop()
                print("edge found at t=", t, "refl=", r)
                ev3.speaker.beep(900, 120)
                return True
        await wait(period)
        t += period
    drive.stop()
    print("edge NOT found within", SEARCH_MS, "ms")
    return False


async def main():
    await wait_start()

    threshold = config.edge_threshold()
    print("threshold =", threshold)
    if config.black_white_threshold() is None:
        print("(using DEFAULT_EDGE_THRESHOLD, run calibrate.py for real values)")

    for seg in range(SEGMENTS):
        print("--- segment", seg, "---")
        if not await find_edge(threshold):
            drive.turn(60)
            continue

        # 退一点，让传感器从"全白"退到"边界"附近
        drive.straight(-80)
        await wait(200)

        tele = Tele(threshold)
        twists = await edge.follow_edge_bangbang(
            threshold=threshold,
            speed=config.FOLLOW_SPEED,
            side=-1,
            max_ms=FOLLOW_MS,
            on_tick=tele.tick,
        )
        print("distance", int(drive.distance()), "mm   edge_twists", twists)
        tele.report()

        if seg < SEGMENTS - 1:
            drive.turn(30)

    drive.stop()
    ev3.speaker.beep(500, 500)
    print("demo_edge done")


run_task(main())
