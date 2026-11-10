r"""
probe_timing.py — 颜色传感器模式切换代价 & 读数是否"卡住"。不驱动电机。

    pbrun.cmd src\probe_timing.py

目的：11-03 的 color_probe 每轮连调 hsv()+rgb()+color()，20 s 读数纹丝不动。
本程序区分两种假设：
  H1 模式来回切换导致读不到新值 / 很慢
  H2 传感器本身（高度/朝向）看不到被测物体

操作：每个阶段开始会响一声。**整个阶段里用手把小车在黑地面和白边之间慢慢来回推**
（或拿白纸在光斑下来回移动），让读数理应不断变化。

阶段（每段 6 s）：
  A reflection 单模式            B rgb 单模式
  C reflection+rgb 交替          D hsv+rgb+color 交替（旧 color_probe 写法）
  E reflection + wait(50)       —— 实测巡线循环的真实周期

每段打印：循环次数、每轮平均 ms、读数范围、不同取值个数、最长"连续相同值"长度。
判读：A/B 有变化而 D 不变 -> H1 成立，必须单模式读；全部不变 -> H2，检查传感器高度。
"""

from common import log
from hardware import color, ev3

from pybricks.tools import StopWatch, run_task, wait

PHASE_MS = 6000

# A/B/C/D 四个阶段之间没有 wait，循环会跑到传感器读取的极限（每秒几百次）。
# 原版把所有样本都 append 进列表，6 秒就把 MicroPython 的堆吃掉：
#   MemoryError（2026-11-10 实机，phase A）
# 所以只保留前 MAX_KEEP 个样本做 distinct 统计，min/max/最长连续相同值
# 用累加字段算，覆盖全部样本，不受裁剪影响。
MAX_KEEP = 400


def bright(rgb):
    return (rgb[0] + rgb[1] + rgb[2]) / 3.0


class Track:
    def __init__(self, name):
        self.name = name
        self.vals = []
        self.n = 0
        self.lo = None
        self.hi = None
        self.run = 0
        self.max_run = 0
        self.last = None

    def add(self, v):
        self.n += 1
        if self.lo is None or v < self.lo:
            self.lo = v
        if self.hi is None or v > self.hi:
            self.hi = v
        if v == self.last:
            self.run += 1
        else:
            self.run = 1
            self.last = v
        if self.run > self.max_run:
            self.max_run = self.run
        if len(self.vals) < MAX_KEEP:
            self.vals.append(v)

    def report(self):
        if not self.n:
            return "  {}: none".format(self.name)
        distinct = len(set(self.vals))
        return "  {:10s} n={} min={} max={} distinct={} longest_same={}".format(
            self.name, self.n, self.lo, self.hi, distinct, self.max_run)


async def phase(tag, reads, extra_wait=0):
    ev3.speaker.beep(1000, 150)
    log(ev3, "phase", tag)
    tracks = {}
    sw = StopWatch()
    loops = 0
    while sw.time() < PHASE_MS:
        for kind in reads:
            if kind == "reflection":
                v = await color.reflection()
            elif kind == "rgb":
                v = bright(await color.rgb())
            elif kind == "hsv":
                v = (await color.hsv()).v
            else:
                v = str(await color.color())
            if kind not in tracks:
                tracks[kind] = Track(kind)
            tracks[kind].add(v)
        if extra_wait:
            await wait(extra_wait)
        loops += 1
    ms = sw.time() / loops if loops else 0
    print("PHASE {} reads={} loops={} ms_per_loop={:.1f}".format(tag, reads, loops, ms))
    for k in reads:
        print(tracks[k].report())
    await wait(800)


async def main():
    ev3.screen.clear()
    log(ev3, "probe_timing")
    log(ev3, "keep sliding over")
    log(ev3, "black <-> white")
    await wait(3000)
    await phase("A", ("reflection",))
    await phase("B", ("rgb",))
    await phase("C", ("reflection", "rgb"))
    await phase("D", ("hsv", "rgb", "color"))
    await phase("E", ("reflection",), extra_wait=50)
    ev3.speaker.beep(500, 400)
    log(ev3, "done")


run_task(main())
