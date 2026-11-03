"""在真正的 MicroPython 下执行纯逻辑模块（fsm / calib）的关键检查。

    MICROPYPATH=src:tests micropython tests/mpy_logic_check.py

CPython 的 unittest 覆盖更全；这里只证明这些模块在 MicroPython 的
解释器/格式化/数学库下行为一致（EV3 上跑的是 MicroPython）。
"""
import random

from calib import analyze
from fsm import (BoxClassifier, Debounce, EdgeFollower, LoadMonitor, Odometry,
                 build_signature, features)
from reference_20261103 import follow_edge_bangbang_reference

random.seed(1)


def noise(sd):
    # MicroPython 的 random 没有 gauss：用 4 个均匀分布近似
    return (sum(random.random() for _ in range(4)) - 2.0) * sd * 1.7


def noisy(base, n=40, sd=0.6):
    out = []
    for _ in range(n):
        out.append(tuple(max(0, int(round(c + noise(sd)))) for c in base))
    return out


P = dict(threshold=6.35, speed=60, side=-1, approach_steer=22, turn_rate=130,
         escape_ms=350, clear_ms=400, period=50)

# 1. follower 与 11-03 实测版逐 tick 等价
for trial in range(200):
    seq = []
    n = random.randint(1, 300)
    while len(seq) < n:
        white = random.random() < 0.35
        v = random.choice((10.0, 12.0, 14.5)) if white else random.choice((0.3, 0.7, 2.0))
        seq += [v] * random.randint(1, 30)
    seq = seq[:n]
    f = EdgeFollower(-1, 60, 22, 130, 350, 400)
    got = []
    t = 0
    for r in seq:
        got.append(f.step(t, r > 6.35))
        t += 50
    ref, tw = follow_edge_bangbang_reference(seq, P["threshold"], P["speed"], P["side"],
                                            P["approach_steer"], P["turn_rate"],
                                            P["escape_ms"], P["clear_ms"], P["period"])
    assert got == ref, trial
    assert f.twists == tw
print("follower equivalence: OK (200 sequences)")

# 2. 分类器 + 混合光斑
fl = build_signature("floor", noisy((5, 5, 5)))
wh = build_signature("white", noisy((24, 23, 22)))
fi = build_signature("finish", noisy((20, 7, 6)))
clf = BoxClassifier([fl, wh, fi])
assert clf.classify((5, 5, 5))[0] == "floor"
assert clf.classify((24, 23, 22))[0] == "white"
assert clf.classify((20, 7, 6))[0] == "finish"
for i in range(21):
    k = i / 20.0
    rgb = (5 + 19 * k, 5 + 18 * k, 5 + 17 * k)
    assert clf.classify(rgb)[0] != "finish"
d = Debounce(3)
assert [d.push(x) for x in ("finish", "finish", "finish")] == [None, None, "finish"]
print("classifier: OK", features((20, 10, 10)))

# 3. 标定分析 + 生成的配置文本可被 MicroPython 执行
rep = analyze({"floor": noisy((5, 5, 5)), "white": noisy((24, 23, 22)), "finish": noisy((20, 7, 6))},
              {"finish": [8.0]}, 6.35)
assert rep["ok"], rep["problems"]
ns = {}
exec("\n".join(rep["lines"]), ns)
assert "finish" in ns["SIGNATURES"]
bad = analyze({"floor": noisy((5, 5, 5)), "white": noisy((24, 23, 22)), "finish": noisy((14, 14, 13))})
assert not bad["ok"]
print("calib: OK", rep["lines"][1])

# 4. 负载监测
m = LoadMonitor()
hits = [m.push(25 + noise(2) + (40 if i > 60 else 0)) for i in range(120)]
first = hits.index(True)
assert 60 < first < 75, first
m.restart()
assert m.ready()
print("load monitor: OK trigger at", first)

# 5. 里程计
o = Odometry()
o.update(0, 0)
o.update(500, 0)
o.update(500, 90)
o.update(800, 90)
assert abs(o.x - 500) < 1 and abs(o.y + 300) < 1
rel, dist = o.bearing_to(0, 0)
assert abs(rel - 120.96) < 1
print("odometry: OK")
print("ALL MICROPYTHON CHECKS PASSED")
