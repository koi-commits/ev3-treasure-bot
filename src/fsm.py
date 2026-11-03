r"""
fsm.py — 纯逻辑模块：不 import 任何 pybricks 设备，CPython 与 MicroPython 都能跑。

这里放所有"可以离线测试"的决策逻辑：
  * EdgeFollower    已实机验证的 CRUISE / ESCAPE / CLEAR 三状态巡线（逐行等价移植）
  * BoxClassifier   基于 RGB + 色度的多特征盒式分类器（地面 / 白边 / 色标）
  * Debounce        连续 N 次相同标签才确认（"颜色序列判据"）
  * LoadMonitor     电机负载在线基线 + 抬升检测（推到物体的证据，待实机验证）
  * Odometry        由 DriveBase distance/angle 增量积分出 x, y, heading

时间单位全部是 ms，距离 mm，角度 deg。DriveBase 约定：正角度 = 右转（俯视顺时针）。
"""

try:
    from umath import cos, sin, sqrt, atan2, pi
except ImportError:  # CPython 离线测试
    from math import cos, sin, sqrt, atan2, pi


# ===================================================================
# 1. 三状态巡线 —— 与 edge.follow_edge_bangbang 原实现逐 tick 等价
#    （tests/test_fsm.py::test_follower_equivalence 用原代码对拍）
# ===================================================================
CRUISE = 0
ESCAPE = 1
CLEAR = 2
STATE_NAMES = ("CRUISE", "ESCAPE", "CLEAR")


class EdgeFollower:
    """step(t, white) -> (speed_mm_s, turn_rate_deg_s)

    t     : 计数时钟（每 tick 加 SAMPLE_PERIOD_MS，与实测版本一致）
    white : 本 tick 是否判定为白边。注意：色标样本应传 False（色标不是边界）。

    side  : +1 = 白边在右侧，-1 = 白边在左侧（实测用 -1）
    """

    def __init__(self, side, speed, approach_steer, turn_rate,
                 escape_ms, clear_ms, max_extensions=2):
        self.speed = speed
        self.away = -side * turn_rate
        self.toward = side * approach_steer
        self.escape_ms = escape_ms
        self.clear_ms = clear_ms
        self.max_ext = max_extensions
        self.reset(0)

    def reset(self, t):
        self.state = CRUISE
        self.until = 0
        self.ext = 0
        self.twists = 0
        self.last_white_t = t     # 监督用：最近一次见白的时间
        self.exhausted = 0        # 监督用：续转用完仍在白区的次数（越界信号）

    def step(self, t, white):
        if white:
            self.last_white_t = t

        if self.state == CRUISE:
            if white:
                self.twists += 1
                self.state = ESCAPE
                self.until = t + self.escape_ms
                self.ext = 0
                return 0, self.away
            return self.speed, self.toward

        if self.state == ESCAPE:
            if t >= self.until:
                if white and self.ext < self.max_ext:
                    self.ext += 1
                    self.until = t + self.escape_ms
                else:
                    if white:
                        self.exhausted += 1
                    self.state = CLEAR
                    self.until = t + self.clear_ms
            return 0, self.away

        # CLEAR：盲走，忽略传感器
        if t >= self.until:
            self.state = CRUISE
        return self.speed, self.toward


# ===================================================================
# 2. 颜色特征与盒式分类器
# ===================================================================
def features(rgb):
    """rgb (0–100 %) -> (r, g, b, bright, cr, cg)

    bright = 三通道均值；cr/cg = 色度（通道占比 ×100），对传感器高度/亮度变化不敏感。
    暗处色度噪声大，所以标定时会给出较宽容差；靠 bright 区分黑地面。
    """
    r, g, b = rgb[0], rgb[1], rgb[2]
    s = r + g + b
    if s <= 0:
        return (r, g, b, 0.0, 33.3, 33.3)
    return (r, g, b, s / 3.0, 100.0 * r / s, 100.0 * g / s)


FEATURE_NAMES = ("r", "g", "b", "bright", "cr", "cg")


class Signature:
    """一个类别 = 每个特征一个 [mean - tol, mean + tol] 区间。tol 为 None 表示不检查该维。"""

    def __init__(self, name, mean, tol):
        self.name = name
        self.mean = tuple(mean)
        self.tol = tuple(tol)

    def distance(self, f):
        """归一化 Chebyshev 距离；<=1 表示落在盒内。"""
        worst = 0.0
        for i in range(len(self.mean)):
            tol = self.tol[i]
            if tol is None:
                continue
            d = abs(f[i] - self.mean[i]) / tol
            if d > worst:
                worst = d
        return worst


def signature_from_dict(name, d):
    return Signature(name, d["mean"], d["tol"])


class BoxClassifier:
    def __init__(self, signatures):
        self.sigs = list(signatures)

    def classify(self, rgb):
        """返回 (label 或 None, 距离)。多个类别同时命中时取距离最小的。"""
        f = features(rgb)
        best = None
        best_d = 1e9
        for s in self.sigs:
            d = s.distance(f)
            if d <= 1.0 and d < best_d:
                best = s.name
                best_d = d
        return best, best_d


def overlaps(a, b):
    """两个盒子在所有被检查的维度上都相交 -> 存在混淆可能。"""
    for i in range(len(a.mean)):
        ta, tb = a.tol[i], b.tol[i]
        if ta is None or tb is None:
            continue
        if a.mean[i] + ta < b.mean[i] - tb or b.mean[i] + tb < a.mean[i] - ta:
            return False
    return True


def build_signature(name, samples, k=4.0, floors=(2.0, 2.0, 2.0, 1.5, 4.0, 4.0)):
    """由一组 rgb 样本生成签名：tol = max(k*std, 半极差 + 1, floor)。"""
    fs = [features(s) for s in samples]
    n = len(fs)
    mean = []
    tol = []
    for i in range(len(FEATURE_NAMES)):
        vals = [f[i] for f in fs]
        m = sum(vals) / n
        sd = sqrt(sum((v - m) ** 2 for v in vals) / n)
        half = (max(vals) - min(vals)) / 2 + 1.0
        t = k * sd
        if half > t:
            t = half
        if floors[i] > t:
            t = floors[i]
        mean.append(m)
        tol.append(t)
    return Signature(name, mean, tol)


class Debounce:
    """连续 n 次同一标签才输出确认。用于"色标必须稳定出现若干个样本"。"""

    def __init__(self, n):
        self.n = n
        self.label = None
        self.count = 0

    def push(self, label):
        if label == self.label:
            self.count += 1
        else:
            self.label = label
            self.count = 1
        return self.label if (self.label is not None and self.count >= self.n) else None

    def reset(self):
        self.label = None
        self.count = 0


# ===================================================================
# 3. 负载监测：判断 V 形推板里是否推着物体（待实机验证信号强度）
# ===================================================================
class LoadMonitor:
    """在线学习空载基线，之后检测持续抬升。

    用法：每段直行开始时 restart()，每 tick push(load_sum)。
      每段：先跳过 skip_n 个加速段样本，再用 learn_n 个样本算本段均值；
      基线 = 历来各段均值的**最小值**（空推板一定是最小负载），
      所以即使第一段一开始就推着物体，后面空载的段也会把基线拉回正确值。
      之后滑动窗口均值 > 基线均值 + max(k*std, abs_margin) 连续 hold_n 次 -> True
    """

    def __init__(self, skip_n=6, learn_n=12, window=5, k=4.0, abs_margin=15.0, hold_n=4):
        self.skip_n = skip_n
        self.learn_n = learn_n
        self.window = window
        self.k = k
        self.abs_margin = abs_margin
        self.hold_n = hold_n
        self.mean = None
        self.std = None
        self.restart()

    def reset(self):
        """完全重置（包括基线）。"""
        self.mean = None
        self.std = None
        self.restart()

    def restart(self):
        """新的一段直行：重新跳过加速段、重新测本段均值；基线只会往下更新。"""
        self.n = 0
        self.leg = []
        self.win = []
        self.over = 0
        self.level = 0.0

    def ready(self):
        return self.mean is not None

    def push(self, x):
        self.n += 1
        if self.n <= self.skip_n:
            return False
        if len(self.leg) < self.learn_n:
            self.leg.append(x)
            if len(self.leg) == self.learn_n:
                m = sum(self.leg) / self.learn_n
                if self.mean is None or m < self.mean:
                    self.mean = m
                    self.std = sqrt(sum((v - m) ** 2 for v in self.leg) / self.learn_n)
            return False
        self.win.append(x)
        if len(self.win) > self.window:
            self.win.pop(0)
        self.level = sum(self.win) / len(self.win)
        margin = self.k * self.std
        if self.abs_margin > margin:
            margin = self.abs_margin
        if len(self.win) == self.window and self.level > self.mean + margin:
            self.over += 1
        else:
            self.over = 0
        return self.over >= self.hold_n


# ===================================================================
# 4. 里程计（DriveBase 不提供 x/y，只给累计 distance 与 angle）
# ===================================================================
class Odometry:
    """heading 与 DriveBase 一致：deg，正 = 右转（顺时针）。
    x 轴 = 启动时车头方向，y 轴 = 启动时车左侧。"""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self._d = None

    def update(self, distance, angle):
        if self._d is None:
            self._d = distance
            self.heading = angle
            return
        dd = distance - self._d
        self._d = distance
        mid = (self.heading + angle) / 2.0
        rad = -mid * pi / 180.0          # DriveBase 正角度为顺时针
        self.x += dd * cos(rad)
        self.y += dd * sin(rad)
        self.heading = angle

    def bearing_to(self, x, y):
        """返回到 (x, y) 需要转的相对角度（DriveBase 约定，-180..180）和距离。"""
        dx = x - self.x
        dy = y - self.y
        target = -atan2(dy, dx) * 180.0 / pi
        rel = wrap180(target - self.heading)
        return rel, sqrt(dx * dx + dy * dy)


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def heading_of(dx, dy):
    """odometry 平面向量 -> DriveBase 航向（deg，顺时针为正）。"""
    return -atan2(dy, dx) * 180.0 / pi


def straight_run_heading(pts, max_dev_deg=12.0, min_span=120.0):
    """沿边锯齿的逃逸点 -> 当前这条边的方向（DriveBase 航向）。
    从最新点往回找，只保留与"最新一段"方向偏差 < max_dev_deg 的点（跨过拐角的旧点会被丢掉）。
    跨度不足 min_span 返回 None。"""
    n = len(pts)
    if n < 2:
        return None
    xn, yn = pts[-1]
    x1, y1 = pts[-2]
    ref = heading_of(xn - x1, yn - y1)
    best = None
    for k in range(n - 2, -1, -1):
        dx = xn - pts[k][0]
        dy = yn - pts[k][1]
        if abs(wrap180(heading_of(dx, dy) - ref)) > max_dev_deg:
            break
        if dx * dx + dy * dy >= min_span * min_span:
            best = heading_of(dx, dy)
    return best

