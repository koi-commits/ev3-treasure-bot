"""
sim.py — 轻量 2D 场地仿真，驱动 tests/mock/pybricks。

这是**逻辑验证工具**，不是物理仿真：
  * 差速车运动学 + 加速度限幅（模拟"见白时车还在往前冲"）
  * 颜色传感器光斑 = 车头前 sensor_offset 处一个小圆的平均值 + 高斯噪声
  * 物体被 V 形推板"捕获"后随车移动，并给电机负载加一个常量
  * 里程计误差：轮径/轮距设错时，DriveBase 报告的距离/角度与真实运动按比例不同

坐标：mm，场地内部 [0, W] x [0, H]；theta 为数学约定（逆时针，弧度）。
DriveBase 约定：turn_rate / angle 正 = 顺时针（右转）。
"""

import math
import random

try:
    _Random = random.Random
except AttributeError:      # MicroPython：没有 Random 类，用模块级函数做一个够用的替身
    class _Random:
        def __init__(self, seed=0):
            random.seed(seed)

        def random(self):
            return random.random()

        def uniform(self, a, b):
            return a + (b - a) * random.random()

        def choice(self, seq):
            return seq[int(random.random() * len(seq))]

        def gauss(self, mu, sd):
            return mu + (sum(random.random() for _ in range(4)) - 2.0) * sd * 1.7


class _Holder:
    sim = None


SIM_HOLDER = _Holder()


class Done:
    """立即完成的 awaitable。"""

    def __await__(self):
        return None
        yield  # noqa: unreachable — makes this a generator

    __iter__ = __await__   # MicroPython 的 await 走迭代器协议


class Value:
    """读传感器：可选读数耗时（模拟模式切换代价）。"""

    def __init__(self, fn):
        self.fn = fn

    def __await__(self):
        cost = SIM_HOLDER.sim.read_cost_ms
        if cost:
            yield ("sleep", cost)
        return self.fn()

    __iter__ = __await__   # MicroPython 的 await 走迭代器协议


class Until:
    def __init__(self, pred):
        self.pred = pred

    def __await__(self):
        while not self.pred():
            yield ("sleep", 5)

    __iter__ = __await__   # MicroPython 的 await 走迭代器协议


# -------------------------------------------------------------------
# 表面
# -------------------------------------------------------------------
FLOOR = {"refl": 0.7, "rgb": (5, 5, 5)}
WHITE = {"refl": 12.0, "rgb": (24, 23, 22)}
RED_MARK = {"refl": 8.0, "rgb": (20, 7, 6)}       # 故意让反射率 > 边界阈值 6.35
BLUE_MARK = {"refl": 4.0, "rgb": (5, 8, 16)}


class SimTimeout(Exception):
    pass


class Sim:
    def __init__(self, W=1200, H=900, band=40, outside="floor",
                 start=(600, 450, 0.0), sensor_offset=60.0, spot_r=4.0,
                 noise=0.15, markers=(), objects=(), obstacles=(),
                 accel=750.0, turn_accel=750.0, odo_dist_scale=1.0, odo_turn_scale=1.0,
                 read_cost_ms=1, buttons=(), seed=0, max_ms=400000):
        self.W, self.H, self.band = W, H, band
        self.outside = FLOOR if outside == "floor" else WHITE
        self.x, self.y, self.th = start
        self.sensor_offset = sensor_offset
        self.spot_r = spot_r
        self.noise = noise
        self.markers = list(markers)        # (x0, y0, x1, y1, surface)
        self.objects = [dict(o) for o in objects]   # {"x","y","r","load","carried"}
        self.obstacles = list(obstacles)    # (x0, y0, x1, y1)
        self.accel, self.turn_accel = accel, turn_accel
        self.ds, self.ts = odo_dist_scale, odo_turn_scale
        self.read_cost_ms = read_cost_ms
        self.button_events = list(buttons)  # (t_from, t_to, Button)
        self.rng = _Random(seed)
        self.max_ms = max_ms

        self.t = 0.0
        self.v = 0.0        # 实际线速度 mm/s
        self.w = 0.0        # 实际角速度 deg/s（顺时针为正）
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.man = None     # 定距/定角动作
        self.dist_acc = 0.0
        self.ang_acc = 0.0
        self.stalled = False
        self.block_ms = 0.0
        self.light = None
        self.beeps = []
        self.screen = []
        self.sensor_trace = []      # (t, sx, sy, surface_name)
        self.max_out = 0.0          # 传感器越出场地内部的最大距离（mm）
        self.max_out_center = 0.0   # 车体中心越出的最大距离
        self.reads = 0
        SIM_HOLDER.sim = self

    # ---------------------------------------------------------- surface
    def surface_at(self, px, py):
        for (x0, y0, x1, y1, surf) in self.markers:
            if x0 <= px <= x1 and y0 <= py <= y1:
                return surf
        inside = 0 <= px <= self.W and 0 <= py <= self.H
        if inside:
            return FLOOR
        b = self.band
        if -b <= px <= self.W + b and -b <= py <= self.H + b:
            return WHITE
        return self.outside

    def sensor_pos(self):
        return (self.x + self.sensor_offset * math.cos(self.th),
                self.y + self.sensor_offset * math.sin(self.th))

    def _spot(self):
        sx, sy = self.sensor_pos()
        r = self.spot_r
        pts = ((0, 0), (r, 0), (-r, 0), (0, r), (0, -r))
        refl = 0.0
        rgb = [0.0, 0.0, 0.0]
        for dx, dy in pts:
            s = self.surface_at(sx + dx, sy + dy)
            refl += s["refl"]
            for i in range(3):
                rgb[i] += s["rgb"][i]
        n = len(pts)
        return refl / n, [c / n for c in rgb]

    def read_sensor(self, kind):
        self.reads += 1
        refl, rgb = self._spot()
        g = self.rng.gauss
        if kind == "reflection":
            return max(0.0, round(refl + g(0, self.noise), 1))
        if kind == "rgb":
            return tuple(max(0, int(round(c + g(0, self.noise * 3)))) for c in rgb)
        from pybricks.parameters import Color
        if kind == "hsv":
            return Color(h=0, s=10.0, v=sum(rgb) / 3.0)
        if kind == "color":
            return Color.NONE
        raise NotImplementedError(kind)

    # ---------------------------------------------------------- commands
    def cmd_drive(self, v, w):
        self.man = None
        self.cmd_v, self.cmd_w = float(v), float(w)

    def cmd_stop(self):
        self.man = None
        self.cmd_v = self.cmd_w = 0.0
        self.v = self.w = 0.0       # DriveBase.stop 为 coast；这里简化为立即停

    def cmd_straight(self, mm, speed):
        self.man = {"kind": "d", "start": self.dist_acc, "target": mm, "speed": abs(speed)}

    def cmd_turn(self, deg, rate):
        self.man = {"kind": "a", "start": self.ang_acc, "target": deg, "speed": abs(rate)}

    def maneuver_done(self):
        return self.man is None

    def cmd_reset(self, d=0, a=0):
        self.cmd_stop()
        self.dist_acc, self.ang_acc = d / self.ds, a / self.ts

    def odo_distance(self):
        return int(self.dist_acc * self.ds)

    def odo_angle(self):
        return self.ang_acc * self.ts

    def motor_angle(self, side):
        return 0

    def motor_load(self, side):
        base = 20.0 + abs(self.v) * 0.05
        extra = sum(o["load"] for o in self.objects if o.get("carried")) / 2.0
        if self.stalled:
            extra += 150
        return base + extra + self.rng.gauss(0, 2.0)

    def buttons_pressed(self):
        """button_events: (t0, t1, Button) 或 (t0, t1, Button, (x, y, th))。
        带位姿的事件在按下时把车"搬"到该位置（模拟操作员把车放到某个表面上）。"""
        out = set()
        for ev in self.button_events:
            t0, t1, b = ev[0], ev[1], ev[2]
            if t0 <= self.t <= t1:
                out.add(b)
                if len(ev) > 3 and not getattr(self, "_moved_" + str(id(ev)), False):
                    self.x, self.y, self.th = ev[3]
                    setattr(self, "_moved_" + str(id(ev)), True)
        return out

    # ---------------------------------------------------------- physics
    def _targets(self):
        if self.man is None:
            return self.cmd_v, self.cmd_w
        m = self.man
        if m["kind"] == "d":
            done = abs(self.dist_acc - m["start"])
            rem = abs(m["target"]) - done
            if rem <= 0.5:
                self.man = None
                self.v = 0.0
                return 0.0, 0.0
            sp = min(m["speed"], math.sqrt(2 * self.accel * rem) + 5)
            return math.copysign(sp, m["target"]), 0.0
        done = abs(self.ang_acc - m["start"])
        rem = abs(m["target"]) - done
        if rem <= 0.3:
            self.man = None
            self.w = 0.0
            return 0.0, 0.0
        sp = min(m["speed"], math.sqrt(2 * self.turn_accel * rem) + 3)
        return 0.0, math.copysign(sp, m["target"])

    def _blocked(self, nx, ny, nth):
        fx = nx + 90 * math.cos(nth)
        fy = ny + 90 * math.sin(nth)
        for (x0, y0, x1, y1) in self.obstacles:
            if x0 <= fx <= x1 and y0 <= fy <= y1:
                return True
        for o in self.objects:
            if o.get("carried") and o.get("immovable"):
                return True
        return False

    def step(self, dt_ms):
        dt = dt_ms / 1000.0
        tv, tw = self._targets()
        dv = self.accel * dt
        self.v += max(-dv, min(dv, tv - self.v))
        dw = self.turn_accel * dt
        self.w += max(-dw, min(dw, tw - self.w))

        nth = self.th - math.radians(self.w) * dt
        nx = self.x + self.v * dt * math.cos(nth)
        ny = self.y + self.v * dt * math.sin(nth)
        if self.v > 0 and self._blocked(nx, ny, nth):
            self.block_ms += dt_ms
            nx, ny = self.x, self.y
            moved = 0.0
        else:
            self.block_ms = 0.0
            moved = self.v * dt
        self.stalled = self.block_ms > 200
        self.x, self.y, self.th = nx, ny, nth
        self.dist_acc += moved
        self.ang_acc += self.w * dt
        self._objects(moved)

        sx, sy = self.sensor_pos()
        self.max_out = max(self.max_out, self._outside_dist(sx, sy))
        self.max_out_center = max(self.max_out_center, self._outside_dist(self.x, self.y))
        self.t += dt_ms
        if self.t > self.max_ms:
            raise SimTimeout()

    def _outside_dist(self, px, py):
        dx = max(-px, 0, px - self.W)
        dy = max(-py, 0, py - self.H)
        return max(dx, dy)

    PLOW_D = 95.0       # 推板顶点到车体中心
    PLOW_HALF = 70.0    # 推板开口半宽

    def _objects(self, moved):
        c, s = math.cos(self.th), math.sin(self.th)
        for o in self.objects:
            if o.get("carried"):
                if self.v < -5:
                    o["carried"] = False           # 倒车：物体留在原地
                    continue
                o["x"] = self.x + (self.PLOW_D - o["r"]) * c
                o["y"] = self.y + (self.PLOW_D - o["r"]) * s
                continue
            dx, dy = o["x"] - self.x, o["y"] - self.y
            f = dx * c + dy * s
            l = -dx * s + dy * c
            if moved > 0 and 0 < f < self.PLOW_D + o["r"] and abs(l) < self.PLOW_HALF:
                o["carried"] = True
                o["captured_t"] = self.t

    def advance(self, ms):
        n = max(1, int(round(ms / 5.0)))
        for _ in range(n):
            self.step(ms / n)

    # ---------------------------------------------------------- scheduler
    def run(self, coro):
        try:
            while True:
                req = coro.send(None)
                if req is None:
                    self.advance(5)
                elif req[0] == "sleep":
                    self.advance(req[1])
                else:
                    raise RuntimeError(req)
        except StopIteration:
            pass
        finally:
            coro.close()


def rect_arena_marker(W, H, where="right", length=150, depth=60, surf=RED_MARK):
    """在场地内侧贴边放一块色标（贴着白边，伸进场地 depth mm）。"""
    if where == "right":
        y0 = H / 2 - length / 2
        return (W - depth, y0, W, y0 + length, surf)
    if where == "top":
        x0 = W / 2 - length / 2
        return (x0, H - depth, x0 + length, H, surf)
    if where == "left":
        y0 = H / 2 - length / 2
        return (0, y0, depth, y0 + length, surf)
    x0 = W / 2 - length / 2
    return (x0, 0, x0 + length, depth, surf)
