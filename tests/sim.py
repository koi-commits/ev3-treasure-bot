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



def _hypot(a, b):                       # MicroPython 没有 math.hypot
    return math.sqrt(a * a + b * b)

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


# 车体几何（车体坐标：x 向前，y 向左，原点 = 两轮轴线中点）。全部是假设值，实机量出后改这里。
GEOM = dict(
    body_front=45.0, body_back=-80.0, body_half=62.0, wheel_y=57.0,
    plow=True,
    apex=70.0,          # V 形推板顶点（口袋最深处）—— 在光斑(60)前 10 mm
    plow_depth=80.0,    # 顶点到臂尖的前向长度
    plow_half=65.0,     # 臂尖半宽（开口 130 mm，夹角约 78°）
    side_plates=True,   # 车身侧面延长板：封住"臂外侧 + 车头"形成的凹角（否则物体会卡在臂外侧被推着走）
)


class Sim:
    def __init__(self, W=1200, H=900, band=40, outside="floor",
                 start=(600, 450, 0.0), sensor_offset=60.0, spot_r=4.0,
                 noise=0.15, markers=(), objects=(), obstacles=(),
                 accel=750.0, turn_accel=750.0, odo_dist_scale=1.0, odo_turn_scale=1.0,
                 read_cost_ms=1, buttons=(), seed=0, max_ms=400000,
                 geom=None, contact_mu=0.3, load_noise=2.5, coast_decel=900.0, brake_decel=2000.0,
                 legacy_objects=False, sensor_lat=0.0, battery_mv=7480,
                 battery_events=()):
        self.W, self.H, self.band = W, H, band
        self.outside = FLOOR if outside == "floor" else WHITE
        self.x, self.y, self.th = start
        self.sensor_offset = sensor_offset
        self.sensor_lat = sensor_lat        # 光斑横向偏移（mm，正 = 车左侧）
        self.spot_r = spot_r
        self.noise = noise
        self.markers = list(markers)        # (x0, y0, x1, y1, surface)
        # 物体：{"x","y","r","load"(推动时两电机合计负载 mNm), 可选 "immovable"}
        self.objects = [dict(o) for o in objects]
        for o in self.objects:
            o.setdefault("out", False)
            o.setdefault("pushed_ms", 0.0)
        self.obstacles = list(obstacles)    # (x0, y0, x1, y1)
        self.accel, self.turn_accel = accel, turn_accel
        self.ds, self.ts = odo_dist_scale, odo_turn_scale
        self.read_cost_ms = read_cost_ms
        self.button_events = list(buttons)  # (t_from, t_to, Button)
        self.battery_initial_mv = battery_mv
        self.battery_events = sorted(battery_events)  # (t_from_ms, voltage_mv)
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
        g = dict(GEOM)
        g.update(geom or {})
        self.g = g
        self.contact_mu = contact_mu
        self.load_noise = load_noise
        self.coast_decel, self.brake_decel = coast_decel, brake_decel
        self.legacy_objects = legacy_objects
        self.stop_mode = None
        self.acc_v = 0.0
        self.load_drift = [0.0, 0.0]
        self.push_load = 0.0
        self.wheel_max_out = 0.0    # 轮子触地点越过白边内沿的最大距离
        self.object_out = False     # 有物体中心越过白边外沿（推出场地）
        # 可选采样钩子，供学习数据收集使用。默认关闭，既有测试/脚本行为不变。
        # 签名 hook(sim)；采集器不得改写运动/接触状态。
        self.step_hook = None
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
        c, s = math.cos(self.th), math.sin(self.th)
        return (self.x + self.sensor_offset * c - self.sensor_lat * s,
                self.y + self.sensor_offset * s + self.sensor_lat * c)

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
        self.stop_mode = None
        self.cmd_v, self.cmd_w = float(v), float(w)

    def cmd_stop(self, mode="coast"):
        """Pybricks: stop()=滑行(coast)，brake()=被动制动，hold()=主动保持。都不是瞬间停。"""
        self.man = None
        self.cmd_v = self.cmd_w = 0.0
        self.stop_mode = mode

    def cmd_straight(self, mm, speed):
        self.stop_mode = None
        self.man = {"kind": "d", "start": self.dist_acc, "target": mm, "speed": abs(speed)}

    def cmd_turn(self, deg, rate):
        self.stop_mode = None
        self.man = {"kind": "a", "start": self.ang_acc, "target": deg, "speed": abs(rate)}

    def maneuver_done(self):
        return self.man is None

    def cmd_reset(self, d=0, a=0):
        self.cmd_stop("hold")
        self.v = self.w = 0.0
        self.dist_acc, self.ang_acc = d / self.ds, a / self.ts

    def odo_distance(self):
        return int(self.dist_acc * self.ds)

    def odo_angle(self):
        return self.ang_acc * self.ts

    def motor_angle(self, side):
        return 0

    def motor_load(self, side):
        """单个电机负载估计（mNm）：摩擦 + 速度项 + 加速度项 + 漂移 + 白噪声 + 推物体 + 堵转。"""
        i = 0 if side == "L" else 1
        base = 18.0 + abs(self.v) * 0.04 + abs(self.w) * 0.03 + abs(self.acc_v) * 0.03
        extra = self.push_load / 2.0
        if self.stalled:
            extra += 150
        return base + extra + self.load_drift[i] + self.rng.gauss(0, self.load_noise)

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

    def battery_voltage(self):
        value = self.battery_initial_mv
        for t_from, voltage_mv in self.battery_events:
            if self.t < t_from:
                break
            value = voltage_mv
        return value

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
        return False

    def step(self, dt_ms):
        dt = dt_ms / 1000.0
        tv, tw = self._targets()
        if self.stop_mode is not None:
            dec = {"coast": self.coast_decel, "brake": self.brake_decel}.get(self.stop_mode, self.accel)
            dv = dec * dt
            dw = dec * dt
        else:
            dv = self.accel * dt
            dw = self.turn_accel * dt
        v0 = self.v
        self.v += max(-dv, min(dv, tv - self.v))
        self.w += max(-dw, min(dw, tw - self.w))
        self.acc_v = (self.v - v0) / dt

        nth = self.th - math.radians(self.w) * dt
        nx = self.x + self.v * dt * math.cos(nth)
        ny = self.y + self.v * dt * math.sin(nth)
        if self.v > 0 and (self._blocked(nx, ny, nth) or self._immovable_contact(nx, ny, nth)):
            self.block_ms += dt_ms
            nx, ny = self.x, self.y
            moved = 0.0
        else:
            self.block_ms = 0.0
            moved = self.v * dt
        self.stalled = self.block_ms > 200
        ox, oy, oth = self.x, self.y, self.th
        self.x, self.y, self.th = nx, ny, nth
        self.dist_acc += moved
        self.ang_acc += self.w * dt
        if self.legacy_objects:
            self._objects_legacy(moved)
        else:
            self._push_objects(ox, oy, oth, dt_ms)
        for i in (0, 1):
            self.load_drift[i] += -0.01 * self.load_drift[i] + self.rng.gauss(0, 0.3)

        sx, sy = self.sensor_pos()
        self.max_out = max(self.max_out, self._outside_dist(sx, sy))
        self.max_out_center = max(self.max_out_center, self._outside_dist(self.x, self.y))
        c, s_ = math.cos(self.th), math.sin(self.th)
        wy = self.g["wheel_y"]
        for sgn in (-1, 1):
            wx, wyy = self.x - sgn * wy * s_, self.y + sgn * wy * c
            self.wheel_max_out = max(self.wheel_max_out, self._outside_dist(wx, wyy))
        self.t += dt_ms
        if self.step_hook is not None:
            self.step_hook(self)
        if self.t > self.max_ms:
            raise SimTimeout()

    def _outside_dist(self, px, py):
        dx = max(-px, 0, px - self.W)
        dy = max(-py, 0, py - self.H)
        return max(dx, dy)

    # ---------------------------------------------------------- 物体：准静态接触推动
    def segments_robot_frame(self):
        g = self.g
        segs = []
        fr, bk, bh = g["body_front"], g["body_back"], g["body_half"]
        segs += [((fr, -bh), (fr, bh)), ((bk, -bh), (bk, bh)), ((bk, bh), (fr, bh)), ((bk, -bh), (fr, -bh))]
        if g["plow"]:
            ax, dp, ph = g["apex"], g["plow_depth"], g["plow_half"]
            segs += [((ax, 0.0), (ax + dp, ph)), ((ax, 0.0), (ax + dp, -ph)),
                     ((fr, 0.0), (ax, 0.0))]      # 推板支架
            if g.get("side_plates", True) and ph >= bh:
                xs = ax + dp * bh / ph            # 臂到达车宽处的 x
                segs += [((fr, bh), (xs, bh)), ((fr, -bh), (xs, -bh))]
        return segs

    def _world_segments(self, x, y, th):
        c, s = math.cos(th), math.sin(th)
        out = []
        for (a, b) in self.segments_robot_frame():
            out.append(((x + a[0] * c - a[1] * s, y + a[0] * s + a[1] * c),
                        (x + b[0] * c - b[1] * s, y + b[0] * s + b[1] * c)))
        return out

    @staticmethod
    def _closest(p, a, b):
        ax, ay = a
        bx, by = b
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * ex + (p[1] - ay) * ey) / L2))
        return ax + t * ex, ay + t * ey

    def _immovable_contact(self, nx, ny, nth):
        for o in self.objects:
            if not o.get("immovable"):
                continue
            for (a, b) in self._world_segments(nx, ny, nth):
                cx, cy = self._closest((o["x"], o["y"]), a, b)
                if (o["x"] - cx) ** 2 + (o["y"] - cy) ** 2 < o["r"] ** 2:
                    return True
        return False

    def _push_objects(self, ox, oy, oth, dt_ms):
        """车体/推板线段扫过物体时把物体沿接触法向推出（准静态、无惯性），
        再按接触摩擦系数带走一部分切向运动。物体自己不会动。"""
        self.push_load = 0.0
        segs = self._world_segments(self.x, self.y, self.th)
        for o in self.objects:
            if o["out"] or o.get("immovable"):
                continue
            x0, y0 = o["x"], o["y"]
            r = o["r"]
            for _ in range(30):
                hit = False
                for (a, b) in segs:
                    cx, cy = self._closest((o["x"], o["y"]), a, b)
                    dx, dy = o["x"] - cx, o["y"] - cy
                    d = _hypot(dx, dy)
                    if d >= r:
                        continue
                    hit = True
                    if d < 1e-6:
                        ex, ey = b[0] - a[0], b[1] - a[1]
                        L = _hypot(ex, ey) or 1.0
                        dx, dy, d = -ey / L, ex / L, 1.0
                        nxn, nyn = dx, dy
                        pen = r
                    else:
                        nxn, nyn = dx / d, dy / d
                        pen = r - d
                    o["x"] += nxn * pen
                    o["y"] += nyn * pen
                    # 切向摩擦：接触点随车体移动的切向分量带走一部分
                    if self.contact_mu > 0:
                        rx, ry = cx - self.x, cy - self.y
                        c0, s0 = math.cos(oth - self.th), math.sin(oth - self.th)
                        px_old = ox + (rx * c0 - ry * s0) * 1.0
                        py_old = oy + (rx * s0 + ry * c0) * 1.0
                        mvx, mvy = cx - px_old, cy - py_old
                        tx, ty = -nyn, nxn
                        tang = mvx * tx + mvy * ty
                        o["x"] += self.contact_mu * tang * tx
                        o["y"] += self.contact_mu * tang * ty
                if not hit:
                    break
            moved = _hypot(o["x"] - x0, o["y"] - y0)
            if moved > 0.02:
                o["pushed_ms"] += dt_ms
                o.setdefault("first_push_t", self.t)
                self.push_load += o["load"]
            if self.in_pocket(o):
                o.setdefault("captured_t", self.t)
            if self._outside_dist(o["x"], o["y"]) > self.band:
                o["out"] = True
                o["out_t"] = self.t

    def in_pocket(self, o):
        """物体是否在 V 形推板口袋里（仅用于统计）。"""
        g = self.g
        if not g["plow"]:
            return False
        c, s = math.cos(self.th), math.sin(self.th)
        dx, dy = o["x"] - self.x, o["y"] - self.y
        f = dx * c + dy * s
        l = -dx * s + dy * c
        k = (f - g["apex"]) / g["plow_depth"]
        return 0 < k <= 1.1 and abs(l) <= k * g["plow_half"] + 2

    # 旧模型（理想化：进口即吸附在顶点，只有倒车才脱落），保留用于对比
    PLOW_D = 95.0
    PLOW_HALF = 70.0

    def _objects_legacy(self, moved):
        c, s = math.cos(self.th), math.sin(self.th)
        self.push_load = 0.0
        for o in self.objects:
            if o.get("carried"):
                if self.v < -5:
                    o["carried"] = False
                    continue
                o["x"] = self.x + (self.PLOW_D - o["r"]) * c
                o["y"] = self.y + (self.PLOW_D - o["r"]) * s
                self.push_load += o["load"]
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
