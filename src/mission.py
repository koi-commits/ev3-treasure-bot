r"""
mission.py — 完整自主任务。逻辑全部【离线验证，待实机验证】，
巡线核心复用已实机验证的 fsm.EdgeFollower（与 demo_edge 同参数、同时序）。

    pbrun.cmd src\mission.py

模式（config.MISSION_MODE）：
  edge_to_finish  寻边 -> 沿边巡线 -> 运行中识别终点色标 -> 停车
  survey          寻边 -> 沿边巡线 FOLLOW_MAX_MS，只记录色标/遥测，不停车
  calib           找终点 -> 色标定位 -> 场上转角自标定，打印 turn_k（不需要物体）
  capture         只做搜索 + 负载嫌疑 + tap 确认，确认即停（不放物体 = 测误报率）
  full            找终点 -> 场上转角自标定 + 色标定位（calibrate_and_map）
                  -> 对角线反弹搜索（V 形推板）-> 负载嫌疑 + tap/seat 确认
                  -> 推送：纯追踪到场内待命点，再正对色标推进，让物体中心到目标点 -> 倒车放下
                  （推送中负载回落 = 物体丢失 -> 回到搜索；到不了目标 -> 放弃本次，不虚报）

推送模式（ctx.carrying）的规则（来自仿真接触模型：原地转会把物体甩出推板）：
  * 不倒车、不原地转；转向只用"前进中的弧线"（carry_turn）
  * 沿边时 ESCAPE 用前进中的急弯（EdgeFollower escape_speed>0）
  * 按里程计让**物体**（不是光斑）停在色标点上；光斑先压到色标也立即停
  * 推送中不沿边找色标（仿真：常把物体推出界）

异常恢复（监督层，不改动三状态机本身）：
  * 丢边：CRUISE 中 LOST_EDGE_MS 未见白 -> 重新直行寻边
  * 越界：ESCAPE 续转用完仍在白区 -> 倒车 OVERRUN_BACK_MM，状态机重置
  * 堵转：drive.stalled() 持续 STALL_MS -> 倒车 + 转离边界
  * 出界：直行超过 ARENA_DIAG_MM 仍未见白（凸场地内不可能）-> 朝最近见白位置返回、穿过白带
  * 起步光斑在白边上 -> 先直线倒车回到地面
  * 在白边处停车用 brake（stop() 是滑行，会多冲一段）
  * 所有定距动作都带超时（drive.done() 轮询），不会因堵转卡死；整场 MISSION_MAX_MS 兜底

实机可观测的越界指标（SUMMARY 里）：max_white_run_mm = 光斑连续见白期间车走过的最大距离。

按键：RIGHT 确认后开始（不会超时自动开车），CENTER 随时急停（Pybricks 默认）。
"""

import config
from common import aread, battery_check, log, require_gyro_sign, wait_for_button
from fsm import (CLEAR, CRUISE, ESCAPE, EdgeFollower, LoadMonitor, Odometry, axis_snap,
                 diagonal_choices, heading_of, rotate_about, straight_run_heading, tap_verdict,
                 wrap180)
from hardware import color, drive, ev3, gyro, left, right, touch
from sensing import Sensing

from pybricks.parameters import Color
from pybricks.tools import StopWatch, run_task, wait

try:
    from umath import atan2, cos, pi, sin, sqrt
except ImportError:
    from math import atan2, cos, pi, sin, sqrt

def hypot(x, y):
    return sqrt(x*x + y*y)


from boundary import Boundary, fit_axis, orient_axis

try:
    import urandom as random
except ImportError:
    import random

PERIOD = config.SAMPLE_PERIOD_MS

LANE_VARIANT_CFG = "LANE_VARIANT_EXPERIMENT"   # 调试用
FINISH = config.FINISH_LABEL
SIDE = config.EDGE_SIDE
INTO_ARENA = -SIDE              # 远离白边的转向符号（与 ESCAPE 方向一致）
TRACE_MS = 1000


def rand_sign():
    return 1 if random.randint(0, 1) else -1


class Abort(Exception):
    pass


class Ctx:
    """整场任务共享状态 + 统计。"""

    def __init__(self, sense):
        self.phase = "edge_probe"
        self.boundary = None
        self.boundary_hits = []
        self.white_backoff = None
        self.use_fence = True
        self.boundary_kind = None     # last boundary reading: "white" / "red"
        self.scan_wall_k = None       # wall family being scanned for the marker
        self.marker_wall_k = None     # wall family the marker was found on
        self.marker_along_center = None
        self.marker_inner_mm = None
        self.marker_extent_mm = None
        self.sense = sense
        self.odom = Odometry()
        self.clock = StopWatch()
        self.stall_ms = 0
        self.finish_pose = None      # 确认终点时车体位姿 (x, y, heading)
        self.finish_pt = None        # 确认终点时光斑位置 (x, y) = 交付目标点
        self.markers_seen = {}
        self.last_marker = None
        self.n = 0
        self.white_n = 0
        self.twists = 0
        self.lost = 0
        self.overruns = 0
        self.stalls = 0
        self.seek_fail = 0
        self.suspects = 0            # 负载嫌疑次数
        self.captures = 0            # tap 确认通过次数
        self.carrying = False
        self.last_white = None       # 最近一次见白时的车体位置 (x, y)
        self.outs = 0
        self.edge_trace = []         # 最近几次 ESCAPE 时的车体位置 -> 估计边的方向
        self.axis = None             # 场地轴向（矩形：所有边平行/垂直于它），DriveBase 航向
        self.marker_pts = []         # 光斑落在终点色标上的位置（原始标签，用于求色标中心）
        self.target = None           # 交付目标点：物体中心应停在这里
        self.load_base = None        # tap 自由段负载（推送中判断物体是否丢失的参照）
        self.lost_objects = 0
        self.heading_fixes = 0       # edge peek 航向修正次数
        self.heading_fix_sum = 0.0   # 修正量绝对值累计（度）
        self.turn_k = None           # 场上自标定得到的转角比例
        self.waypoint = None         # 交付前的场内待命点（正对色标）
        self.white_d0 = None         # 本次连续见白开始时的 drive.distance()
        self.max_white_run = 0       # 连续见白期间走过的最大距离（mm）
        self.next_trace = 0
        # ---- Mission v3 ----
        self.use_gyro = False        # main() 里按 config.USE_GYRO / gyro is not None 决定
        self.use_touch = False       # 只有 v3 模式每个 tick 读触碰（老模式省读时间）
        self.gyro_sign = 1.0         # require_gyro_sign() 的结果（+1 = 右转时角度增大）
        self.pressed_n = 0           # 触碰挡板连续 pressed 计数（去抖）
        self.carry_inplace_turns = 0 # 携带中发生过的原地转（应为 0）
        self.last_press = 0          # 最近一次 pressed 的时刻（运丢判定用）
        self.captured = False        # 已通过触碰确认捕获（去抖后锁存）
        self.path = []               # 最近的车体位姿轨迹
        self.last_white_ms = 0       # 最近一次见白的时刻（转角确认要用）
        self.prev_white = False      # 上一周期是否见白（找"刚压上白带"的时刻）
        self.onsets = 0              # 见白的上升沿次数（判断是否真的在沿边）
        self.white_hits = []         # 白带前沿的触碰点（拟合墙线方向，最多 24 个）
        self.lane_log = []           # 每条车道的结果：(方向, 结果, 里程)
        self.lane_dir = None         # 车道前进方向（DriveBase 航向），±90° 于边
        self.object_pt = None        # 捕获时物体中心落在里程计坐标里的位置（色标未知时先存着）
        self.sweep_adv_mm = 0.0      # 本次扫掠沿墙推进的里程（反向补扫前要退回来）
        self.sweep_adv_dir = 0.0     # 沿墙推进的方向

    def now(self):
        return self.clock.time()


ctx = None


def ev(*args):
    """事件日志：终端带时间戳，屏幕只显示内容。"""
    print("[{:6d}]".format(ctx.now()), *args)
    ev3.screen.print(*args)


def load_sum():
    return abs(left.load()) + abs(right.load())


def carry_lost():
    """推送中物体是否掉出推板（v3：触碰挡板连续 LOST_MS 没被压住）。"""
    if not ctx.carrying:
        return False
    if config.USE_TOUCH and touch is not None:
        return ctx.now() - ctx.last_press > config.LOST_MS
    return False


async def read_heading():
    """当前航向（deg，DriveBase 约定：正 = 右转）。

    v3：航向来自陀螺仪（EV3 的 DriveBase 没有 use_gyro，只能自己闭环），
    符号由 GYRO_POSITIVE_DIRECTION 给出。没有陀螺仪 / 关掉 USE_GYRO 时退回编码器。
    """
    if not ctx.use_gyro:
        return drive.angle()
    return (await aread(gyro.angle())) * ctx.gyro_sign


async def read_touch():
    """触碰挡板是否被压住。必须经 aread()：实机上 pressed() 可能是 awaitable。"""
    if touch is None or not config.USE_TOUCH or not ctx.use_touch:
        return False
    return bool(await aread(touch.pressed()))


async def odom_update():
    ctx.odom.update(drive.distance(), await read_heading())


def ahead(x, y, heading, d, lat=0.0):
    """从 (x, y) 沿 DriveBase 航向 heading 前进 d、再向左偏移 lat 的点。"""
    rad = -heading * pi / 180.0
    fx, fy = cos(rad), sin(rad)
    return x + d * fx - lat * fy, y + d * fy + lat * fx


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def hold_heading(speed, target):
    """保持航向直行：DriveBase 只给 (速度, 转向率)，转向率由航向误差闭环给出。"""
    err = wrap180(target - ctx.odom.heading)
    drive.drive(speed, clamp(config.KP_HEADING * err, -config.MAX_TURN_RATE, config.MAX_TURN_RATE))


async def tick():
    """每个循环周期调用一次：读传感器、更新里程计、堵转计时、触碰/色标记录、超时检查。"""
    s = await ctx.sense.read()
    await odom_update()
    ctx.n += 1
    o = ctx.odom
    b = is_boundary(s)
    if b:
        ctx.white_n += 1
        ctx.boundary_kind = "red" if s.label == FINISH else "white"
        ctx.last_white = (o.x, o.y)
        ctx.last_white_ms = ctx.now()
        if not ctx.prev_white:
            point = ahead(o.x, o.y, o.heading, config.SENSOR_OFFSET_MM, config.SENSOR_LATERAL_MM)
            if s.white and len(ctx.white_hits) < 24:
                # 白带前沿的触碰点：它们都躺在同一条墙线上，用来拟合墙的方向
                ctx.white_hits.append(point)
            ctx.onsets += 1
            ctx.boundary_hits.append((point, o.heading, ctx.boundary_kind))
            if len(ctx.boundary_hits) > 200:
                ctx.boundary_hits.pop(0)
            if ctx.boundary is not None:
                ctx.boundary.observe(point, o.heading)
            if ctx.boundary_kind == "red" and ctx.scan_wall_k is not None:
                ctx.marker_wall_k = ctx.scan_wall_k
            ctx.white_backoff = 0.0
        if ctx.white_d0 is None:
            ctx.white_d0 = drive.distance()
        run = abs(drive.distance() - ctx.white_d0)
        if run > ctx.max_white_run:
            ctx.max_white_run = run
    else:
        ctx.white_d0 = None
    ctx.prev_white = b
    if drive.stalled():
        ctx.stall_ms += PERIOD
    else:
        ctx.stall_ms = 0

    # ---- 触碰（v3 捕获）：所有阶段都看，去抖后锁存 ----
    if await read_touch():
        ctx.pressed_n += 1
        ctx.last_press = ctx.now()
        if not ctx.carrying and ctx.pressed_n >= config.TOUCH_DEBOUNCE_N and not ctx.captured:
            ctx.captured = True
            ev("touch captured")
    else:
        ctx.pressed_n = 0

    # ---- 红点（色标）记录：光斑位置 = 车体位姿 + 前向/侧向偏移 ----
    if s.label == FINISH and not ctx.carrying:
        p = ahead(o.x, o.y, o.heading, config.SENSOR_OFFSET_MM, config.SENSOR_LATERAL_MM)
        if len(ctx.marker_pts) < 300 and not near_any(p, ctx.marker_pts, config.MARKER_HIT_DEDUPE_MM):
            ctx.marker_pts.append(p)

    # ---- 轨迹（给 edge_probe 的定轴用；滚动保留最近 240 个 tick）----
    if len(ctx.path) < 240:
        ctx.path.append((o.x, o.y))
    else:
        ctx.path.pop(0)
        ctx.path.append((o.x, o.y))

    if s.marker is not None and s.marker != ctx.last_marker:
        ctx.markers_seen[s.marker] = ctx.markers_seen.get(s.marker, 0) + 1
        ev("marker", s.marker, "x={:.0f} y={:.0f}".format(o.x, o.y))
        if s.marker == FINISH and ctx.finish_pose is None:
            ctx.finish_pose = (o.x, o.y, o.heading)
            ctx.finish_pt = ahead(o.x, o.y, o.heading, config.SENSOR_OFFSET_MM)
            ev3.speaker.beep(1200, 80)
    ctx.last_marker = s.marker

    if ctx.now() >= ctx.next_trace:
        print("  trace refl={} rgb={} lab={} w={} xy=({:.0f},{:.0f}) h={:.0f}".format(
            s.refl, s.rgb, s.label, int(s.white), o.x, o.y, o.heading))
        ctx.next_trace = ctx.now() + TRACE_MS

    if ctx.now() > config.MISSION_MAX_MS:
        raise Abort("mission timeout")
    return s


def near_any(p, pts, tol):
    """p 是否落在已有任何一个点的 tol 以内（红点去重）。"""
    for q in pts:
        if abs(p[0] - q[0]) <= tol and abs(p[1] - q[1]) <= tol:
            return True
    return False


def dot(p, q):
    return p[0]*q[0] + p[1]*q[1]


def is_boundary(s):
    """White tape or the red paper on top of it: both are the arena edge."""
    return s.white or getattr(s, "label", None) == FINISH


def marker_seen():
    """A red reading is enough to start measuring; 3 hits are only needed for a centroid."""
    return ctx.finish_pose is not None or marker_center() is not None


def marker_anchor():
    c = marker_center()
    if c is not None:
        return c
    return ctx.marker_pts[0] if ctx.marker_pts else None


# -------------------------------------------------------------------
# 基本动作：定距/定角，带超时，结束后更新里程计
# -------------------------------------------------------------------
async def _settle(timeout_ms):
    t = 0
    while not drive.done():
        await wait(PERIOD)
        t += PERIOD
        await odom_update()
        if t > timeout_ms:
            drive.brake()
            ev("move timeout")
            break
    await odom_update()


async def straight(mm):
    await odom_update()
    drive.straight(mm, wait=False)
    await _settle(2000 + abs(mm) * 25)
    if ctx.white_backoff is not None:
        ctx.white_backoff = max(0.0, ctx.white_backoff - mm)


async def rotate(deg):
    if ctx.carrying:
        ctx.carry_inplace_turns += 1
        raise AssertionError("in-place turn while carrying")
    if ctx.use_gyro and ctx.white_backoff is not None:
        required = config.AXLE_TRACK_MM / 2 + 30
        if ctx.white_backoff < required:
            await straight(-(required - ctx.white_backoff))
        ctx.white_backoff = None
    await odom_update()
    drive.turn(deg, wait=False)
    await _settle(2000 + abs(deg) * 25)


async def unstall():
    ctx.stalls += 1
    ev("stall -> back")
    drive.brake()
    await straight(-config.STALL_BACK_MM)
    await rotate(INTO_ARENA * config.STALL_TURN_DEG)
    ctx.stall_ms = 0


async def drive_until(speed, max_mm, stop_on_finish=False, monitor=None, geofence=False):
    """直行（DriveBase 自带航向保持）直到：
    'white' 连续 EDGE_CONFIRM_SAMPLES 个白样本 | 'finish' 色标确认 |
    'stall' | 'suspect'（monitor 判定负载抬升）| 'distance' |
    'capture'（v3 触碰确认捕获）| 'lost'（推送中物体丢了）|
    'geofence'（geofence=True 且推板前缘到达估计边界）。停车一律 brake。"""
    d0 = drive.distance()
    whites = 0
    low = 0
    drive.drive(speed, 0)
    while True:
        s = await tick()
        r = None
        if geofence and fence_reached(speed):
            drive.brake()
            return "geofence"
        if ctx.captured and not ctx.carrying:
            drive.brake()
            return "capture"
        # 推送中：物体丢了（v3 = 触碰挡板连续 LOST_MS 没被压住；旧 = 负载回落）
        if ctx.carrying and carry_lost():
            drive.brake()
            return "lost"
        if ctx.carrying and ctx.load_base is not None and abs(drive.distance() - d0) > 15:
            if load_sum() < ctx.load_base + config.TAP_MARGIN * 0.5:
                low += 1
            else:
                low = 0
            if low >= config.CARRY_LOST_N:
                drive.brake()
                return "lost"
        if stop_on_finish and s.marker == FINISH:
            r = "finish"
        elif is_boundary(s):
            whites += 1
            if whites >= config.EDGE_CONFIRM_SAMPLES:
                r = "white"
        else:
            whites = 0
        if r is None:
            if ctx.stall_ms >= config.STALL_MS:
                r = "stall"
            elif monitor is not None and monitor.push(load_sum()):
                r = "suspect"
            elif abs(drive.distance() - d0) >= max_mm:
                r = "distance"
        if r is not None:
            drive.brake()
            return r
        await wait(PERIOD)


# -------------------------------------------------------------------
# 起步检查 + 寻边：随机起点/朝向
# -------------------------------------------------------------------
async def start_check():
    """起步时光斑已压在白边上：直线倒车直到连续看到地面。
    前提：车体中心放在深色场地内（摆放规则），倒车 = 光斑回到场内。"""
    s = await tick()
    if not is_boundary(s):
        return
    ev("start on white -> back")
    d0 = drive.distance()
    dark = 0
    drive.drive(-config.SEEK_SPEED, 0)
    while abs(drive.distance() - d0) < config.START_BACK_MAX_MM and dark < 3:
        await wait(PERIOD)
        s = await tick()
        dark = 0 if is_boundary(s) else dark + 1
    drive.brake()
    if dark < 3:
        ev("still white: turn 180")
        await rotate(180)
    else:
        await straight(-config.SEEK_BACKOFF_MM)


async def seek_edge(stop_on_finish):
    """返回 'edge' / 'finish' / 'capture' / None（全部尝试失败）。
    推着物体时（ctx.carrying）不倒车：V 形推板倒车会把物体留在原地。"""
    for attempt in range(config.SEEK_ATTEMPTS):
        r = await drive_until(config.CARRY_SPEED if ctx.carrying else config.SEEK_SPEED,
                              config.ARENA_DIAG_MM, stop_on_finish)
        if r == "finish":
            return "finish"
        if r == "capture":
            return "capture"
        if r == "white":
            ev3.speaker.beep(900, 60)
            ev("edge found", attempt)
            if not ctx.carrying:
                await straight(-config.SEEK_BACKOFF_MM)
            return "edge"
        if r == "stall":
            await unstall()
        elif ctx.last_white is not None:
            # 直线超过对角线仍未见白：凸场地内不可能 -> 已在场外，走回去
            ctx.seek_fail += 1
            await return_inside()
            continue
        else:  # 还从没见过白：可能场地比配置大，换方向再试
            ctx.seek_fail += 1
            ev("seek: no edge, turn")
        await rotate(random.randint(90, 180) * rand_sign())
    return None


async def park_early_capture():
    """Leave the tyre where it was caught and turn away before seeking again."""
    drive.brake()
    heading = ctx.odom.heading
    point = ahead(ctx.odom.x, ctx.odom.y, heading, config.OBJ_LEAD_MM)
    ctx.phase = "early_drop"
    ev("early capture: park at", point)
    ctx.carrying = False
    await straight(-150)
    ctx.captured = False
    ctx.pressed_n = 0
    await rotate(90 * rand_sign())
    ctx.phase = "seek"
    return point, heading


# -------------------------------------------------------------------
# 沿边巡线 + 监督
# -------------------------------------------------------------------
async def follow(max_ms, until_finish, side=SIDE):
    """返回 'finish' / 'lost' / 'overrun' / 'time'。推送模式下 ESCAPE 用前进急弯。"""
    esc_v = config.CARRY_ESCAPE_SPEED if ctx.carrying else 0
    if config.FOLLOW_PROFILE == "v3_sim":
        speed, steer, rate, esc_ms, clear_ms = config.V3_SIM_FOLLOW
    else:
        steer, esc_ms, clear_ms = config.FOLLOW_PROFILES[config.FOLLOW_PROFILE]
        speed, rate = config.FOLLOW_SPEED, config.EDGE_TURN_RATE
    f = EdgeFollower(side, speed, steer, rate, esc_ms, clear_ms,
                     escape_speed=esc_v)
    t = 0
    handled_exhaust = 0
    overrun_streak = 0
    seg_n = 0
    seg_white = 0
    prev_state = CRUISE
    while t < max_ms:
        s = await tick()
        seg_n += 1
        if is_boundary(s):
            seg_white += 1

        if ctx.captured and not ctx.carrying:
            drive.brake()
            ctx.twists += f.twists
            return "capture"

        if ctx.carrying and carry_lost():
            drive.brake()
            ctx.twists += f.twists
            return "lost"

        if until_finish and s.marker == FINISH:
            drive.brake()
            ctx.twists += f.twists
            return "finish"

        tw = f.twists
        sp, tr = f.step(t, is_boundary(s))
        drive.drive(sp, tr)
        if f.twists != tw:
            ctx.edge_trace.append((ctx.odom.x, ctx.odom.y))
            if len(ctx.edge_trace) > 10:
                ctx.edge_trace.pop(0)

        # ---- 监督层 ----
        if prev_state == ESCAPE and f.state == CLEAR and f.exhausted == handled_exhaust:
            overrun_streak = 0              # 一次干净的逃逸（转出了白区），清零越界计数
        prev_state = f.state

        if f.exhausted != handled_exhaust:
            handled_exhaust = f.exhausted
            overrun_streak += 1
            ctx.overruns += 1
            ev("overrun", overrun_streak)
            drive.brake()
            await straight(-config.OVERRUN_BACK_MM)
            if overrun_streak > config.OVERRUN_LIMIT:
                ctx.twists += f.twists
                return "overrun"
            f.reset(t)
            handled_exhaust = 0
            prev_state = CRUISE

        elif f.state == CRUISE and t - f.last_white_t > config.LOST_EDGE_MS:
            drive.brake()
            ctx.lost += 1
            ctx.twists += f.twists
            ev("edge lost")
            return "lost"

        elif ctx.stall_ms >= config.STALL_MS:
            await unstall()
            f.reset(t)
            handled_exhaust = 0
            prev_state = CRUISE

        await wait(PERIOD)
        t += PERIOD

    drive.brake()
    ctx.twists += f.twists
    if seg_n:
        print("  follow white_frac={:.2f} twists={}".format(seg_white / seg_n, f.twists))
    return "time"


async def edge_then_follow(max_ms, until_finish, side=SIDE):
    """寻边 + 巡线 + 丢边/越界后重新寻边，直到找到终点或用完时间。"""
    deadline = ctx.now() + max_ms
    if not ctx.carrying:
        await start_check()
    while ctx.now() < deadline:
        r = await seek_edge(until_finish)
        if r == "finish":
            return "finish"
        if r is None:
            return "seek_failed"
        ctx.edge_trace = []
        r = await follow(deadline - ctx.now(), until_finish, side)
        if r == "finish":
            return "finish"
        if r == "time":
            return "time"
        if r == "overrun":
            await rotate(INTO_ARENA * 150)     # 疑似越界：掉头回场地内部再寻边
        # 'lost'：保持当前朝向直接重新直行寻边
        # （试过"朝最后见白位置回头"，会与 seek_edge 的旋转叠加，lost_edge 用例 4/4 -> 1/4）
    return "time"


async def return_inside():
    """直线走了超过场地对角线仍未见白 = 已在场外（凸场地内不可能）。
    朝最近一次见白的位置走回去，穿过白带，再往场内走 RETURN_IN_MM。"""
    ctx.outs += 1
    ev("outside -> return")
    if ctx.last_white is None:
        await rotate(180)
    else:
        rel, dist = ctx.odom.bearing_to(ctx.last_white[0], ctx.last_white[1])
        await rotate(rel)
    r = await drive_until(config.SEEK_SPEED, config.ARENA_DIAG_MM + 400)
    if r == "white":
        d0 = drive.distance()
        dark = 0
        drive.drive(config.SEEK_SPEED, 0)
        while dark < 3 and abs(drive.distance() - d0) < 300:
            await wait(PERIOD)
            s = await tick()
            dark = 0 if is_boundary(s) else dark + 1
        drive.brake()
        await straight(config.RETURN_IN_MM)


# -------------------------------------------------------------------
# full 模式：搜索 -> 确认 -> 推送 -> 交付
# -------------------------------------------------------------------
def new_monitor():
    return LoadMonitor(config.LOAD_SKIP_N, config.LOAD_LEARN_N, config.LOAD_WINDOW,
                       config.LOAD_K, config.LOAD_ABS_MARGIN, config.LOAD_HOLD_N)


async def tap_confirm(mon):
    """负载嫌疑后的确认（tap + seat）：
      1. 倒车 TAP_BACK_MM 松开物体（物体留在原地）；
      2. 低速前进："自由段"（还没碰到）-> "重新接触早段" -> "推进晚段"（再推约 TAP_SEAT_MM）。
    早段和晚段负载都要比自由段高出 TAP_MARGIN 才算确认。
    晚段很重要：物体若只是被推板**外侧**斜面碰到，继续直推时会滑开、负载回落 —— 那不是"在口袋里"。"""
    ctx.suspects += 1
    drive.brake()
    await wait(150)
    back = config.TAP_BACK_MM
    seat = config.TAP_SEAT_MM
    await straight(-back)
    await wait(150)
    d0 = drive.distance()
    free = []
    early = []
    late = []
    drive.drive(config.TAP_SPEED, 0)
    while True:
        s = await tick()
        trav = abs(drive.distance() - d0)
        if is_boundary(s):
            break
        x = load_sum()
        if 8 <= trav <= back * 0.5:
            free.append(x)
        elif back + 15 <= trav < back + seat * 0.5:
            early.append(x)
        elif trav >= back + seat * 0.6:
            late.append(x)
        if trav >= back + seat:
            break
        await wait(PERIOD)
    drive.brake()
    ok1, d1 = tap_verdict(free, early, config.TAP_MARGIN)
    ok2, d2 = tap_verdict(free, late, config.TAP_MARGIN)
    ok = ok1 and ok2
    ev("tap d_early={:.1f} d_late={:.1f} -> {}".format(d1, d2, "YES" if ok else "no"))
    if ok and free:
        ctx.load_base = sum(free) / len(free)
    return ok


async def carry_turn(deg):
    """推送模式下转向：前进中的弧线（物体被持续压在推板顶点），绝不原地转。"""
    if abs(deg) < 1:
        return "ok"
    a0 = ctx.odom.heading
    sgn = 1 if deg > 0 else -1
    drive.drive(config.CARRY_SPEED, sgn * config.CARRY_TURN_RATE)
    while abs(wrap180(ctx.odom.heading - a0)) < abs(deg):
        s = await tick()
        if s.marker == FINISH:
            drive.brake()
            return "finish"
        if is_boundary(s):
            drive.brake()
            return "white"
        await wait(PERIOD)
    drive.brake()
    return "ok"


async def turn_to(target):
    """转到绝对航向 target。运送中只能走前进弧线：物体一直被压在推板顶点，
    原地转会把物体甩出去（2026-11-17 仿真里捕获后第一次原地转就丢了）。"""
    d = wrap180(target - ctx.odom.heading)
    if abs(d) < 1.0:
        return "ok"
    if ctx.carrying:
        return await carry_turn(d)
    await rotate(d)
    return "ok"


def pick_diagonal():
    """随机反弹的新航向：已知场地轴向时选对角线方向（±BOUNCE_JITTER_DEG），否则返回 None。"""
    if ctx.axis is None:
        return None
    c = diagonal_choices(ctx.axis, ctx.odom.heading)
    if not c:
        return None
    h = c[random.randint(0, len(c) - 1)]
    j = config.BOUNCE_JITTER_DEG
    return h + random.randint(-j, j)


async def turn_for_next_leg():
    h = pick_diagonal()
    if h is None:
        await rotate(rand_sign() * random.randint(100, 170))
    else:
        await rotate(wrap180(h - ctx.odom.heading))


async def edge_peek():
    """搜索中每段碰边后：沿这条边走 PEEK_MS（已验证的巡线），由锯齿点测出这条边的方向，
    吸附到场地轴向（矩形：边 = axis + k*90），把累计的航向误差从里程计里扣掉。
    原地转很多次后航向误差会累积（3% 转角误差 × 十几次转向 ≈ 几十度），不修正就送不准。"""
    if ctx.axis is None or not config.EDGE_PEEK:
        return
    await straight(-config.SEEK_BACKOFF_MM)     # 与 edge_to_finish 相同的入口：先退再巡线
    ctx.edge_trace = []
    await follow(config.PEEK_MS, False)
    m = straight_run_heading(ctx.edge_trace)
    if m is None:
        return
    delta = axis_snap(m, ctx.axis)
    if abs(delta) <= config.PEEK_MAX_CORR:
        ctx.odom.correct_heading(delta)
        ctx.heading_fixes += 1
        ctx.heading_fix_sum += abs(delta)
        ev("heading fix {:.1f}".format(delta))


async def bounce():
    """随机反弹覆盖：直行到白边 -> 倒车 -> 原地转到新方向 -> 再直行。
    新方向优先取场地对角线方向（碰边入射角约 45°，轮子不会先于光斑越过白边）。
    直行中负载可疑 -> tap 确认；确认则返回 'capture'（物体在推板前方）。"""
    mon = new_monitor() if config.CAPTURE_MODE == "load" else None
    for leg in range(config.BOUNCE_MAX_LEGS):
        if mon is not None:
            mon.restart()
        r = await drive_until(config.SWEEP_SPEED, config.ARENA_DIAG_MM, False, mon)
        ev("leg", leg, r)
        if r == "suspect" or r == "stall":
            if r == "stall":
                await straight(-20)
            if await tap_confirm(mon):
                ctx.captures += 1
                ev3.speaker.beep(700, 150)
                return "capture"
            continue                        # 误报：沿当前朝向继续
        if r == "distance":
            await return_inside()
            continue
        await edge_peek()
        await straight(-config.SWEEP_END_BACK_MM)
        await turn_for_next_leg()
    return "done"


def marker_center():
    """色标中心估计：先剔掉距中位数 > MARKER_REJECT_MM 的离群点，再取均值。

    红点可能来自场地别的红色物体（或车身反光），单靠均值会被拉偏。
    """
    pts = ctx.marker_pts
    if len(pts) < config.MARKER_MIN_HITS:
        return None
    mx = sorted(p[0] for p in pts)[len(pts) // 2]
    my = sorted(p[1] for p in pts)[len(pts) // 2]
    keep = [p for p in pts
            if abs(p[0] - mx) <= config.MARKER_REJECT_MM and abs(p[1] - my) <= config.MARKER_REJECT_MM]
    if not keep:
        return None
    return (sum(p[0] for p in keep) / len(keep), sum(p[1] for p in keep) / len(keep))


def marker_wall(c=None):
    """Which arena wall the paper lies on: (axis index, bound, inward/along vectors).

    The marker scan records the wall family it was facing when it found red
    (ctx.marker_wall_k); without that, fall back to the nearest observed bound.
    """
    if ctx.boundary is None:
        return None
    c = marker_anchor() if c is None else c
    if c is None:
        return None
    axes = ctx.boundary.axes
    candidates = []
    for k in ((ctx.marker_wall_k,) if ctx.marker_wall_k is not None else (0, 1)):
        u, v = axes[k]
        q = u*c[0] + v*c[1]
        for side, b in ((0, ctx.boundary.bounds[k][0]), (1, ctx.boundary.bounds[k][1])):
            if b is not None:
                candidates.append((abs(q - b), k, side, b))
    if not candidates:
        return None
    d, k, side, b = min(candidates)
    if d > config.MARKER_REJECT_MM:
        return None
    axis = axes[k]
    # The extrema used by the fence can sit a little past the tape (an oblique
    # hit or an overshoot).  For the marker target use the median of the white
    # hits on that same wall instead.
    vals = []
    for p, h, kind in ctx.boundary_hits:
        if kind != "white" or ctx.boundary.axis_index(h) != k:
            continue
        value = dot(p, axis)
        if (side == 1 and value > 0) or (side == 0 and value < 0):
            vals.append(value)
    if vals:
        b = median(vals)
    v_in = axis if side == 0 else (-axis[0], -axis[1])
    v_along = axes[1-k]
    return dict(k=k, b=b, side=side, axis=axis, v_in=v_in, v_along=v_along,
                line_n=b if side == 0 else -b,
                inward_heading=wrap180(heading_of(v_in[0], v_in[1])),
                along_heading=wrap180(heading_of(v_along[0], v_along[1])))


def marker_target():
    """Target = centre of the part of the paper inside the arena.

    Along the wall it is the paper centre; across it, halfway between the inner
    edge of the white line (the arena boundary) and the paper's inner edge.  The
    target is never past the arena boundary.
    """
    c = marker_anchor()
    if c is None:
        return None, None
    w = marker_wall(c)
    if w is None:
        return c, dict(inner=0.0, along=0.0, extent=0.0, wall=None)
    vi, va = w["v_in"], w["v_along"]
    ns = [dot(p, vi) - w["line_n"] for p in ctx.marker_pts]
    aa = [dot(p, va) for p in ctx.marker_pts]
    inner = max(ns) if ns else 0.0
    if ctx.marker_inner_mm is not None:
        inner = ctx.marker_inner_mm
    inner = clamp(inner, 0.0, config.MARKER_SIDE_MM * 0.75 + 15.0)
    if ctx.marker_along_center is not None:
        along = ctx.marker_along_center
    elif aa:
        along = (min(aa) + max(aa)) / 2.0
    else:
        along = dot(c, va)
    base = (w["b"] * w["axis"][0], w["b"] * w["axis"][1])
    target = (base[0] + along*va[0] + 0.5*inner*vi[0],
              base[1] + along*va[1] + 0.5*inner*vi[1])
    return target, dict(inner=inner, along=along,
                        extent=ctx.marker_extent_mm or 0.0, wall=w)


def marker_edge_inward():
    """色标所在那条边的「朝场内」法向。

    不能用扫掠用的 ctx.axis：那是**扫地时起步的那条边**，和色标所在的边可能垂直，
    于是待命点会落到色标旁边，交付时变成沿着白边推（光斑先压到白带、物体离目标
    还差 150 mm 就判失败）。色标自己的那些点都在同一条边沿上，直接拟合它们。
    """
    c = marker_center()
    if ctx.boundary is not None and c is not None:
        x, y = c
        choices = []
        for k,(u,v) in enumerate(ctx.boundary.axes):
            lo,hi = ctx.boundary.limits(k)
            q = u*x+v*y
            choices.append((abs(q-lo), heading_of(u, v)))
            choices.append((abs(hi-q), wrap180(heading_of(u, v) + 180)))
        return min(choices)[1]
    pts = ctx.marker_pts
    if len(pts) < 3:
        return None
    fit = _fit_line(pts)
    if fit is None:
        return None
    edge = fit[0]
    # 场内方向：路径质心（机器人走过的位置）相对色标中心的投影为正的那一支
    cx = sum(p[0] for p in ctx.path) / len(ctx.path) if ctx.path else 0.0
    cy = sum(p[1] for p in ctx.path) / len(ctx.path) if ctx.path else 0.0
    mx = sum(p[0] for p in pts) / len(pts)
    my = sum(p[1] for p in pts) / len(pts)
    vx, vy = cx - mx, cy - my
    best = None
    for cand in (wrap180(edge + 90.0), wrap180(edge - 90.0)):
        rad = -cand * pi / 180.0
        d = vx * cos(rad) + vy * sin(rad)
        if best is None or d > best[1]:
            best = (cand, d)
    return best[0]


def compute_target():
    """由红纸的实测范围求交付目标点与场内待命点。

    目标 = 纸在场内那部分的中心：沿墙方向取纸中心，横向取"白线内沿 ↔ 纸内沿"
    的中点，永不越过白线内沿。红点不足时退回旧的中心估计。
    """
    target, info = marker_target()
    if target is None:
        ctx.target = None
        ctx.waypoint = None
        return
    ctx.target = target
    w = None if info is None else info.get("wall")
    if w is not None:
        inward = w["inward_heading"]
    else:
        inward = marker_edge_inward()
        if inward is None:
            inward = None if ctx.axis is None else wrap180(ctx.axis + INTO_ARENA * 90.0)
    if inward is None:
        ctx.waypoint = None
    else:
        ctx.waypoint = ahead(ctx.target[0], ctx.target[1], inward, config.APPROACH_STANDOFF_MM)
    if info["inner"] < 20.0:
        ev("marker inside part narrow: {:.0f} mm".format(info["inner"]))
    ev("marker target: along {:.0f} extent {:.0f} inner {:.0f} -> ({:.0f},{:.0f})".format(
        info["along"], info["extent"], info["inner"], ctx.target[0], ctx.target[1]))


async def map_marker():
    """（无自标定时）终点首次确认后继续沿边走 MAP_MS，求色标中心与场地轴向。"""
    await follow(config.MAP_MS, False)
    ax = straight_run_heading(ctx.edge_trace)
    if ax is None:
        await follow(config.MAP_MS, False)    # 可能刚好在拐角附近：再走一段
        ax = straight_run_heading(ctx.edge_trace)
    ctx.axis = ax                             # 仍拟合不出就保持 None：反弹改用随机转向
    compute_target()
    ev("marker map n={} axis={}".format(len(ctx.marker_pts), None if ax is None else int(ax)))


async def leave_wall(dist=120):
    """巡线结束后离开边：原地转向场内，再**向前**开走。
    不能直接倒车 —— 巡线时车头大多斜朝场内，倒车会往白线方向走（仿真里轮子因此越线）。"""
    if ctx.axis is not None:
        await rotate(wrap180(ctx.axis + INTO_ARENA * 90.0 - ctx.odom.heading))
    else:
        await rotate(INTO_ARENA * 90)
    await drive_until(config.SEEK_SPEED, dist)


async def calibrate_and_map():
    """终点确认后：场上转角自标定 + 色标定位（一次完成，约 30 s）。

      A0 = 找终点时这条边的方向（锯齿点拟合）；不够就再沿边走一小段；
      离开边 -> 原地转 SELFCAL_SPIN -> 正对这条边开回去 -> 沿边 SELFCAL_FOLLOW_MS（经过色标，同时记录色标点）-> 测 A1。
      矩形场地任何一条边 ≡ A0 (mod 90)：航向误差 err = (A1 - A0) 的 90° 残差 = 转过角度 × 比例误差。
      修正转角比例 k 和当前航向，并把"原地转之后"记录的位置（当前位置、色标点）绕原地转的位置旋转 err：
      err 是在原地转时产生的，之后的位移都按偏了 err 的航向积分。"""
    a0 = straight_run_heading(ctx.edge_trace)
    if a0 is None:
        await follow(config.MAP_MS // 2, False)
        a0 = straight_run_heading(ctx.edge_trace)
    if a0 is None or not config.SELF_CAL:
        await map_marker()
        return
    ctx.axis = a0
    inward = a0 + INTO_ARENA * 90.0
    r0 = ctx.odom.raw
    await rotate(wrap180(inward - ctx.odom.heading))
    r = await drive_until(config.SEEK_SPEED, config.SELFCAL_IN_MM)
    if r != "distance":
        compute_target()
        return
    await rotate(INTO_ARENA * config.SELFCAL_SPIN)
    pivot = (ctx.odom.x, ctx.odom.y)
    await rotate(wrap180(inward + 180.0 - ctx.odom.heading))
    r = await drive_until(config.SEEK_SPEED, config.SELFCAL_IN_MM + 200)
    if r != "white":
        compute_target()
        return
    await straight(-config.SEEK_BACKOFF_MM)
    ctx.edge_trace = []
    ctx.marker_pts = []                      # 色标点重新记录（原地转之后的坐标）
    await follow(config.SELFCAL_FOLLOW_MS, False)
    a1 = straight_run_heading(ctx.edge_trace)
    rot = ctx.odom.raw - r0
    err = None if a1 is None else -axis_snap(a1, a0)
    if err is None or abs(rot) < 300 or abs(err) > config.SELFCAL_MAX_ERR:
        ev("selfcal: skip")
        compute_target()
        return
    k = 1.0 - err / rot
    # DriveBase 航向偏大 err（顺时针）=> 积分出的位移被顺时针转了 err => 数学约定下逆时针转回 err
    ctx.marker_pts = [rotate_about(p, pivot, err) for p in ctx.marker_pts]
    ctx.odom.x, ctx.odom.y = rotate_about((ctx.odom.x, ctx.odom.y), pivot, err)
    ctx.odom.correct_heading(-err)
    ctx.odom.set_turn_scale(ctx.odom.k * k)
    ctx.turn_k = ctx.odom.k
    compute_target()
    ev("selfcal err={:.1f} rot={:.0f} k={:.4f} marker n={}".format(err, rot, ctx.odom.k, len(ctx.marker_pts)))


async def carry_goto(pt, lead=0.0, tol=25.0, max_ms=25000):
    """推送模式下走到 pt：纯追踪（一直前进，每 tick 朝目标方向修正转向率），不原地转。
    lead>0：让车前 lead 处的物体中心到达 pt。
    返回 'arrived'（到达且基本正对）/ 'missed'（到了但没对准或已经走过）/
         'finish'（光斑压到终点色标）/ 'white' / 'stall' / 'lost' / 'timeout'。"""
    ctx.phase = "deliver" if lead else "carry"
    t = 0
    whites = 0
    low = 0
    best = 1e9                                # 追踪过程中离目标最近的剩余距离
    k = config.PURSUIT_GAIN
    lim = config.CARRY_TURN_RATE
    while t < max_ms:
        s = await tick()
        if carry_lost():
            ev("carry lost: last press {} ms ago".format(ctx.now() - ctx.last_press))
            drive.brake()
            return "lost"
        rel, dist = ctx.odom.bearing_to(pt[0], pt[1])
        if lead == 0 and ctx.boundary is not None:
            # Fence the path to the waypoint along the direction we are about
            # to drive; checking the current heading alone aborts a leg that
            # starts beside the marker wall but heads inward.
            if geofence_mm(wrap180(ctx.odom.heading + rel)) <= \
                    config.CARRY_SPEED * PERIOD / 1000:
                drive.brake()
                return "lost"
        if dist - lead < best:
            best = dist - lead
        r = None
        if is_boundary(s):
            whites += 1
            if whites >= config.EDGE_CONFIRM_SAMPLES:
                r = "white"
        else:
            whites = 0
        if r is None:
            if ctx.stall_ms >= config.STALL_MS:
                r = "stall"
            elif dist - lead < tol:
                r = "arrived" if abs(rel) < 30 else "missed"
            elif abs(rel) > 100 and best < 80 and dist - lead > best + 30:
                r = "missed"                  # 已经接近过、又走远了：错过了
            elif ctx.load_base is not None and t > 400:
                if load_sum() < ctx.load_base + config.TAP_MARGIN * 0.5:
                    low += 1
                else:
                    low = 0
                if low >= config.CARRY_LOST_N:
                    r = "lost"
        if r is not None:
            drive.brake()
            return r
        rate = k * rel
        if rate > lim:
            rate = lim
        elif rate < -lim:
            rate = -lim
        drive.drive(config.CARRY_SPEED, rate)
        await wait(PERIOD)
        t += PERIOD
    drive.brake()
    return "timeout"


async def gyro_align(target):
    drive.brake()
    deadline = ctx.now() + 10000
    stable = 0
    while stable < 3:
        await tick()
        if ctx.target is not None:
            error, _ = ctx.odom.bearing_to(*ctx.target)
        else:
            error = wrap180(target - ctx.odom.heading)
        stable = stable + 1 if abs(error) <= 2 else 0
        if stable:
            drive.brake()
        else:
            drive.drive(min(config.CARRY_SPEED, abs(error)), clamp(2*error, -config.CARRY_TURN_RATE, config.CARRY_TURN_RATE))
        if ctx.now() > deadline:
            drive.brake()
            raise Abort("gyro alignment failed")
        await wait(PERIOD)
    drive.brake()


async def deliver():
    """推着物体去终点：先到场内待命点，再从场内正对色标直线推进，让**物体中心**停在目标点。
    返回 'marker' / 'odometry' / 'lost' / 'fail'。"""
    ctx.phase = "deliver"
    ctx.carrying = True
    if ctx.target is not None:
        for attempt in range(2):
            if ctx.waypoint is not None:
                r = await carry_goto(ctx.waypoint, 0.0, 60.0)
                ev("deliver waypoint:", r)
                if r in ("finish", "white", "lost"):
                    # 去待命点途中就压到色标/白边：说明是在边附近推到的物体，
                    # 物体（在光斑前方）很可能已经越过边线 —— 不声称交付，退回搜索
                    return "lost"
                if r not in ("arrived", "missed"):
                    break
            drive.brake()
            rel, _ = ctx.odom.bearing_to(ctx.target[0], ctx.target[1])
            await gyro_align(wrap180(ctx.odom.heading + rel))
            r = await carry_goto(ctx.target, config.OBJ_LEAD_MM, config.DELIVER_TOL_MM)
            ev("deliver target:", r, "left {:.0f}".format(
                ctx.odom.bearing_to(ctx.target[0], ctx.target[1])[1] - config.OBJ_LEAD_MM))
            if r == "white":
                # 光斑先压到白边了：物体中心离目标还有多远？很近就认，不近就退一步再推一次。
                left = ctx.odom.bearing_to(ctx.target[0], ctx.target[1])[1] - config.OBJ_LEAD_MM
                if left <= 45.0:
                    return "odometry"
                await straight(-70.0)
                r = await carry_goto(ctx.target, config.OBJ_LEAD_MM, config.DELIVER_TOL_MM)
                ev("deliver target (retry):", r)
            if r == "finish":
                return "marker"
            if r == "arrived":
                return "odometry"
            if r == "lost":
                return "lost"
            if r != "missed":
                break
    # 不再"推着物体沿边找色标"：仿真里那样做常把物体推出界、轮子越线。
    # 到不了目标就放弃本次交付（不声称成功），回到搜索。
    return "fail"


async def release():
    drive.brake()
    ev3.light.on(Color.GREEN)
    ev3.speaker.beep(1500, 300)
    ctx.carrying = False
    await straight(-config.FINISH_BACK_MM)     # 倒车：V 形推板把物体留在原地


# -------------------------------------------------------------------
# Mission v3：陀螺航向 + 车道扫描 + 触碰捕获
#   【整段仅离线仿真验证，未上实机】见 docs/mission-v3.md
# -------------------------------------------------------------------
def median(values):
    v = sorted(values)
    return v[len(v) // 2] if v else None


def geofence_mm(heading=None):
    """Remaining travel before the plow front would reach an estimated wall.

    Empty search uses known walls only: an unobserved wall is found by the
    color sensor, because stopping at the conservative dimension prior would
    end every lane inside the arena. When carrying, the prior stays in the
    fence so an unobserved wall cannot swallow the object. `heading` lets a
    caller ask about the direction of travel rather than the current heading.
    """
    if not ctx.use_fence or ctx.boundary is None:
        return 1e9
    h = ctx.odom.heading if heading is None else heading
    return ctx.boundary.clearance(ctx.odom.x, ctx.odom.y, h,
                                  config.PLOW_FRONT_MM,
                                  known_only=not ctx.carrying) - config.GEOFENCE_MARGIN_MM


def fence_reached(speed):
    """One tick of travel or less left before the plow front meets a wall."""
    return geofence_mm() <= speed * PERIOD / 1000.0


async def sweep_lane(target, carry, limit=None, speed=None):
    """沿一条车道保持航向直行。返回 (结果, 里程)：
    'white' 光斑到白边 | 'geofence' 推送围栏 | 'far' 车道长度上限 |
    'capture' 触碰捕获 | 'lost' 推送中物体丢了 | 'stall' 堵转。"""
    d0 = drive.distance()
    local = limit is not None
    limit = config.LANE_MAX_MM if limit is None else limit
    speed = config.SWEEP_SPEED if speed is None else speed
    while True:
        s = await tick()
        if ctx.captured and not ctx.carrying:
            drive.brake()
            return "capture", abs(drive.distance() - d0)
        if carry and carry_lost():
            drive.brake()
            return "lost", abs(drive.distance() - d0)
        d = abs(drive.distance() - d0)
        if is_boundary(s):
            drive.brake()
            if local:
                return "white", d
            await straight(-max(config.BACKOFF_MM, config.AXLE_TRACK_MM/2+30))
            return ("end" if d < config.LANE_END_MM else "white"), d
        if ctx.stall_ms >= config.STALL_MS:
            drive.brake()
            if not local:
                await unstall()
            return "stall", d
        if fence_reached(speed):
            drive.brake()
            return "geofence", d
        if d >= limit:
            drive.brake()
            return "far", d
        hold_heading(speed, target)
        await wait(PERIOD)


async def move_along_edge(dist, heading):
    """Parallel motion only off-white; stop on ANY white or plow geofence."""
    d0 = drive.distance()
    while abs(drive.distance() - d0) < dist:
        s = await tick()
        if ctx.captured and not ctx.carrying:
            drive.brake()
            return "capture"
        if ctx.carrying and carry_lost():
            drive.brake()
            return "lost"
        if ctx.stall_ms >= config.STALL_MS:
            drive.brake()
            await unstall()
            return "stall"
        if is_boundary(s) or fence_reached(config.SWEEP_SPEED):
            drive.brake()
            return "end"
        hold_heading(config.SWEEP_SPEED, heading)
        await wait(PERIOD)
    drive.brake()
    return "ok"


def _fit_line(pts):
    """最小二乘拟合一条直线，返回 (航向, 残差 rms, 跨度) 或 None。"""
    n = len(pts)
    if n < 3:
        return None
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = syy = sxy = 0.0
    for p in pts:
        dx, dy = p[0] - mx, p[1] - my
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    if sxx + syy < 40.0 * 40.0:
        return None
    ang = 0.5 * atan2(2.0 * sxy, sxx - syy)
    ux, uy = cos(ang), sin(ang)
    if ux * (pts[-1][0] - pts[0][0]) + uy * (pts[-1][1] - pts[0][1]) < 0.0:
        ux, uy = -ux, -uy                # 方向取旧 -> 新（与行驶方向一致）
    rms = 0.0
    for p in pts:
        d = -(p[0] - mx) * uy + (p[1] - my) * ux     # 到拟合直线的垂距
        rms += d * d
    rms = (rms / n) ** 0.5
    span = abs((pts[-1][0] - pts[0][0]) * ux + (pts[-1][1] - pts[0][1]) * uy)
    return wrap180(heading_of(ux, uy)), rms, span


def wall_fit(min_pts=None, min_span=None, max_rms=6.0, pts=None):
    """用「压上白带的那些点」拟合墙线。返回 (航向, rms, 点数) 或 None。

    只取最近一段（从新点往回试后缀）：转角处会混进另一条墙的点，整段拟合会被带偏，
    从最新的点往回找第一段「真的在一条直线上」的即可。
    """
    if min_pts is None:
        min_pts = 3                 # 三点才能定一条线（edge_probe 每条边只采 EDGE_PROBE_N 个点）
    if min_span is None:
        min_span = 120.0
    pts = ctx.white_hits if pts is None else pts
    n = len(pts)
    if n < min_pts:
        return None
    for s in range(n - min_pts, -1, -1):
        sub = pts[s:]
        f = _fit_line(sub)
        if f is None:
            continue
        h, rms, span = f
        if rms <= max_rms and span >= min_span:
            return h, rms, len(sub)
    return None


def inward_dir(wall):
    """在 wall 的两个法向里挑朝场内那个：路径质心相对最近白点的方向。

    机器人每次都是"从场内撞上白带再退回来"，所以沿边的历史位置整体在场内一侧，
    质心 - 最近白点 就指向场内，投影到法向上取符号。
    """
    n = len(ctx.path)
    if n < 3 or ctx.last_white is None:
        return wrap180(wall + 90.0 * INTO_ARENA)
    cx = sum(p[0] for p in ctx.path) / n
    cy = sum(p[1] for p in ctx.path) / n
    vx = cx - ctx.last_white[0]
    vy = cy - ctx.last_white[1]
    best = None
    for cand in (wrap180(wall + 90.0), wrap180(wall - 90.0)):
        rad = -cand * pi / 180.0
        d = vx * cos(rad) + vy * sin(rad)
        if best is None or d > best[1]:
            best = (cand, d)
    return best[0]


async def lane_sweep(max_lanes, carry=False, adv_sign=1.0, want_marker=False):
    """从当前（第一条边的转角）出发做 boustrophedon 车道扫描。返回：
    'capture' / 'marker'（推送中扫到色标）/ 'covered'（沿边走到头）/ 'exhausted' / 'time'。

    几何（转角处）：墙线方向用「压上白带的那些点」拟合（它们都在同一条墙线上），
    车道走墙线的法向（朝场内），相邻车道沿墙线平移 LANE_SPACING_MM。
    转角正好是两条边的交线，直接开第一条车道会全程压在转角后那条边的白带上，所以要
    先沿墙线让开半个车道（也顺便退出白带）。
    朝场内那一侧用「路径质心相对最近白点」的方向判定，不依赖转角/边序号的符号假设。
    """
    ctx.phase = "lane"
    if ctx.axis is None:
        ctx.axis = wrap180(ctx.odom.heading)
    wall = ctx.axis
    lane_dir = inward_dir(wall)
    advance = wall if adv_sign > 0 else wrap180(wall + 180.0)
    ctx.lane_dir = lane_dir
    ev("lane frame: wall={:.0f} lane={:.0f} adv={:.0f} hits={}".format(
        wall, lane_dir, advance, len(ctx.white_hits)))
    turn90 = wrap180(advance - lane_dir)
    if not carry:
        await straight(-config.BACKOFF_MM)               # 从白带里退出来
    # 先沿车道方向彻底离开白带（最多 LANE_CLEAR_MM）；还是白的说明这个方向朝场外，
    # 掉头再试一次 —— 转角上超调时车常常已经跑到白带外面了。
    for attempt in range(2):
        await turn_to(lane_dir)
        d0 = drive.distance()
        clean = 0
        while abs(drive.distance() - d0) < config.LANE_CLEAR_MM:
            s = await tick()
            clean = 0 if is_boundary(s) else clean + 1
            if clean >= config.EDGE_CONFIRM_SAMPLES:
                break
            hold_heading(config.SWEEP_SPEED, lane_dir)
        drive.brake()
        if clean >= config.EDGE_CONFIRM_SAMPLES:
            break
        ev("lane start: still on white, flip (try {})".format(attempt + 1))
        lane_dir = wrap180(lane_dir + 180.0)
        turn90 = wrap180(advance - lane_dir)
        ctx.lane_dir = lane_dir
    await turn_to(advance)
    await move_along_edge(config.LANE_SPACING_MM / 2.0, advance)
    await turn_to(lane_dir)
    dirn = 1.0
    # 扫掠必须能自己停下来：沿墙方向的长度最多是场地长边（先验 [TBD]），
    # 而且连续两条车道一次白都没见到就说明已经扫出场地了。
    max_adv = max(config.ARENA_INTERIOR_W_MM, config.ARENA_INTERIOR_H_MM)
    cap = int(max_adv / config.LANE_SPACING_MM) + 1
    if max_lanes > cap:
        max_lanes = cap
    adv = 0.0
    far_run = 0
    for k in range(max_lanes):
        target = lane_dir if dirn > 0 else wrap180(lane_dir + 180.0)
        r, d = await sweep_lane(target, carry)
        ctx.lane_log.append(("+" if dirn > 0 else "-", r, int(d)))
        ev("lane", k, ("+" if dirn > 0 else "-"), r, int(d),
           "at ({:.0f},{:.0f})".format(ctx.odom.x, ctx.odom.y))
        if r == "capture":
            return "capture"
        if r == "lost":
            # 运送中丢了物体：倒车、放弃这次携带，然后照常平移去下一条车道继续扫。
            # （不能只翻方向就 continue —— 那样 adv 不增加，车会在原地来回打圈到超时。）
            ev("carry lost at lane {} -> drop and keep sweeping".format(k))
            await straight(-config.BACKOFF_MM)
            ctx.carrying = False
            ctx.captured = False
            ctx.pressed_n = 0
        if r == "end":
            ev("sweep: lane {} starts on white -> this wall ends here".format(k))
            ctx.sweep_adv_dir = advance
            ctx.sweep_adv_mm = adv
            return "covered"
        if r == "white":
            far_run = 0
            await straight(-config.BACKOFF_MM)     # 运送中见白：立即刹住并倒车
        elif r != "lost":
            far_run += 1 if r == "far" else 0
            if far_run >= 1:
                # 车道从场内出发必然在 LANE_MAX_MM 之内撞到对面的白带；一次白都没见到
                # 只可能是"朝场外开"，立刻收手（否则一条车道就开出去 1.6 m）。
                ev("sweep: no white in lane {} -> heading out of the arena".format(k))
                ctx.sweep_adv_dir = advance
                ctx.sweep_adv_mm = adv
                return "covered"
            await straight(-config.BACKOFF_MM)
        if (carry or want_marker) and marker_seen():
            return "marker"
        # 沿墙平移一个车道宽度。必须显式转向 advance 方向再走：车道方向每轮翻转，
        # 用固定的 ±90 转向会在相邻两条车道之间来回走（原地打圈）。
        await turn_to(advance)
        r2 = await move_along_edge(config.LANE_SPACING_MM, advance)
        if r2 == "capture":
            return "capture"
        if r2 == "end":
            ev("sweep: corner crossed while advancing after lane {}".format(k))
            ctx.sweep_adv_dir = advance
            ctx.sweep_adv_mm = adv
            return "covered"
        if r2 == "covered":
            ev("sweep covered after", k, "lanes")
            return "covered"
        if r2 == "lost":
            ev("lost while moving along edge")
        dirn = -dirn
        nxt = lane_dir if dirn > 0 else wrap180(lane_dir + 180.0)
        await turn_to(nxt)
        adv += config.LANE_SPACING_MM
        if adv >= max_adv:
            ev("sweep: advance {:.0f}mm reached the arena extent".format(adv))
            ctx.sweep_adv_dir = advance
            ctx.sweep_adv_mm = adv
            return "covered"
    ctx.sweep_adv_dir = advance
    ctx.sweep_adv_mm = adv
    return "exhausted"


async def go_capture(pt):
    """走到记下的物体位置，重新把轮胎收进推板。返回 'captured' / 'failed'。

    捕获时如果还不知道色标在哪，就把物体记在里程计坐标里、退回场内继续扫色标；
    等色标找到了再回来按记下的位置重新收一次 —— 比"推着物体满场找色标"稳：
    推着物体转弯时轮胎很容易从 V 口掉出来（实测每条车道都 lost）。
    """
    ctx.captured = False
    ctx.pressed_n = 0
    for attempt in range(3):
        rel, dist = ctx.odom.bearing_to(pt[0], pt[1])
        await turn_to(wrap180(ctx.odom.heading + rel))
        r = await drive_until(config.SWEEP_SPEED, max(150.0, dist + 100.0))
        if ctx.captured:
            ev("re-capture at attempt", attempt, "dist {:.0f}".format(dist))
            return "captured"
        ev("re-capture", attempt, "->", r, "dist {:.0f}".format(dist))
        await straight(-60.0)
        await rotate(20.0 if attempt % 2 == 0 else -20.0)
    return "failed"


async def return_arc(stage, heading):
    """Reach a staging pose with forward motion and the live plow fence."""
    deadline = ctx.now() + 30000
    while ctx.now() < deadline:
        sample = await tick()
        error, dist = ctx.odom.bearing_to(*stage)
        aim_error = wrap180(heading - ctx.odom.heading)
        if dist <= 40 and abs(aim_error) <= 12:
            drive.brake()
            return True
        if is_boundary(sample) or ctx.stall_ms >= config.STALL_MS or ctx.captured:
            break
        # Approach the staging line from behind. Within one turn radius,
        # blend toward its final heading rather than orbiting the waypoint.
        if dist < 70 and abs(aim_error) < 65:
            error = aim_error
        rate = clamp(2 * error, -config.CARRY_TURN_RATE, config.CARRY_TURN_RATE)
        speed = min(config.SWEEP_SPEED, max(9, 38 - abs(error) * 0.35))
        if fence_reached(speed) or geofence_mm(ctx.odom.heading + rate * PERIOD / 1000) <= speed * PERIOD / 1000:
            break
        drive.drive(speed, rate)
        await wait(PERIOD)
    drive.brake()
    return False


async def recover_parked(point, heading):
    """Try the remembered line, then adjacent search lanes around the tyre."""
    for offset in (0, -config.LANE_SPACING_MM, config.LANE_SPACING_MM):
        ctx.phase = "reacquire" if offset == 0 else "reacquire_sweep"
        ctx.captured = False
        ctx.pressed_n = 0
        target = ahead(*point, heading, 0, offset)
        stage = ahead(*target, heading, -250)
        # Line up using a lead-in to the staging point. It is optional:
        # there may be no room for it next to a wall.
        lead_in = ahead(*stage, heading, -100)
        if geofence_mm(heading) > 100:
            await return_arc(lead_in, heading)
        staged = not ctx.captured and await return_arc(stage, heading)
        if staged:
            # Sweep along this line no farther than 200 mm beyond the tyre.
            end = ahead(*target, heading, 200)
            remaining = dot((end[0] - ctx.odom.x, end[1] - ctx.odom.y),
                            ahead(0, 0, heading, 1))
            await sweep_lane(heading, False, limit=remaining,
                             speed=min(35, config.SWEEP_SPEED))
        if ctx.captured and not fence_reached(config.CARRY_SPEED) and await seat_ok(geofence=True):
            ev("parked tyre recaptured", offset)
            return True
        ctx.captured = False
        ctx.pressed_n = 0
    ev("parked tyre not recovered, resume normal lane sweep")
    return False


async def measure_marker():
    """Measure the paper's along-wall extent and inward edge with local probes.

    A scan hit only gives one point.  Probe perpendicular to the wall at small
    along-wall steps: red means the probe line crosses the paper, white means it
    lands beside it.  Bisect each edge, then keep the red onsets (the paper's
    inner edge) and the measured along interval.
    """
    ctx.phase = "marker_probe"
    w = marker_wall()
    if w is None or not ctx.marker_pts:
        ev("marker measure: no wall estimate")
        return
    vi, va = w["v_in"], w["v_along"]
    line_n = w["line_n"]
    ev("marker wall k={} b={:.0f} line_n={:.0f} bounds={}".format(
        w["k"], w["b"], line_n, ctx.boundary.bounds))
    face = wrap180(w["inward_heading"] + 180.0)
    along_h = w["along_heading"]
    clear = max(80.0, config.MARKER_SIDE_MM * 0.8 + 40.0)
    step = config.MARKER_PROBE_STEP_MM
    max_steps = int((config.MARKER_SIDE_MM * 1.5 + 100.0) / step) + 2
    a_cur = None
    rows = []
    inner = []

    async def probe_at(a_target):
        nonlocal a_cur
        if a_cur is None:
            await turn_to(face)
            n_cur = dot((ctx.odom.x, ctx.odom.y), vi) - line_n
            await straight(-(clear - n_cur))
            a_cur = dot((ctx.odom.x, ctx.odom.y), va)
        delta = a_target - a_cur
        if abs(delta) > 1.0:
            await turn_to(along_h)
            await straight(delta)
            await turn_to(face)
        a_cur = a_target
        n_cur = dot((ctx.odom.x, ctx.odom.y), vi) - line_n
        if abs(n_cur - clear) > 25.0:
            await straight(-(clear - n_cur))
        a_act = dot((ctx.odom.x, ctx.odom.y), va)
        ctx.boundary_kind = None
        # Limit the axle to the estimated line, even if colour is missed.
        n_cur = dot((ctx.odom.x, ctx.odom.y), vi) - line_n
        d0 = drive.distance()
        r = await drive_until(config.SWEEP_SPEED, max(0, n_cur - config.SENSOR_OFFSET_MM))
        kind = ctx.boundary_kind if r in ("white", "capture") else None
        if kind == "red" and ctx.boundary_hits:
            inner.append(dot(ctx.boundary_hits[-1][0], vi) - line_n)
        await straight(-max(drive.distance() - d0, config.AXLE_TRACK_MM/2 + 30))
        rows.append((a_act, kind))
        return kind, a_act

    a0 = sum(dot(p, va) for p in ctx.marker_pts) / len(ctx.marker_pts)
    kind, a_act = await probe_at(a0)
    if kind != "red":
        for off in (step, -step, 2*step, -2*step):
            kind, a_act = await probe_at(a0 + off)
            if kind == "red":
                break
    if kind != "red":
        ev("marker measure: no red at hit")
        return
    centre = a_act
    for direction in (1.0, -1.0):
        a = centre
        miss = 0
        for _ in range(max_steps):
            a += direction * step
            kind, a_act = await probe_at(a)
            if kind == "red":
                miss = 0
            else:
                miss += 1
                if miss >= 2:
                    break
    reds = [a for a, k in rows if k == "red"]
    whites = [a for a, k in rows if k == "white"]
    if not reds:
        ev("marker measure: no red probes")
        return
    a_lo, a_hi = min(reds), max(reds)

    async def bisect(lo, hi, red_at_hi):
        for _ in range(3):
            mid = (lo + hi) / 2.0
            kind, a_act = await probe_at(mid)
            if (kind == "red") == red_at_hi:
                hi = a_act
            else:
                lo = a_act
        return (lo + hi) / 2.0

    left_out = [a for a in whites if a < a_lo]
    right_out = [a for a in whites if a > a_hi]
    left_edge = await bisect(max(left_out), a_lo, True) if left_out else a_lo
    right_edge = await bisect(a_hi, min(right_out), False) if right_out else a_hi
    n_inner = max(inner) if inner else None
    ctx.marker_along_center = (left_edge + right_edge) / 2.0
    ctx.marker_extent_mm = right_edge - left_edge
    ctx.marker_inner_mm = n_inner
    ev("marker measured: along {:.0f} extent {:.0f} inner {} (probes {})".format(
        ctx.marker_along_center, ctx.marker_extent_mm,
        "none" if n_inner is None else "{:.0f}".format(n_inner), len(rows)))


def park_point():
    """A safe interior point (centre of the estimated boundary rectangle)."""
    if ctx.boundary is None:
        return None
    u0, u1 = ctx.boundary.limits(0)
    v0, v1 = ctx.boundary.limits(1)
    u = (u0 + u1) / 2.0
    v = (v0 + v1) / 2.0
    ax0, ax1 = ctx.boundary.axes
    return (u*ax0[0] + v*ax1[0], u*ax0[1] + v*ax1[1])


async def marker_search(turn_away=True):
    """Find the colour marker while the object is parked elsewhere.

    The plow fence keeps the sensor ~140 mm inside a wall, but the marker is a
    60 mm strip against the wall, so a fence-limited lane never sees it. With no
    object in front the plow may cross the tape, so disable the fence and first
    walk the perimeter at ~60 mm standoff; only if that misses, sweep both arena
    orientations as a fallback.
    """
    initial_axis = ctx.axis
    # Turn away from the parked object: the nearest wall in the carry heading
    # may lie beyond it, and driving to that wall would shove it out.
    if turn_away and ctx.object_pt is not None:
        await rotate(180)
    ctx.use_fence = False
    try:
        ctx.captured = False
        ctx.pressed_n = 0
        r = await edge_walk(config.EDGE_WALK_MS, step_mm=config.MARKER_SCAN_STEP_MM,
                            backoff_mm=config.MARKER_SCAN_BACKOFF_MM,
                            fence=False, want_marker=True)
        ev("marker perimeter scan:", r, "marker", marker_center())
        if marker_seen():
            await measure_marker()
            return "marker"
        for turn in range(2):
            ctx.axis = wrap180(initial_axis + 90.0 * turn)
            for sign in (1.0, -1.0):
                ctx.captured = False
                ctx.pressed_n = 0
                r = await lane_sweep(config.LANE_MAX_COUNT, False, sign, want_marker=True)
                ev("marker search:", turn, sign, r, "marker", marker_center())
                if marker_seen():
                    await measure_marker()
                    return "marker"
    finally:
        ctx.use_fence = True
        ctx.axis = initial_axis
        ctx.scan_wall_k = None
    return "none"


async def seat_ok(ms=400, geofence=False):
    """Creep forward and require the touch to stay pressed: a graze is not a capture."""
    for _ in range(int(ms / PERIOD)):
        sample = await tick()
        if (geofence and (is_boundary(sample) or fence_reached(config.CARRY_SPEED))) or not await read_touch():
            drive.brake()
            return False
        drive.drive(config.CARRY_SPEED, 0)
        await wait(PERIOD)
    drive.brake()
    return True


async def after_capture(mode):
    """Confirm a normal capture, then find the marker and deliver."""
    ctx.captured = True
    if mode == "capture":
        ev3.speaker.beep(900, 200)
        return "CAPTURED (touch, unverified)"
    if not await seat_ok():
        ev("capture did not stay seated")
        ctx.captured = False
        ctx.pressed_n = 0
        return "retry"
    if not marker_seen():
        # Carry the object to a safe interior point first: the close boundary
        # scan would otherwise shove a parked object through the paper/tape.
        ev("marker unknown -> park the object in the middle")
        pt = park_point()
        if pt is not None:
            ctx.carrying = True
            rp = await carry_goto(pt, config.OBJ_LEAD_MM, 60.0)
            ev("park carry:", rp, "at ({:.0f},{:.0f})".format(ctx.odom.x, ctx.odom.y))
            if rp in ("lost", "white"):
                ctx.carrying = False
                ctx.captured = False
                ctx.pressed_n = 0
                return "retry"
            ctx.object_pt = ahead(ctx.odom.x, ctx.odom.y, ctx.odom.heading, config.OBJ_LEAD_MM)
        else:
            ctx.object_pt = ahead(ctx.odom.x, ctx.odom.y, ctx.odom.heading, config.OBJ_LEAD_MM)
        ctx.carrying = False
        ctx.captured = False
        ctx.pressed_n = 0
        await straight(-config.BACKOFF_MM)
        r = await marker_search()
        if not marker_seen():
            return "END captured but marker unknown (" + r + ")"
        how = await go_capture(ctx.object_pt)
        ev("re-capture:", how)
        if how != "captured" or not await seat_ok():
            ctx.captured = False
            ctx.carrying = False
            ctx.pressed_n = 0
            return "retry"
    return await deliver_capture()


async def deliver_capture():
    compute_target()
    if ctx.target is None:
        return "END captured, no target"
    ev("target", ctx.target, "marker", marker_center())
    how = await deliver()
    ev("deliver:", how)
    if how in ("lost", "fail"):
        if how == "lost":
            ctx.lost_objects += 1
        ctx.carrying = False
        ctx.captured = False
        ctx.pressed_n = 0
        await straight(-config.BACKOFF_MM)
        return "retry"
    await release()
    # 机器人看不见物体：这是"按计划完成交付"，不是"确认物体落在色标上"
    return "DELIVERED (unverified, check by eye)"


async def edge_walk(max_ms, step_mm=None, backoff_mm=None, fence=True, want_marker=False):
    """Map perimeter with normal approaches and backed-off parallel moves.

    Each leg stops at the plow geofence, or at white when fence=False. A touched
    object is parked and left. backoff_mm controls the standoff for the parallel
    leg; the marker scan uses a small one so the spot passes over the marker band.
    """
    ctx.phase = "marker_scan" if want_marker else "edge_walk"
    step = config.EDGE_WALK_MM if step_mm is None else step_mm
    face = min((wrap180(ctx.axis + 90*k) for k in range(4)),
               key=lambda h: abs(wrap180(h-ctx.odom.heading)))
    deadline = ctx.now() + max_ms
    corners = 0
    if backoff_mm is None:
        backoff = max(config.BACKOFF_MM, config.AXLE_TRACK_MM / 2 + 30)
    else:
        backoff = backoff_mm
    while ctx.now() < deadline and corners < 5 and not (
            want_marker and marker_seen()):
        if ctx.boundary is not None:
            ctx.scan_wall_k = ctx.boundary.axis_index(face)
        await turn_to(face)
        r = await drive_until(config.SWEEP_SPEED, config.EDGE_PROBE_MM, geofence=fence)
        ev("edge walk: face {:.0f} {}".format(face, r))
        if want_marker and marker_seen():
            return "marker"
        if r == "capture":
            # A touched object is the latest known object position, even while
            # only looking for the marker (the parked one may have been nudged).
            ctx.object_pt = ahead(ctx.odom.x, ctx.odom.y, ctx.odom.heading, config.OBJ_LEAD_MM)
            ctx.captured = False
            ctx.pressed_n = 0
        elif r not in ("white", "geofence"):
            return r
        await straight(-backoff)
        along = wrap180(face + 90)
        await turn_to(along)
        r = await drive_until(config.SWEEP_SPEED, step, geofence=fence)
        ev("edge walk: along {:.0f} {}".format(along, r))
        if want_marker and marker_seen():
            return "marker"
        if r == "capture":
            ctx.object_pt = ahead(ctx.odom.x, ctx.odom.y, ctx.odom.heading, config.OBJ_LEAD_MM)
            ctx.captured = False
            ctx.pressed_n = 0
            await straight(-config.BACKOFF_MM)
        if r in ("white", "geofence"):
            await straight(-backoff)
            face = along
            corners += 1
            ev("edge walk corner", corners, r)
    return "mapped" if corners >= 5 else "time"


async def edge_probe(n=3):
    """在离边 BACKOFF 的场内侧采三个白带点，量出这条边的方向。

    不能靠"沿边巡线 + 航向平均"：光斑只前伸 20 mm、半轮距 57 mm，贴边巡线时内侧轮心
    必然越出白带（仿真实测 56 mm，等于半辆车掉到场外）。

    做法：1) 直行撞白 -> P1；2) 倒车后转 EDGE_PROBE_TURN_DEG 再撞 -> P2；3) 沿边平移 SHIFT
    再转回垂直撞 -> P3。三点拟合。试过"全部垂直撞"（平移代替转向）：第二撞的平移方向由
    h_hit 推出来，h_hit 不垂直时第二次撞的点就不在同一条边上，定出来的轴反而更差
    （实测 8 seeds 只有 1 个成功），所以保留斜撞那一撞。
    """
    ctx.phase = "edge_probe"
    ctx.white_hits = []          # 只要这次采样的点：之前寻边阶段的点是别的边/乱转时采的
    r = await drive_until(config.SWEEP_SPEED, config.EDGE_PROBE_MM)
    if r != "white":
        ev("edge probe 0 ->", r)
        return r
    h_hit = wrap180(ctx.odom.heading)
    await straight(-config.EDGE_PROBE_BACKOFF_MM)
    if n >= 2:
        await rotate(config.EDGE_PROBE_TURN_DEG)
        r = await drive_until(config.SWEEP_SPEED, config.EDGE_PROBE_MM)
        if r != "white":
            ev("edge probe 1 ->", r)
            return r
        await straight(-config.EDGE_PROBE_BACKOFF_MM)
    if n >= 3 and len(ctx.white_hits) >= 2:
        a, b = ctx.white_hits[-2], ctx.white_hits[-1]
        edge = wrap180(heading_of(b[0] - a[0], b[1] - a[1]))
        if abs(wrap180(edge - h_hit)) > 90.0:
            edge = wrap180(edge + 180.0)
        await turn_to(edge)
        await drive_until(config.SWEEP_SPEED, config.EDGE_PROBE_SHIFT_MM)
        # 转回"朝白带"的方向：edge 的两个垂直方向里更接近 h_hit 的那个
        c1 = wrap180(edge + 90.0)
        c2 = wrap180(edge - 90.0)
        face = c1 if abs(wrap180(c1 - h_hit)) < abs(wrap180(c2 - h_hit)) else c2
        await turn_to(face)
        r = await drive_until(config.SWEEP_SPEED, config.EDGE_PROBE_MM)
        if r != "white":
            ev("edge probe 2 ->", r)
            return r
        await straight(-config.EDGE_PROBE_BACKOFF_MM)
    return "white"


async def run_v3(mode):
    """v3 主流程：起步 -> 寻边 -> 采样定场地轴向 -> 车道扫描 -> 触碰捕获 -> 交付。

    不做"转角检测"（follower 在刚上边时会乱转十几秒，任何航向突变都不能当转角），
    也不贴边巡线（光斑 20 mm / 半轮距 57 mm 的几何下内侧轮必然越出白带）。
    改为在边上采样三个点拟合出这条边的方向，然后从边上起扫；
    正向扫到底再反向扫一遍，补上起点身后那条还没扫到的窄条。
    """
    if not ctx.sense.marker_enabled:
        ev("WARN: no marker sensing (SENSE_MODE reflection)")
    ctx.phase = "seek"
    await start_check()
    parked = None
    while True:
        r = await seek_edge(False)
        if r == "edge":
            break
        if r == "capture":
            if mode != "full":
                return "CAPTURED (touch, unverified)"
            if not ctx.sense.marker_enabled:
                return "FAIL full needs markers (SENSE_MODE rgb + SIGNATURES)"
            # During initial seek no usable boundary model exists yet.
            parked = await park_early_capture()
            continue
        if r == "finish":
            return "END finish before edge"
        ev("seek retry with a new heading")
        await rotate(random.randint(90, 180) * rand_sign())
    if mode == "full" and not ctx.sense.marker_enabled:
        return "FAIL full needs markers (SENSE_MODE rgb + SIGNATURES)"
    r = await edge_probe(config.EDGE_PROBE_N)
    ev("edge probe:", r, "hits", len(ctx.white_hits))
    if ctx.captured and mode == "capture":
        return await after_capture(mode)
    if ctx.captured:
        ctx.object_pt = ahead(ctx.odom.x, ctx.odom.y, ctx.odom.heading, config.OBJ_LEAD_MM)
        ctx.captured = False
        ctx.pressed_n = 0
        await straight(-config.BACKOFF_MM)
    face = ctx.boundary_hits[-1][1] if ctx.boundary_hits else ctx.odom.heading
    fit = wall_fit(min_pts=3, min_span=60.0, pts=ctx.white_hits[-config.EDGE_PROBE_N - 1:])
    if fit is not None:
        ctx.axis = orient_axis(fit[0], face)
        ev("axis from probe: {:.0f} rms={:.1f} pts={}".format(ctx.axis, fit[1], fit[2]))
    else:
        # Near a corner the probe can hit the perpendicular wall, so a line fit
        # fails. Cluster the hits along candidate arena axes instead.
        axis = fit_axis([p for p, _, _ in ctx.boundary_hits[-12:]], face)
        if axis is not None:
            ctx.axis = axis
            ev("axis from clusters: {:.0f} (probe weak, pts={})".format(
                ctx.axis, len(ctx.white_hits)))
        else:
            ctx.axis = wrap180(face + 90.0)
            ev("axis from heading: {:.0f} (probe weak, pts={})".format(
                ctx.axis, len(ctx.white_hits)))
        f3 = _fit_line(ctx.white_hits[-3:]) if len(ctx.white_hits) >= 3 else None
        for i, p in enumerate(ctx.white_hits[-4:]):
            ev("  hit", i, "({:.0f},{:.0f})".format(p[0], p[1]))
        if f3 is not None:
            ev("  last fit rms={:.1f} span={:.0f}".format(f3[1], f3[2]))
    ctx.boundary = Boundary(ctx.axis, config.ARENA_INTERIOR_W_MM, config.ARENA_INTERIOR_H_MM)
    for point, heading, _kind in ctx.boundary_hits:
        ctx.boundary.observe(point, heading)
    ev("boundary estimates", ctx.boundary.bounds)
    ev("sweep frame: wall={:.0f} hits={}".format(ctx.axis, len(ctx.white_hits)))
    if mode == "sweep":
        n = await lane_sweep(config.SWEEP_PREVIEW_LANES, False, 1.0)
        compute_target()
        ev("sweep:", n, "marker", marker_center(), "axis", ctx.axis)
        return "END sweep {} marker={} lanes={}".format(
            n, marker_center(), len(ctx.lane_log))
    if parked is not None:
        point, heading = parked
        # The normal probe may have passed the parked tyre again.
        if ctx.object_pt is None:
            ctx.object_pt = point
        if not marker_seen():
            await marker_search(turn_away=False)
        if marker_seen():
            if await recover_parked(ctx.object_pt, heading):
                result = await deliver_capture()
                if result != "retry":
                    return result
        else:
            ev("parked tyre: marker not yet found, resume sweep")
    # No perimeter walk before the object search: it would shove an interior
    # object against a wall before we ever try to capture it. The marker gets
    # its own close perimeter scan after a capture.
    empty_sweeps = 0
    initial_axis = ctx.axis
    while empty_sweeps < 3:
        ctx.axis = wrap180(initial_axis + 90 * (empty_sweeps % 2))
        captured = False
        for sign in (1.0, -1.0):
            r = await lane_sweep(config.LANE_MAX_COUNT, False, sign)
            if r == "capture":
                result = await after_capture(mode)
                if result != "retry":
                    return result
                captured = True
                empty_sweeps = 0
                break
            if sign > 0 and ctx.sweep_adv_mm >= config.LANE_SPACING_MM:
                back = wrap180(ctx.sweep_adv_dir + 180)
                await turn_to(back)
                rr = await move_along_edge(ctx.sweep_adv_mm, back)
                if rr == "capture":
                    result = await after_capture(mode)
                    if result != "retry":
                        return result
                    captured = True
                    empty_sweeps = 0
                    break
        if not captured:
            empty_sweeps += 1
            ev("empty sweep", empty_sweeps, "rotate lanes 90 degrees")
    ev("object not found")
    return "END object not found"



# -------------------------------------------------------------------
async def run_mode(mode):
    if mode == "edge_to_finish":
        if not ctx.sense.marker_enabled:
            ev("no marker -> follow only")
        r = await edge_then_follow(config.FOLLOW_MAX_MS, ctx.sense.marker_enabled)
        if r == "finish":
            drive.brake()
            ev3.speaker.beep(1500, 300)
            return "PASS finish"
        return "END " + r

    if mode == "survey":
        r = await edge_then_follow(config.FOLLOW_MAX_MS, False)
        return "END survey " + r

    if mode == "calib":
        # 只验证"找终点 -> 色标定位 -> 场上转角自标定"，不需要物体
        if not ctx.sense.marker_enabled:
            return "FAIL calib needs markers (SENSE_MODE rgb + SIGNATURES)"
        r = await edge_then_follow(config.FOLLOW_MAX_MS, True)
        if r != "finish":
            return "FAIL finish not found: " + r
        await calibrate_and_map()
        return "END calib turn_k={} axis={}".format(ctx.turn_k, ctx.axis)

    if mode == "capture":
        if not config.USE_LOAD_CAPTURE:
            return await run_v3("capture")          # v3：扫到触碰即停
        # 旧路径：搜索 + 负载嫌疑 + tap 确认（不放物体时用来测误报率）
        await start_check()
        r = await bounce()
        return "END capture: " + ("CONFIRMED" if r == "capture" else "none")

    if mode == "sweep":
        return await run_v3("sweep")

    if mode == "full":
        if not (config.USE_LOAD_CAPTURE or config.USE_BOUNCE_SEARCH or config.USE_SELF_CAL):
            return await run_v3("full")             # v3：陀螺航向 + 车道扫描 + 触碰捕获
        if not ctx.sense.marker_enabled:
            return "FAIL full needs markers (SENSE_MODE rgb + SIGNATURES)"
        r = await edge_then_follow(config.FOLLOW_MAX_MS, True)
        if r != "finish":
            return "FAIL finish not found: " + r
        ev("finish recorded")
        await calibrate_and_map()
        drive.brake()
        await leave_wall()
        await turn_for_next_leg()
        while True:
            r = await bounce()
            if r != "capture":
                return "END search done, no capture"
            how = await deliver()
            ev("deliver:", how)
            if how == "lost":
                # 推送途中物体丢了：多半就在附近 —— 退出推送状态，回到搜索
                ctx.lost_objects += 1
                ctx.carrying = False
                await straight(-config.TAP_BACK_MM)
                await turn_for_next_leg()
                continue
            if how != "fail":
                await release()
                # 机器人无法看到物体：这是"按计划完成交付"，不是"确认物体在色标上"
                return "DELIVERED via " + how + " (unverified)"
            ctx.carrying = False
            await leave_wall()                 # 交付失败多半停在边上（沿边找过），不能倒车
            await turn_for_next_leg()

    return "FAIL unknown mode " + mode


async def main():
    global ctx
    ev3.screen.clear()
    log(ev3, "=== mission ===")
    log(ev3, config.MISSION_MODE)
    battery_check(ev3, config.LOW_BATTERY_MV)
    try:
        sense = Sensing(color)
    except ValueError as e:
        log(ev3, "config error:", repr(e))
        return
    print("sensing:", sense.describe())

    await wait_for_button(ev3, prompt="RIGHT to start")
    ev3.light.on(Color.GREEN)
    ctx = Ctx(sense)
    # v3 的三个模式用陀螺航向 + 触碰捕获；edge_to_finish / survey / calib 是已实机验证的
    # 老路径，保持 drive.angle()：每个 tick 多读两次传感器在「慢读」场景会把任务拖超时。
    v3_mode = config.MISSION_MODE in ("sweep", "capture", "full")
    if config.USE_GYRO and gyro is not None and v3_mode:
        ctx.gyro_sign = require_gyro_sign()   # GYRO_POSITIVE_DIRECTION 未实测就报错，不猜
        ctx.use_gyro = True
    if config.USE_TOUCH and touch is not None and v3_mode:
        ctx.use_touch = True
    drive.reset()
    if ctx.use_gyro:
        # 起步先静止 GYRO_SETTLE_MS 再归零：刚上电的零漂会直接进航向
        await wait(config.GYRO_SETTLE_MS)
        gyro.reset_angle(0)
        ev("gyro zeroed sign={:+.0f}".format(ctx.gyro_sign))
    await odom_update()

    try:
        result = await run_mode(config.MISSION_MODE)
    except Abort as e:
        result = "ABORT " + repr(e.args)
    drive.brake()

    o = ctx.odom
    print("==== SUMMARY ====")
    print("mode", config.MISSION_MODE, "sense", sense.mode)
    print("result:", result)
    print("time_ms", ctx.now(), "samples", ctx.n,
          "white_frac {:.2f}".format(ctx.white_n / ctx.n if ctx.n else 0),
          "max_white_run_mm", ctx.max_white_run)
    print("twists", ctx.twists, "lost", ctx.lost, "overruns", ctx.overruns,
          "stalls", ctx.stalls, "seek_fail", ctx.seek_fail, "outside_returns", ctx.outs)
    print("suspects", ctx.suspects, "captures", ctx.captures, "lost_in_carry", ctx.lost_objects,
          "heading_fixes", ctx.heading_fixes, "turn_k", ctx.turn_k)
    print("markers", ctx.markers_seen, "finish_pose", ctx.finish_pose, "target", ctx.target,
          "axis", ctx.axis)
    print("end pose x={:.0f} y={:.0f} h={:.0f}".format(o.x, o.y, o.heading))
    ev3.screen.print(result)
    ev3.speaker.beep(500, 400)


run_task(main())
