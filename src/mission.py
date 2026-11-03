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

按键：RIGHT 开始（超时自动开始），CENTER 随时急停（Pybricks 默认）。
"""

import config
from common import battery_check, log, wait_button_or_timeout
from fsm import (CLEAR, CRUISE, ESCAPE, EdgeFollower, LoadMonitor, Odometry, axis_snap,
                 diagonal_choices, rotate_about, straight_run_heading, tap_verdict, wrap180)
from hardware import color, drive, ev3, left, right
from sensing import Sensing

from pybricks.parameters import Color
from pybricks.tools import StopWatch, run_task, wait

try:
    from umath import cos, pi, sin
except ImportError:
    from math import cos, pi, sin

try:
    import urandom as random
except ImportError:
    import random

PERIOD = config.SAMPLE_PERIOD_MS
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

    def now(self):
        return self.clock.time()


ctx = None


def ev(*args):
    """事件日志：终端带时间戳，屏幕只显示内容。"""
    print("[{:6d}]".format(ctx.now()), *args)
    ev3.screen.print(*args)


def load_sum():
    return abs(left.load()) + abs(right.load())


def odom_update():
    ctx.odom.update(drive.distance(), drive.angle())


def ahead(x, y, heading, d):
    """从 (x, y) 沿 DriveBase 航向 heading 前进 d 的点。"""
    rad = -heading * pi / 180.0
    return x + d * cos(rad), y + d * sin(rad)


async def tick():
    """每个循环周期调用一次：读传感器、更新里程计、堵转计时、色标记录、越界统计、超时检查。"""
    s = await ctx.sense.read()
    odom_update()
    ctx.n += 1
    o = ctx.odom
    if s.white:
        ctx.white_n += 1
        ctx.last_white = (o.x, o.y)
        if ctx.white_d0 is None:
            ctx.white_d0 = drive.distance()
        run = abs(drive.distance() - ctx.white_d0)
        if run > ctx.max_white_run:
            ctx.max_white_run = run
    else:
        ctx.white_d0 = None
    if drive.stalled():
        ctx.stall_ms += PERIOD
    else:
        ctx.stall_ms = 0

    if s.label == FINISH and not ctx.carrying and len(ctx.marker_pts) < 300:
        ctx.marker_pts.append(ahead(o.x, o.y, o.heading, config.SENSOR_OFFSET_MM))

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


# -------------------------------------------------------------------
# 基本动作：定距/定角，带超时，结束后更新里程计
# -------------------------------------------------------------------
async def _settle(timeout_ms):
    t = 0
    while not drive.done():
        await wait(PERIOD)
        t += PERIOD
        if t > timeout_ms:
            drive.brake()
            ev("move timeout")
            break
    odom_update()


async def straight(mm):
    odom_update()
    drive.straight(mm, wait=False)
    await _settle(2000 + abs(mm) * 25)


async def rotate(deg):
    odom_update()
    drive.turn(deg, wait=False)
    await _settle(2000 + abs(deg) * 25)


async def unstall():
    ctx.stalls += 1
    ev("stall -> back")
    drive.brake()
    await straight(-config.STALL_BACK_MM)
    await rotate(INTO_ARENA * config.STALL_TURN_DEG)
    ctx.stall_ms = 0


async def drive_until(speed, max_mm, stop_on_finish=False, monitor=None):
    """直行（DriveBase 自带航向保持）直到：
    'white' 连续 EDGE_CONFIRM_SAMPLES 个白样本 | 'finish' 色标确认 |
    'stall' | 'suspect'（monitor 判定负载抬升）| 'distance' |
    'lost'（推送中负载回落 = 物体丢了）。停车一律 brake。"""
    d0 = drive.distance()
    whites = 0
    low = 0
    drive.drive(speed, 0)
    while True:
        s = await tick()
        r = None
        # 推送中：负载回落到空载水平且持续 -> 物体丢了（起步 15 mm 不判）
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
        elif s.white:
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
    if not s.white:
        return
    ev("start on white -> back")
    d0 = drive.distance()
    dark = 0
    drive.drive(-config.SEEK_SPEED, 0)
    while abs(drive.distance() - d0) < config.START_BACK_MAX_MM and dark < 3:
        await wait(PERIOD)
        s = await tick()
        dark = 0 if s.white else dark + 1
    drive.brake()
    if dark < 3:
        ev("still white: turn 180")
        await rotate(180)
    else:
        await straight(-config.SEEK_BACKOFF_MM)


async def seek_edge(stop_on_finish):
    """返回 'edge' / 'finish' / None（全部尝试失败）。
    推着物体时（ctx.carrying）不倒车：V 形推板倒车会把物体留在原地。"""
    for attempt in range(config.SEEK_ATTEMPTS):
        r = await drive_until(config.CARRY_SPEED if ctx.carrying else config.SEEK_SPEED,
                              config.ARENA_DIAG_MM, stop_on_finish)
        if r == "finish":
            return "finish"
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


# -------------------------------------------------------------------
# 沿边巡线 + 监督
# -------------------------------------------------------------------
async def follow(max_ms, until_finish, side=SIDE):
    """返回 'finish' / 'lost' / 'overrun' / 'time'。推送模式下 ESCAPE 用前进急弯。"""
    esc_v = config.CARRY_ESCAPE_SPEED if ctx.carrying else 0
    steer, esc_ms, clear_ms = config.FOLLOW_PROFILES[config.FOLLOW_PROFILE]
    f = EdgeFollower(side, config.FOLLOW_SPEED, steer,
                     config.EDGE_TURN_RATE, esc_ms, clear_ms,
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
        if s.white:
            seg_white += 1

        if until_finish and s.marker == FINISH:
            drive.brake()
            ctx.twists += f.twists
            return "finish"

        tw = f.twists
        sp, tr = f.step(t, s.white)
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
            await rotate(INTO_ARENA * 150)     # 多次越界：掉头回场地内部再寻边
        # 'lost'：保持当前朝向直接重新直行寻边
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
            dark = 0 if s.white else dark + 1
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
        if s.white:
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
    a0 = drive.angle()
    sgn = 1 if deg > 0 else -1
    drive.drive(config.CARRY_SPEED, sgn * config.CARRY_TURN_RATE)
    while abs(drive.angle() - a0) < abs(deg):
        s = await tick()
        if s.marker == FINISH:
            drive.brake()
            return "finish"
        if s.white:
            drive.brake()
            return "white"
        await wait(PERIOD)
    drive.brake()
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


def compute_target():
    """由 ctx.marker_pts（色标上的光斑位置）与 ctx.axis 求交付目标点与场内待命点。"""
    pts = ctx.marker_pts
    if pts:
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
    else:
        cx, cy = ctx.finish_pt
    if ctx.axis is None:
        ctx.target = (cx, cy)
        ctx.waypoint = None
    else:
        inward = ctx.axis + INTO_ARENA * 90.0
        ctx.target = ahead(cx, cy, inward, config.MARKER_INSET_MM)
        ctx.waypoint = ahead(ctx.target[0], ctx.target[1], inward, config.APPROACH_STANDOFF_MM)


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
    t = 0
    whites = 0
    low = 0
    best = 1e9                                # 追踪过程中离目标最近的剩余距离
    k = config.PURSUIT_GAIN
    lim = config.CARRY_TURN_RATE
    while t < max_ms:
        s = await tick()
        rel, dist = ctx.odom.bearing_to(pt[0], pt[1])
        if dist - lead < best:
            best = dist - lead
        r = None
        if s.marker == FINISH:
            r = "finish"
        elif s.white:
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


async def deliver():
    """推着物体去终点：先到场内待命点，再从场内正对色标直线推进，让**物体中心**停在目标点。
    返回 'marker' / 'odometry' / 'lost' / 'fail'。"""
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
            r = await carry_goto(ctx.target, config.OBJ_LEAD_MM, 20.0)
            ev("deliver target:", r)
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
        # 只验证搜索 + 负载嫌疑 + tap 确认；不放物体时用来测误报率
        await start_check()
        r = await bounce()
        return "END capture: " + ("CONFIRMED" if r == "capture" else "none")

    if mode == "full":
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

    await wait_button_or_timeout(ev3, config.START_TIMEOUT_MS, prompt="RIGHT to start")
    ev3.light.on(Color.GREEN)
    ctx = Ctx(sense)
    drive.reset()
    odom_update()

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
