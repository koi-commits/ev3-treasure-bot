r"""
mission.py — 完整自主任务（2026-11-03 晚重写）。逻辑全部【待实机验证】，
巡线核心复用已验证的 fsm.EdgeFollower（与 demo_edge 同参数、同时序）。

    pbrun.cmd src\mission.py

模式（config.MISSION_MODE）：
  edge_to_finish  寻边 -> 沿边巡线 -> 运行中识别终点色标 -> 停车     ← 下周二主测
  survey          寻边 -> 沿边巡线 FOLLOW_MAX_MS，只记录色标/遥测，不停车
  full            找终点并记录位置 -> 覆盖扫场(V 形推板，扫描线垂直于终点所在边) -> 负载判定推到物体
                  -> 按里程计/沿边送到终点 -> 推入 -> 倒车离开

异常恢复（监督层，不改动三状态机本身）：
  * 丢边：CRUISE 中 LOST_EDGE_MS 未见白 -> 重新直行寻边
  * 越界：ESCAPE 续转用完仍在白区 -> 倒车 OVERRUN_BACK_MM，状态机重置
  * 堵转：drive.stalled() 持续 STALL_MS -> 倒车 + 转离边界
  * 出界：直行超过 ARENA_DIAG_MM 仍未见白（凸场地内不可能）-> 朝最近见白位置返回、穿过白带
  * 起步光斑在白边上 -> 先直线倒车回到地面
  * 所有定距动作都带超时（drive.done() 轮询），不会因堵转卡死
  * 整场 MISSION_MAX_MS 兜底

按键：RIGHT 开始（超时自动开始），CENTER 随时急停（Pybricks 默认）。
"""

import config
from common import battery_check, log, wait_button_or_timeout
from fsm import (CLEAR, CRUISE, ESCAPE, EdgeFollower, LoadMonitor, Odometry,
                 straight_run_heading, wrap180)
from hardware import color, drive, ev3, left, right
from sensing import Sensing

from pybricks.parameters import Color
from pybricks.tools import StopWatch, run_task, wait

try:
    import urandom as random
except ImportError:
    import random

PERIOD = config.SAMPLE_PERIOD_MS


def rand_sign():
    return 1 if random.randint(0, 1) else -1

FINISH = config.FINISH_LABEL
SIDE = config.EDGE_SIDE
INTO_ARENA = -SIDE              # 远离白边的转向符号（与 ESCAPE 方向一致）
TRACE_MS = 1000


class Abort(Exception):
    pass




class Ctx:
    """整场任务共享状态 + 统计。"""

    def __init__(self, sense):
        self.sense = sense
        self.odom = Odometry()
        self.clock = StopWatch()
        self.stall_ms = 0
        self.finish_pose = None
        self.markers_seen = {}
        self.last_marker = None
        self.n = 0
        self.white_n = 0
        self.twists = 0
        self.lost = 0
        self.overruns = 0
        self.stalls = 0
        self.seek_fail = 0
        self.captures = 0
        self.carrying = False
        self.last_white = None   # 最近一次见白时的车体位置 (x, y)
        self.outs = 0
        self.edge_trace = []     # 最近几次 ESCAPE 时的车体位置，用来估计边的方向
        self.next_trace = 0
        self.result = "RUNNING"

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


async def tick():
    """每个循环周期调用一次：读传感器、更新里程计、堵转计时、色标记录、超时检查。"""
    s = await ctx.sense.read()
    odom_update()
    ctx.n += 1
    o = ctx.odom
    if s.white:
        ctx.white_n += 1
        ctx.last_white = (o.x, o.y)
    if drive.stalled():
        ctx.stall_ms += PERIOD
    else:
        ctx.stall_ms = 0

    if s.marker is not None and s.marker != ctx.last_marker:
        ctx.markers_seen[s.marker] = ctx.markers_seen.get(s.marker, 0) + 1
        ev("marker", s.marker, "x={:.0f} y={:.0f}".format(o.x, o.y))
        if s.marker == FINISH and ctx.finish_pose is None:
            ctx.finish_pose = (o.x, o.y, o.heading)
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
            drive.stop()
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
    drive.stop()
    await straight(-config.STALL_BACK_MM)
    await rotate(INTO_ARENA * config.STALL_TURN_DEG)
    ctx.stall_ms = 0


async def drive_until(speed, max_mm, stop_on_finish=False, monitor=None):
    """直行（DriveBase 自带航向保持）直到：
    'white' 连续 EDGE_CONFIRM_SAMPLES 个白样本 | 'finish' 色标确认 |
    'stall' | 'capture'（monitor 判定）| 'distance'。"""
    d0 = drive.distance()
    whites = 0
    drive.drive(speed, 0)
    while True:
        s = await tick()
        if stop_on_finish and s.marker == FINISH:
            drive.stop()
            return "finish"
        if s.white:
            whites += 1
            if whites >= config.EDGE_CONFIRM_SAMPLES:
                drive.stop()
                return "white"
        else:
            whites = 0
        if ctx.stall_ms >= config.STALL_MS:
            drive.stop()
            return "stall"
        if monitor is not None and monitor.push(load_sum()):
            drive.stop()
            return "capture"
        if abs(drive.distance() - d0) >= max_mm:
            drive.stop()
            return "distance"
        await wait(PERIOD)


# -------------------------------------------------------------------
# 起步检查 + 寻边：随机起点/朝向
# -------------------------------------------------------------------
async def start_check():
    """起步时光斑已压在白边上：直线倒车直到连续看到地面。
    前提：车体中心放在深色场地内（README 的摆放规则），倒车 = 光斑回到场内。"""
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
    drive.stop()
    if dark < 3:
        ev("still white: turn 180")
        await rotate(180)
    else:
        await straight(-config.SEEK_BACKOFF_MM)


async def seek_edge(stop_on_finish):
    """返回 'edge' / 'finish' / None（全部尝试失败）。
    推着物体时（ctx.carrying）不倒车：V 形推板倒车会把物体留在原地。"""
    for attempt in range(config.SEEK_ATTEMPTS):
        r = await drive_until(config.SEEK_SPEED, config.ARENA_DIAG_MM, stop_on_finish)
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
async def follow(max_ms, until_finish):
    """返回 'finish' / 'lost' / 'overrun' / 'time'。"""
    f = EdgeFollower(SIDE, config.FOLLOW_SPEED, config.APPROACH_STEER,
                     config.EDGE_TURN_RATE, config.EDGE_ESCAPE_MS, config.EDGE_CLEAR_MS)
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
            drive.stop()
            ctx.twists += f.twists
            return "finish"

        tw = f.twists
        sp, tr = f.step(t, s.white)
        drive.drive(sp, tr)
        if f.twists != tw:
            ctx.edge_trace.append((ctx.odom.x, ctx.odom.y))
            if len(ctx.edge_trace) > 6:
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
            drive.stop()
            await straight(-config.OVERRUN_BACK_MM)
            if overrun_streak > config.OVERRUN_LIMIT:
                ctx.twists += f.twists
                return "overrun"
            f.reset(t)
            handled_exhaust = 0
            prev_state = CRUISE

        elif f.state == CRUISE and t - f.last_white_t > config.LOST_EDGE_MS:
            drive.stop()
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

    drive.stop()
    ctx.twists += f.twists
    if seg_n:
        print("  follow white_frac={:.2f} twists={}".format(seg_white / seg_n, f.twists))
    return "time"


async def edge_then_follow(max_ms, until_finish):
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
        r = await follow(deadline - ctx.now(), until_finish)
        if r == "finish":
            return "finish"
        if r == "time":
            return "time"
        if r == "overrun":
            # 多次越界：掉头回场地内部再寻边
            await rotate(INTO_ARENA * 150)
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
        drive.stop()
        await straight(config.RETURN_IN_MM)


def edge_heading():
    """由最近几次 ESCAPE 的位置估计"沿边前进方向"（只用拐角之后的共线点）。"""
    return straight_run_heading(ctx.edge_trace)


async def finish_stop(push_mm):
    drive.stop()
    ev3.light.on(Color.GREEN)
    if push_mm:
        await straight(push_mm)
    ev3.speaker.beep(1500, 300)
    ev("FINISH reached")


# -------------------------------------------------------------------
# full 模式：覆盖扫场 + 推送 + 交付
# -------------------------------------------------------------------
def new_monitor():
    return LoadMonitor(config.LOAD_SKIP_N, config.LOAD_LEARN_N, config.LOAD_WINDOW,
                       config.LOAD_K, config.LOAD_ABS_MARGIN, config.LOAD_HOLD_N)


async def face(heading):
    """转到绝对朝向（DriveBase angle 坐标系，任务开始后从不 reset）。"""
    await rotate(wrap180(heading - drive.angle()))


async def sweep():
    """往返扫描线（boustrophedon），用 DriveBase 绝对角度保持各线平行。
    车头朝场地内部开始；每条线直行到对边，平移 LANE_WIDTH_MM 再折返。
    平移撞到侧边 -> 平移方向反过来，继续扫另一半（第二次撞侧边才结束）。
    返回 'capture' / 'done'。"""
    mon = new_monitor() if config.CAPTURE_MODE == "load" else None
    lane_h = drive.angle()
    base_h = lane_h
    shift = 1
    flips = 0
    for lane in range(config.SWEEP_MAX_LANES):
        if mon is not None:
            mon.restart()
        await face(lane_h)
        r = await drive_until(config.SWEEP_SPEED, config.ARENA_DIAG_MM, False, mon)
        if r == "distance":
            # 直线超过场地对角线仍未见白：一定已在场外
            await return_inside()
            lane_h = base_h if abs(wrap180(drive.angle() - base_h)) < 90 else base_h + 180
            continue
        ev("lane", lane, r)
        if r == "capture":
            ctx.captures += 1
            ev3.speaker.beep(700, 150)
            return "capture"
        if r == "stall":
            # 推不动（太重/卡住）——当作推到物体，交给交付流程
            ctx.captures += 1
            await straight(-20)
            return "capture"
        # 光斑在车轴前方：原地转之前必须退够，否则转向时光斑会扫出白边外
        await straight(-config.SWEEP_END_BACK_MM)
        await face(base_h + 90 * shift)
        r2 = await drive_until(config.SWEEP_SPEED, config.LANE_WIDTH_MM)
        if r2 == "white":
            flips += 1
            ev("sweep: side edge", flips)
            await straight(-config.SWEEP_END_BACK_MM)
            if flips >= 2:
                return "done"
            shift = -shift
        lane_h += 180
    return "done"


async def bounce():
    """随机反弹覆盖（不依赖几何标定/航向估计）：
    直行到白边 -> 倒车 SWEEP_END_BACK_MM -> 原地随机转 100–170° -> 再直行。返回 'capture' / 'done'。"""
    mon = new_monitor() if config.CAPTURE_MODE == "load" else None
    for leg in range(config.BOUNCE_MAX_LEGS):
        if mon is not None:
            mon.restart()
        r = await drive_until(config.SWEEP_SPEED, config.ARENA_DIAG_MM, False, mon)
        ev("leg", leg, r)
        if r == "capture" or r == "stall":
            ctx.captures += 1
            ev3.speaker.beep(700, 150)
            if r == "stall":
                await straight(-20)
            return "capture"
        if r == "distance":
            await return_inside()
            continue
        await straight(-config.SWEEP_END_BACK_MM)
        await rotate(rand_sign() * random.randint(100, 170))
    return "done"


async def goto_pose(x, y, legs=3):
    """按里程计分段走向 (x, y)，途中见色标即停，见白边即返回。"""
    for _ in range(legs):
        rel, dist = ctx.odom.bearing_to(x, y)
        if dist < 40:
            return "arrived"
        await rotate(rel)
        step = dist if dist < 400 else dist / 2
        r = await drive_until(config.SWEEP_SPEED, step + 30, True)
        if r in ("finish", "white", "stall"):
            return r
    return "arrived"


async def deliver():
    """推着物体去终点。返回 True 表示确认到达终点色标。"""
    ctx.carrying = True
    if config.DELIVERY_MODE == "odometry" and ctx.finish_pose is not None:
        x, y, _ = ctx.finish_pose
        r = await goto_pose(x, y)
        ev("goto finish:", r)
        if r == "finish":
            await finish_stop(config.FINISH_PUSH_MM)
            return True
        if r == "stall":
            await unstall()
    # 回退方案：直行到边，沿边找色标
    r = await edge_then_follow(config.DELIVER_FOLLOW_MAX_MS, True)
    if r == "finish":
        await finish_stop(config.FINISH_PUSH_MM)
        return True
    return False


# -------------------------------------------------------------------
async def run_mode(mode):
    if mode == "edge_to_finish":
        if not ctx.sense.marker_enabled:
            ev("no marker -> follow only")
        r = await edge_then_follow(config.FOLLOW_MAX_MS, ctx.sense.marker_enabled)
        if r == "finish":
            await finish_stop(0)
            return "PASS finish"
        return "END " + r

    if mode == "survey":
        r = await edge_then_follow(config.FOLLOW_MAX_MS, False)
        return "END survey " + r

    if mode == "full":
        if not ctx.sense.marker_enabled:
            return "FAIL full needs markers (SENSE_MODE rgb + SIGNATURES)"
        r = await edge_then_follow(config.FOLLOW_MAX_MS, True)
        if r != "finish":
            return "FAIL finish not found: " + r
        ev("finish pose recorded")
        drive.stop()
        eh = edge_heading()
        await straight(-config.FINISH_BACK_MM)
        if eh is not None:
            ev("edge heading", int(eh))
            await face(eh + INTO_ARENA * 90)     # 扫描线垂直于终点所在的边
        else:
            await rotate(INTO_ARENA * 90)
        r = await (bounce() if config.SWEEP_PATTERN == "bounce" else sweep())
        if r != "capture":
            return "END sweep done, no capture"
        ok = await deliver()
        await straight(-config.FINISH_BACK_MM)
        return "PASS delivered" if ok else "FAIL delivery"

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
    drive.stop()
    ctx.result = result

    o = ctx.odom
    print("==== SUMMARY ====")
    print("mode", config.MISSION_MODE, "sense", sense.mode)
    print("result:", result)
    print("time_ms", ctx.now(), "samples", ctx.n,
          "white_frac {:.2f}".format(ctx.white_n / ctx.n if ctx.n else 0))
    print("twists", ctx.twists, "lost", ctx.lost, "overruns", ctx.overruns,
          "stalls", ctx.stalls, "seek_fail", ctx.seek_fail, "captures", ctx.captures,
          "outside_returns", ctx.outs)
    print("markers", ctx.markers_seen, "finish_pose", ctx.finish_pose)
    print("end pose x={:.0f} y={:.0f} h={:.0f}".format(o.x, o.y, o.heading))
    ev3.screen.print(result)
    ev3.speaker.beep(500, 400)


run_task(main())
