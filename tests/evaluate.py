"""evaluate.py — 用仿真"地面真值"评判每次运行，而不是相信机器人自己报告的结果。

每次运行输出一个 dict：
  claim          机器人自己报告成功（PASS / DELIVERED）
  success        地面真值成功：edge_to_finish = 停车时光斑在色标上；full = 物体中心在色标区（±10 mm）
  acceptable     success 且 无"轮子完全越过白带" 且 无物体被推出场地
  wheel_on_white 任一轮子触地点压到白带
  wheel_beyond   任一轮子越过白带外沿（= 整车出界）
  object_out     物体中心越过白带外沿
  false_claim    claim 但 not success
"""
import math
import random

from harness import press_right, run_script
from fsm import build_signature
from sim import FLOOR, RED_MARK, WHITE, Sim, rect_arena_marker

W, H = 1200, 900


def sig(surf, seed=5):
    rng = random.Random(seed)
    s = build_signature("x", [tuple(max(0, int(round(c + rng.gauss(0, 0.45)))) for c in surf["rgb"])
                              for _ in range(30)])
    return {"mean": s.mean, "tol": s.tol}


def rgb_cfg(mark=RED_MARK, **kw):
    c = dict(SENSE_MODE="rgb", RGB_EDGE_BRIGHT=14.0, FOLLOW_MAX_MS=150000,
             SIGNATURES={"floor": sig(FLOOR), "white": sig(WHITE), "finish": sig(mark)})
    c.update(kw)
    return c


# ---------------------------------------------------------------- 起点生成
def start_normal(rng, aw=W, ah=H):
    """正常随机起点：车体中心距白边 ≥ 150 mm，任意朝向（比赛规则下的典型摆放）。"""
    return (rng.uniform(150, aw - 150), rng.uniform(150, ah - 150), rng.uniform(0, 2 * math.pi))


def start_near_edge(rng, aw=W, ah=H):
    """边界起点：车体中心距某条边 60–120 mm，朝向随机（光斑可能已在白边上）。"""
    side = rng.randint(0, 3)
    d = rng.uniform(60, 120)
    th = rng.uniform(0, 2 * math.pi)
    if side == 0:
        return (rng.uniform(150, aw - 150), d, th)
    if side == 1:
        return (rng.uniform(150, aw - 150), ah - d, th)
    if side == 2:
        return (d, rng.uniform(150, ah - 150), th)
    return (aw - d, rng.uniform(150, ah - 150), th)


def start_corner(rng, aw=W, ah=H):
    return (rng.choice([90, aw - 90]), rng.choice([90, ah - 90]), rng.uniform(0, 2 * math.pi))


def object_interior(rng, aw=W, ah=H):
    return {"x": rng.uniform(150, aw - 150), "y": rng.uniform(150, ah - 150), "r": 25, "load": 20}


def object_near_edge(rng, aw=W, ah=H):
    o = object_interior(rng, aw, ah)
    side = rng.randint(0, 3)
    d = rng.uniform(40, 100)
    if side == 0:
        o["y"] = d
    elif side == 1:
        o["y"] = ah - d
    elif side == 2:
        o["x"] = d
    else:
        o["x"] = aw - d
    return o


# ---------------------------------------------------------------- 单次运行
def run_one(seed, mode="edge_to_finish", start=start_normal, obj=object_interior, mark=RED_MARK,
            arena=(W, H), sim_kw=None, cfg=None, obj_kw=None, base=3000):
    aw, ah = arena
    rng = random.Random(base + seed)
    st = start(rng, aw, ah)
    mk = rect_arena_marker(aw, ah, rng.choice(["right", "top", "left", "bottom"]), surf=mark)
    objs = []
    if mode == "full":
        o = obj(rng, aw, ah)
        o.update(obj_kw or {})
        objs = [o]
    s = Sim(W=aw, H=ah, start=st, seed=seed, buttons=press_right(), markers=[mk], objects=objs,
            max_ms=900000, **(sim_kw or {}))
    c = rgb_cfg(mark, MISSION_MODE=mode, MISSION_MAX_MS=600000)
    c.update(cfg or {})
    out, s = run_script("mission", s, seed=seed, allow_timeout=True, **c)
    res = [l for l in out.splitlines() if l.startswith("result:")]
    result = res[0][8:] if res else "(no result)"
    x0, y0, x1, y1, _ = mk
    if mode == "full":
        o = s.objects[0]
        success = (x0 - 10 <= o["x"] <= x1 + 10 and y0 - 10 <= o["y"] <= y1 + 10) and not o["out"]
        object_out = o["out"]
    else:
        sx, sy = s.sensor_pos()
        success = x0 <= sx <= x1 and y0 <= sy <= y1
        object_out = False
    claim = result.startswith("PASS") or result.startswith("DELIVERED")
    wheel_beyond = s.wheel_max_out > s.band
    return dict(seed=seed, result=result, claim=claim, success=success,
                acceptable=success and not wheel_beyond and not object_out,
                wheel_on_white=s.wheel_max_out > 0, wheel_beyond=wheel_beyond,
                object_out=object_out, false_claim=claim and not success,
                sensor_out=s.max_out, wheel_out=s.wheel_max_out, t=s.t / 1000.0, out=out)


def summarize(name, runs):
    n = len(runs)
    k = lambda key: sum(1 for r in runs if r[key])
    ts = sorted(r["t"] for r in runs if r["success"])
    med = ts[len(ts) // 2] if ts else float("nan")
    return ("| {} | {} | {}/{} | **{}/{}** | {} | {} | {} | {} | {:.0f} s |".format(
        name, n, k("success"), n, k("acceptable"), n, k("wheel_on_white"), k("wheel_beyond"),
        k("object_out"), k("false_claim"), med))


HEADER = ("| 场景 | 次数 | 任务成功 | **可接受成功** | 轮压白边 | 轮越过白带(整车出界) | 物体出界 | 虚报成功 | 中位用时 |\n"
          "|---|---|---|---|---|---|---|---|---|")
