"""python tests/report_scenarios.py — 生成仿真场景成功率表（写进 docs/offline-results.md）。

每行 = 若干随机种子；PASS = 任务结果为 PASS（edge_to_finish：到达终点色标；
full：物体最终位于终点色标 ±40 mm 内）。max_out = 光斑越过白边**内沿**的最大距离（仿真白边宽 40 mm）。
"""
import math
import random
import sys

from harness import press_right, run_script
from sim import BLUE_MARK, RED_MARK, Sim, rect_arena_marker
from test_mission_sim import H, W, rgb_cfg

ROWS = []


def scenario(name, n=10, mark=RED_MARK, sim_kw=None, cfg=None, start_fn=None, full=False, obj_load=40,
             arena=(W, H)):
    aw, ah = arena
    ok, outs, times = 0, [], []
    for seed in range(n):
        rng = random.Random(2000 + seed)
        if start_fn:
            start = start_fn(rng)
        else:
            start = (rng.uniform(150, aw - 150), rng.uniform(150, ah - 150), rng.uniform(0, 2 * math.pi))
        mk = rect_arena_marker(aw, ah, rng.choice(["right", "top", "left", "bottom"]), surf=mark)
        objs = []
        if full:
            objs = [{"x": rng.uniform(150, aw - 150), "y": rng.uniform(150, ah - 150), "r": 25, "load": obj_load}]
        s = Sim(W=aw, H=ah, start=start, seed=seed, buttons=press_right(), markers=[mk], objects=objs,
                max_ms=700000, **(sim_kw or {}))
        c = rgb_cfg(mark)
        if full:
            c.update(MISSION_MODE="full", MISSION_MAX_MS=600000)
        c.update(cfg or {})
        out, s = run_script("mission", s, seed=seed, **c)
        if full:
            o = s.objects[0]
            x0, y0, x1, y1, _ = mk
            good = x0 - 40 <= o["x"] <= x1 + 40 and y0 - 40 <= o["y"] <= y1 + 40
        else:
            good = "PASS" in [l for l in out.splitlines() if l.startswith("result:")][0]
        ok += good
        if good:
            times.append(s.t / 1000.0)
        outs.append(s.max_out)
    times.sort()
    row = "| {} | {}/{} | {:.0f} s | {:.0f} mm |".format(
        name, ok, n, times[len(times) // 2] if times else -1, max(outs))
    ROWS.append(row)
    print(row)
    sys.stdout.flush()


if __name__ == "__main__":
    print("| 场景 | 成功 | 中位用时 | 光斑最大越界 |")
    print("|---|---|---|---|")
    scenario("edge_to_finish 随机起点/朝向（基准）", 20)
    scenario("深色色标（反射率 < 边界阈值）", 10, mark=BLUE_MARK)
    scenario("传感器噪声 ×3", 10, sim_kw=dict(noise=0.45))
    scenario("每次读传感器 30 ms（模式切换代价）", 10, sim_kw=dict(read_cost_ms=30))
    scenario("小场地 700×500", 10, arena=(700, 500))
    scenario("起点在角落附近", 10, start_fn=lambda r: (r.choice([90, W - 90]), r.choice([90, H - 90]), r.uniform(0, 6.28)))
    scenario("起步光斑压在白边上（外侧是白桌面）", 10, sim_kw=dict(outside="white"),
             start_fn=lambda r: (r.uniform(200, W - 200), r.uniform(15, 50), r.uniform(0, 6.28)))
    scenario("加速度只有 300（停得更慢）", 10, sim_kw=dict(accel=300, turn_accel=300))
    scenario("光斑前伸 100 mm", 10, sim_kw=dict(sensor_offset=100))
    scenario("里程计 ±8% 误差", 10, sim_kw=dict(odo_dist_scale=1.08, odo_turn_scale=0.95))
    scenario("full：推送到终点（bounce，物体负载 40）", 20, full=True)
    scenario("full：里程计 ±8% 误差", 12, full=True, sim_kw=dict(odo_dist_scale=1.08, odo_turn_scale=0.95))
    scenario("full：轻物体（负载 15，接近阈值）", 12, full=True, obj_load=15)
    scenario("full：DELIVERY_MODE=edge", 12, full=True, cfg=dict(DELIVERY_MODE="edge"))
    scenario("full：SWEEP_PATTERN=lanes（实验）", 12, full=True, cfg=dict(SWEEP_PATTERN="lanes"))
