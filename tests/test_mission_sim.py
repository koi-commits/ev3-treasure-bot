"""在仿真里原样运行 src/mission.py、src/demo_edge.py 和各标定/探测脚本。

仿真只证明**逻辑与状态切换**正确，不证明真实场地的物理表现。
"""

import math
import random
import unittest

from harness import press_right, run_script
from fsm import build_signature
from sim import BLUE_MARK, FLOOR, RED_MARK, WHITE, Sim, rect_arena_marker

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


def result_line(out):
    return [l for l in out.splitlines() if l.startswith("result:")][0]


def random_start(rng, margin=150):
    return (rng.uniform(margin, W - margin), rng.uniform(margin, H - margin), rng.uniform(0, 2 * math.pi))


def run_edge_to_finish(seeds, mark=RED_MARK, sim_kw=None, start_fn=random_start, cfg=None):
    passes, max_out = 0, 0.0
    for seed in seeds:
        rng = random.Random(1000 + seed)
        mk = rect_arena_marker(W, H, rng.choice(["right", "top", "left", "bottom"]), surf=mark)
        s = Sim(W=W, H=H, start=start_fn(rng), seed=seed, buttons=press_right(), markers=[mk], **(sim_kw or {}))
        out, s = run_script("mission", s, seed=seed, **rgb_cfg(mark, **(cfg or {})))
        if "PASS finish" in result_line(out):
            passes += 1
        max_out = max(max_out, s.max_out)
    return passes, max_out


class TestBaselineReproduction(unittest.TestCase):
    def test_demo_edge_matches_field_log(self):
        """未改动的 demo_edge.py 在仿真里的统计应接近 11-03 实测（white_frac 0.10–0.17，每段 4–5 锯齿）。"""
        fracs, twists = [], []
        for seed in range(3):
            s = Sim(start=(600, 450, math.radians(30 * seed)), seed=seed, buttons=press_right())
            out, s = run_script("demo_edge", s, seed=seed)
            fracs += [float(l.split("white_frac=")[1].split()[0]) for l in out.splitlines() if "white_frac=" in l]
            twists += [int(l.split("edge_twists")[1]) for l in out.splitlines() if "edge_twists" in l]
            self.assertLess(s.max_out, 25)
        self.assertTrue(0.03 <= sum(fracs) / len(fracs) <= 0.25, fracs)
        self.assertTrue(3 <= sum(twists) / len(twists) <= 6, twists)


class TestEdgeToFinish(unittest.TestCase):
    def test_random_starts(self):
        p, out = run_edge_to_finish(range(10))
        self.assertGreaterEqual(p, 10)
        self.assertLess(out, 40)        # 仿真白边带宽 40 mm：光斑最远不越过白边外沿

    def test_dark_marker(self):
        p, _ = run_edge_to_finish(range(5), mark=BLUE_MARK)
        self.assertGreaterEqual(p, 5)

    def test_slow_sensor_reads(self):
        """每次读传感器耗时 30 ms（模拟模式切换代价）仍能完成。"""
        p, out = run_edge_to_finish(range(5), sim_kw=dict(read_cost_ms=30))
        self.assertGreaterEqual(p, 5)
        self.assertLess(out, 45)

    def test_sensor_starts_on_white(self):
        p, _ = run_edge_to_finish(range(8), start_fn=lambda r: (r.uniform(200, W - 200), r.uniform(15, 50),
                                                                r.uniform(0, 2 * math.pi)),
                                  sim_kw=dict(outside="white"))
        self.assertGreaterEqual(p, 8)

    def test_reflection_mode_follows_without_marker(self):
        s = Sim(start=(600, 450, 0.3), seed=1, buttons=press_right())
        out, s = run_script("mission", s, FOLLOW_MAX_MS=40000)
        self.assertIn("END time", result_line(out))
        self.assertLess(s.max_out, 25)
        self.assertEqual(s.max_out_center, 0)

    def test_misconfigured_rgb_mode_refuses(self):
        s = Sim(seed=1, buttons=press_right())
        out, _ = run_script("mission", s, SENSE_MODE="rgb")
        self.assertIn("config error", out)


class TestRecovery(unittest.TestCase):
    def test_lost_edge_recovers(self):
        """把丢边超时调得很短，强制频繁触发 lost -> 重新寻边，任务仍应完成。"""
        p, _ = run_edge_to_finish(range(4), cfg=dict(LOST_EDGE_MS=1500))
        self.assertGreaterEqual(p, 4)

    def test_stall_obstacle(self):
        """场地中间放一个推不动的障碍：堵转恢复后仍能找到终点。"""
        passes = 0
        for seed in range(4):
            mk = rect_arena_marker(W, H, "right")
            s = Sim(start=(250, 450, 0.0), seed=seed, buttons=press_right(), markers=[mk],
                    obstacles=[(500, 300, 560, 600)])
            out, s = run_script("mission", s, seed=seed, **rgb_cfg())
            passes += "PASS" in result_line(out)
            self.assertIn("stall", out)
        self.assertGreaterEqual(passes, 4)


class TestFull(unittest.TestCase):
    """full 模式按"地面真值"评判（物体中心是否在色标上），接触式物体模型。"""

    def test_full_interior_objects(self):
        import evaluate as E
        runs = [E.run_one(i, mode="full") for i in range(8)]
        acceptable = sum(r["acceptable"] for r in runs)
        self.assertGreaterEqual(acceptable, 4, [r["result"] for r in runs])
        self.assertLessEqual(sum(r["wheel_beyond"] for r in runs), 1)   # 已知：拐角处偶发

    def test_full_without_markers_refuses(self):
        s = Sim(seed=1, buttons=press_right())
        out, _ = run_script("mission", s, MISSION_MODE="full")
        self.assertIn("FAIL full needs markers", result_line(out))


class TestDeviceScriptsSmoke(unittest.TestCase):
    """每个新/改的上机脚本都在仿真里跑一遍，抓 await/格式化/接口错误。"""

    def run_until_timeout(self, name, sim):
        return run_script(name, sim, allow_timeout=True)[0]

    def test_probe_timing(self):
        out = self.run_until_timeout("probe_timing", Sim(seed=1))
        self.assertIn("PHASE E", out)

    def test_color_probe(self):
        out = self.run_until_timeout("color_probe", Sim(seed=1))
        self.assertIn("color counts", out)

    def test_calibrate_marker(self):
        from pybricks.parameters import Button
        R = Button.RIGHT
        mk = rect_arena_marker(W, H, "right")
        btn = [(100, 200, R, (600, 450, 0)),                 # floor
               (8000, 8100, R, (W - 40, 300, 0)),             # 光斑(车前 60 mm)落在白边带内
               (16000, 16100, R, (W - 100, H / 2, 0))]        # marker
        s = Sim(seed=2, markers=[mk], buttons=btn, max_ms=120000)
        out = self.run_until_timeout("calibrate_marker", s)
        self.assertIn("RESULT: PASS", out)
        self.assertIn("SIGNATURES = {", out)

    def test_probe_load(self):
        s = Sim(start=(100, 450, 0), seed=1, buttons=[(100, 200, __import__("pybricks.parameters").parameters.Button.RIGHT)],
                max_ms=400000)
        out = self.run_until_timeout("probe_load", s)
        self.assertIn("EMPTY", out)

    def test_calibrate_geometry(self):
        from pybricks.parameters import Button
        R = Button.RIGHT
        s = Sim(start=(100, 450, 0), seed=1, max_ms=200000,
                buttons=[(100, 200, R), (20000, 20100, R), (50000, 50100, R)])
        out = self.run_until_timeout("calibrate_geometry", s)
        self.assertIn("new WHEEL_DIAMETER_MM", out)
        self.assertIn("AXLE", out)

    def test_survey_mode(self):
        s = Sim(start=(600, 450, 1.0), seed=3, buttons=press_right(), markers=[rect_arena_marker(W, H, "top")])
        out, _ = run_script("mission", s, **rgb_cfg(MISSION_MODE="survey", FOLLOW_MAX_MS=120000))
        self.assertIn("END survey", result_line(out))
        self.assertIn("marker finish", out)


if __name__ == "__main__":
    unittest.main()
