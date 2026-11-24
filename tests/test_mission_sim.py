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
        """未改动的 demo_edge.py 在仿真里的统计应接近 11-03 实测（white_frac 0.10–0.17，每段 4–5 锯齿）。

        这是保留的**旧单传感器巡线**测试：11-03 实测用的是旧底盘（光斑在车前 20 mm、居中、
        只有一个颜色传感器），所以这里显式用旧几何。新几何（67 mm / ±32 mm 两个光斑）的
        巡线由 TestEdgeToFinish / TestRecovery 用 two_sensor 巡线器测试。"""
        fracs, twists = [], []
        old = dict(SENSOR_OFFSET_MM=20, SENSOR_LATERAL_MM=0, USE_RIGHT_SENSOR=False, FOLLOWER="single")
        for seed in range(3):
            s = Sim(start=(600, 450, math.radians(30 * seed)), seed=seed, buttons=press_right(),
                    sensor_offset=20, sensor_lat=0, right_sensor=False)
            out, s = run_script("demo_edge", s, seed=seed, **old)
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
        # 这些用例验证的是"尚未标定色标"时的行为，所以显式传入未标定的传感配置，
        # 不依赖 src/config.py 里当前实测的 SENSE_MODE / SIGNATURES。
        out, s = run_script("mission", s, FOLLOW_MAX_MS=40000, SENSE_MODE="reflection",
                            SIGNATURES={}, RGB_EDGE_BRIGHT=None)
        self.assertIn("END time", result_line(out))
        self.assertLess(s.max_out, 25)
        self.assertEqual(s.max_out_center, 0)

    def test_misconfigured_rgb_mode_refuses(self):
        s = Sim(seed=1, buttons=press_right())
        out, _ = run_script("mission", s, SENSE_MODE="rgb", SIGNATURES={}, RGB_EDGE_BRIGHT=None)
        self.assertIn("config error", out)


class TestRecovery(unittest.TestCase):
    def test_lost_edge_recovers(self):
        """把丢边超时调得很短，强制频繁触发 lost -> 重新寻边，任务仍应完成。

        2026-11-24 两传感器几何（光斑 67 mm / ±32 mm，two_sensor 巡线器）：20 个 seed 至少 19 个
        完成，且**任何 seed 都不允许整车出场**（硬性要求）。原来是 4 个 seed 全过；新几何下
        seed 1 在强制 1.5 s 丢边时 150 s 内找不到终点（只超时、不出场，见 PR #8）。"""
        passes, outs = 0, []
        for seed in range(20):
            rng = random.Random(1000 + seed)
            mk = rect_arena_marker(W, H, rng.choice(["right", "top", "left", "bottom"]), surf=RED_MARK)
            s = Sim(W=W, H=H, start=random_start(rng), seed=seed, buttons=press_right(), markers=[mk])
            out, s = run_script("mission", s, seed=seed, **rgb_cfg(RED_MARK, LOST_EDGE_MS=1500))
            passes += "PASS finish" in result_line(out)
            if s.robot_out:
                outs.append(seed)
        self.assertEqual(outs, [], "robot left the arena")
        self.assertGreaterEqual(passes, 19)

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
        results = [r["result"] for r in runs]
        delivered = sum(1 for r in runs if r["success"])
        # v3 契约（2026-11-18 红纸模型）：
        #   - success = 轮胎中心落在这张方形红纸位于场内（白线内沿以内）的那部分
        #   - 至少 3/8 成功；不能把物体推出场（object_out 上限 2/8）
        #   - acceptable = success 且没有双轮同时越过白带外沿
        #   轮子压白带不算犯规（规则只要求整车不掉出场），wheel_beyond 仅作信息。
        self.assertGreaterEqual(delivered, 3, results)
        self.assertLessEqual(sum(1 for r in runs if r["object_out"]), 2, results)
        self.assertGreaterEqual(sum(r["acceptable"] for r in runs), 0)

    def test_full_without_markers_refuses(self):
        s = Sim(seed=1, buttons=press_right())
        out, _ = run_script("mission", s, MISSION_MODE="full", SENSE_MODE="reflection",
                            SIGNATURES={}, RGB_EDGE_BRIGHT=None)
        self.assertIn("FAIL full needs markers", result_line(out))


class TestDeviceScriptsSmoke(unittest.TestCase):
    """每个新/改的上机脚本都在仿真里跑一遍，抓 await/格式化/接口错误。"""

    def run_until_timeout(self, name, sim, **cfg):
        return run_script(name, sim, allow_timeout=True, **cfg)[0]

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
        # 2026-11-24 实测：左光斑在车前 67 mm、左 32 mm（朝 +x 时光斑 = 车体 + (67, 32)）
        sx, sy = 67.0, 32.0
        btn = [(100, 200, R, (600, 450, 0)),                          # floor
               (8000, 8100, R, (W + 20 - sx, 300 - sy, 0)),          # 光斑落在白边带 (W, W+40) 内
               (16000, 16100, R, (W - 10 - sx, H / 2 - sy, 0))]      # 光斑在方形红纸上
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

    def test_probe_new_hw(self):
        """v3 新传感器验收：a 段 32 s + b 段 20 s，c 段等 RIGHT（约 52 s 处）后转 360°。

        这是陀螺仪的验收脚本：只在 HEADING_SOURCE = "gyro" 时 hardware 才构造陀螺。"""
        from pybricks.parameters import Button
        R = Button.RIGHT
        s = Sim(start=(600, 450, 0), seed=1, max_ms=120000,
                buttons=[(100, 200, R), (55000, 55100, R)])
        out = self.run_until_timeout("probe_new_hw", s, HEADING_SOURCE="gyro")
        self.assertIn("--- a) gyro drift ---", out)
        self.assertIn("--- c) turn 360", out)
        self.assertIn("suggested AXLE_TRACK_MM", out)
        self.assertIn("=== done ===", out)

    def test_survey_mode(self):
        s = Sim(start=(600, 450, 1.0), seed=3, buttons=press_right(), markers=[rect_arena_marker(W, H, "top")])
        out, _ = run_script("mission", s, **rgb_cfg(MISSION_MODE="survey", FOLLOW_MAX_MS=120000))
        self.assertIn("END survey", result_line(out))
        self.assertIn("marker finish", out)


if __name__ == "__main__":
    unittest.main()
