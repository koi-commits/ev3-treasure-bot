"""纯逻辑单元测试（不需要仿真）。"""

import random
import unittest

import harness  # noqa: F401  (sets sys.path)
from fsm import (BoxClassifier, Debounce, EdgeFollower, LoadMonitor, Odometry,
                 axis_snap, build_signature, diagonal_choices, features, overlaps, straight_run_heading,
                 tap_verdict, wrap180)
from reference_20261103 import follow_edge_bangbang_reference

P = dict(threshold=6.35, speed=60, side=-1, approach_steer=22, turn_rate=130,
         escape_ms=350, clear_ms=400, period=50)


def run_new(refls):
    f = EdgeFollower(P["side"], P["speed"], P["approach_steer"], P["turn_rate"],
                     P["escape_ms"], P["clear_ms"])
    cmds = []
    t = 0
    for r in refls:
        cmds.append(f.step(t, r > P["threshold"]))
        t += P["period"]
    return cmds, f.twists


class TestFollowerEquivalence(unittest.TestCase):
    def test_random_sequences(self):
        rng = random.Random(1)
        for trial in range(300):
            n = rng.randint(1, 400)
            # 成块的黑/白段，模拟真实的锯齿与内转角长白段
            seq = []
            while len(seq) < n:
                white = rng.random() < 0.35
                seq += [rng.choice((10.0, 12.0, 14.5)) if white else rng.choice((0.3, 0.7, 2.0))] * rng.randint(1, 30)
            seq = seq[:n]
            self.assertEqual(run_new(seq), follow_edge_bangbang_reference(seq, **P), trial)

    def test_both_sides(self):
        for side in (-1, 1):
            q = dict(P, side=side)
            seq = [0.5] * 10 + [12] * 40 + [0.5] * 30
            f = EdgeFollower(side, 60, 22, 130, 350, 400)
            got = [f.step(i * 50, r > 6.35) for i, r in enumerate(seq)]
            self.assertEqual(got, follow_edge_bangbang_reference(seq, **q)[0])

    def test_exhausted_flag(self):
        f = EdgeFollower(-1, 60, 22, 130, 350, 400)
        t = 0
        for _ in range(60):            # 一直白：首次 + 2 次续转后进入 CLEAR 时仍白
            f.step(t, True)
            t += 50
        self.assertGreaterEqual(f.exhausted, 1)


class TestClassifier(unittest.TestCase):
    def setUp(self):
        rng = random.Random(2)
        noisy = lambda base, n=30: [tuple(max(0, int(round(c + rng.gauss(0, 0.6)))) for c in base) for _ in range(n)]
        self.floor = build_signature("floor", noisy((5, 5, 5)))
        self.white = build_signature("white", noisy((24, 23, 22)))
        self.finish = build_signature("finish", noisy((20, 7, 6)))
        self.clf = BoxClassifier([self.floor, self.white, self.finish])

    def test_basic(self):
        self.assertEqual(self.clf.classify((5, 5, 5))[0], "floor")
        self.assertEqual(self.clf.classify((24, 23, 22))[0], "white")
        self.assertEqual(self.clf.classify((20, 7, 6))[0], "finish")
        self.assertIsNone(self.clf.classify((5, 9, 18))[0])     # 没标定过的蓝色

    def test_no_overlap(self):
        self.assertFalse(overlaps(self.white, self.finish))
        self.assertFalse(overlaps(self.floor, self.finish))

    def test_black_white_mixture_is_not_marker(self):
        """跨越黑白边界时光斑混合出的灰色，任何比例都不应被判为色标。"""
        for a in range(0, 101, 5):
            k = a / 100.0
            rgb = tuple(int(round(5 * (1 - k) + w * k)) for w, k in zip((24, 23, 22), (k, k, k)))
            self.assertNotEqual(self.clf.classify(rgb)[0], "finish", rgb)

    def test_floor_marker_mixture_short(self):
        """进入色标时的混合样本即使偶尔命中，也要靠 Debounce 才能确认。"""
        d = Debounce(3)
        seq = ["floor", "finish", None, "finish", "finish", "finish"]
        out = [d.push(x if x == "finish" else None) for x in seq]
        self.assertEqual(out, [None, None, None, None, None, "finish"])

    def test_features(self):
        f = features((20, 10, 10))
        self.assertAlmostEqual(f[3], 40 / 3)
        self.assertAlmostEqual(f[4], 50.0)
        self.assertEqual(features((0, 0, 0))[3], 0)


class TestLoadMonitor(unittest.TestCase):
    def test_no_false_positive_on_noise(self):
        rng = random.Random(3)
        for _ in range(50):
            m = LoadMonitor()
            hit = any(m.push(25 + rng.gauss(0, 2)) for _ in range(300))
            self.assertFalse(hit)

    def test_step_detected(self):
        rng = random.Random(4)
        m = LoadMonitor()
        res = [m.push(25 + rng.gauss(0, 2) + (40 if i > 60 else 0)) for i in range(120)]
        first = res.index(True)
        self.assertTrue(60 < first < 80, first)

    def test_baseline_learned_with_object_recovers(self):
        """第一段一开始就推着物体（基线学高了）；第二段空载把基线拉低，第三段再推到物体能检出。"""
        rng = random.Random(5)
        m = LoadMonitor()
        self.assertFalse(any(m.push(65 + rng.gauss(0, 2)) for _ in range(80)))   # 段 1：带物体
        m.restart()
        self.assertFalse(any(m.push(25 + rng.gauss(0, 2)) for _ in range(80)))   # 段 2：空载
        self.assertLess(m.mean, 30)
        m.restart()
        res = [m.push(25 + rng.gauss(0, 2) + (40 if i > 40 else 0)) for i in range(80)]
        self.assertIn(True, res)

    def test_accel_spike_ignored(self):
        m = LoadMonitor()
        seq = [120] * 6 + [25] * 100        # 起步加速的大负载被 skip_n 跳过
        self.assertFalse(any(m.push(x) for x in seq))


class TestOdometry(unittest.TestCase):
    def test_square(self):
        o = Odometry()
        o.update(0, 0)
        o.update(500, 0)          # 向前 500
        o.update(500, 90)         # 右转 90
        o.update(800, 90)         # 再走 300（向右 = -y）
        self.assertAlmostEqual(o.x, 500, delta=1)
        self.assertAlmostEqual(o.y, -300, delta=1)
        rel, dist = o.bearing_to(0, 0)
        self.assertAlmostEqual(dist, (500 ** 2 + 300 ** 2) ** 0.5, delta=1)
        # 在 (500,-300) 朝 -y，回原点需要朝向 (-500, +300)：相对右转 ~ +121°
        self.assertAlmostEqual(rel, 120.96, delta=1)

    def test_edge_heading_ignores_corner(self):
        pts = [(0, 0), (0, 90), (0, 180), (10, 260), (100, 270), (190, 265), (280, 272)]
        self.assertAlmostEqual(straight_run_heading(pts), -1.5, delta=4)
        self.assertIsNone(straight_run_heading(pts[2:4]))

    def test_diagonal_choices(self):
        # 沿对角线 45° 撞边：不能原路返回(-135)，也不能继续向前(45)，只剩 ±90° 的两个
        c = sorted(diagonal_choices(0.0, 45.0))
        self.assertEqual(c, [-45.0, 135.0])
        for h in diagonal_choices(10.0, 97.0):
            self.assertAlmostEqual((h - 10.0 - 45.0) % 90.0, 0.0, delta=1e-6)

    def test_axis_snap(self):
        self.assertAlmostEqual(axis_snap(95.0, 0.0), -5.0)
        self.assertAlmostEqual(axis_snap(-178.0, 0.0), -2.0)
        self.assertAlmostEqual(axis_snap(33.0, 30.0), -3.0)
        self.assertAlmostEqual(axis_snap(120.0 - 7.0, 30.0), 7.0)
        o = Odometry()
        o.update(0, 0)
        o.correct_heading(10.0)
        o.update(100, 0)
        self.assertAlmostEqual(o.heading, 10.0)
        self.assertLess(o.y, 0)          # 航向 +10（右偏）-> y 变负

    def test_turn_scale(self):
        o = Odometry()
        o.update(0, 0)
        o.update(0, 90)
        o.set_turn_scale(0.5)
        o.update(0, 190)                 # 原始又转了 100 -> 计 50
        self.assertAlmostEqual(o.heading, 140.0)
        o.correct_heading(-10)
        o.update(0, 210)                 # 再 20 -> 计 10
        self.assertAlmostEqual(o.heading, 140.0)

    def test_tap_verdict(self):
        self.assertTrue(tap_verdict([40, 41, 39, 40], [55, 56, 54], 8)[0])
        self.assertFalse(tap_verdict([40, 41, 39, 40], [42, 43, 41], 8)[0])
        self.assertFalse(tap_verdict([40], [60, 60, 60], 8)[0])     # 样本不足

    def test_wrap(self):
        self.assertEqual(wrap180(190), -170)
        self.assertEqual(wrap180(-190), 170)


if __name__ == "__main__":
    unittest.main()
