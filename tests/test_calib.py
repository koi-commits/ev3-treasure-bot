import random
import unittest

import harness  # noqa: F401
from calib import analyze

rng = random.Random(7)


def noisy(base, n=40, sd=0.6):
    return [tuple(max(0, int(round(c + rng.gauss(0, sd)))) for c in base) for _ in range(n)]


class TestCalib(unittest.TestCase):
    def test_pass_red_marker(self):
        rep = analyze({"floor": noisy((5, 5, 5)), "white": noisy((24, 23, 22)), "finish": noisy((20, 7, 6))},
                      {"finish": [8.0, 8.3]}, 6.35)
        self.assertTrue(rep["ok"], rep["problems"])
        self.assertTrue(any("reflection mode" in n for n in rep["notes"]))
        self.assertIn('SENSE_MODE = "rgb"', rep["lines"])
        self.assertTrue(10 < rep["rgb_edge_bright"] < 20)

    def test_fail_gray_marker(self):
        """灰色色标 = 黑白混合色，必须判 FAIL（巡线跨边时会误报终点）。"""
        rep = analyze({"floor": noisy((5, 5, 5)), "white": noisy((24, 23, 22)), "finish": noisy((14, 14, 13))})
        self.assertFalse(rep["ok"])

    def test_fail_no_contrast(self):
        rep = analyze({"floor": noisy((5, 5, 5)), "white": noisy((6, 6, 6))})
        self.assertFalse(rep["ok"])

    def test_missing(self):
        self.assertFalse(analyze({"floor": noisy((5, 5, 5))})["ok"])

    def test_generated_config_is_python(self):
        rep = analyze({"floor": noisy((5, 5, 5)), "white": noisy((24, 23, 22)), "finish": noisy((5, 8, 16))})
        ns = {}
        exec("\n".join(rep["lines"]), ns)
        self.assertEqual(set(ns["SIGNATURES"]), {"floor", "white", "finish"})


if __name__ == "__main__":
    unittest.main()
