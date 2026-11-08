"""实机行动授权与电机脚本 await 回归：只有 RIGHT 明确按下才允许运动。"""
import unittest
from harness import run_script
from sim import Sim
from pybricks.parameters import Button


class TestExplicitStart(unittest.TestCase):
    def assert_stationary_without_right(self, script, **config_overrides):
        s = Sim(start=(500, 400, 0), buttons=[], max_ms=9000)
        out, s = run_script(script, s, allow_timeout=True, **config_overrides)
        self.assertIn('(sim timeout)', out)
        self.assertAlmostEqual(s.x, 500, places=3)
        self.assertAlmostEqual(s.y, 400, places=3)
        self.assertAlmostEqual(s.dist_acc, 0, places=3)
        self.assertAlmostEqual(s.ang_acc, 0, places=3)
        self.assertIsNone(s.man)

    def test_mission_does_not_auto_start(self):
        self.assert_stationary_without_right('mission')

    def test_motor_test_does_not_auto_start(self):
        self.assert_stationary_without_right('test_motors')

    def test_load_probe_does_not_auto_start(self):
        self.assert_stationary_without_right('probe_load')

    def test_learned_capture_does_not_auto_start(self):
        self.assert_stationary_without_right('learned_capture')

    def test_motor_test_waits_for_actions(self):
        R = Button.RIGHT
        s = Sim(start=(500, 400, 0), max_ms=90000, buttons=[
            (100, 300, R), (10000, 10200, R), (25000, 25200, R)])
        out, s = run_script('test_motors', s, allow_timeout=True)
        self.assertIn('=== done ===', out)
        self.assertNotIn('(sim timeout)', out)
        self.assertRegex(out, r'fwd -> distance (19[89]|200) mm')
        self.assertRegex(out, r'turn L -> angle (8[89]|90) deg')


if __name__ == '__main__':
    unittest.main()
