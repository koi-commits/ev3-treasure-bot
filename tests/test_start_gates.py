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

    def test_tap_shadow_does_not_auto_start(self):
        self.assert_stationary_without_right('probe_tap_shadow')

    def test_tap_shadow_refuses_white_start(self):
        s = Sim(start=(-80, 400, 0), buttons=[(100, 300, Button.RIGHT)], max_ms=2500)
        out, s = run_script('probe_tap_shadow', s, allow_timeout=True)
        self.assertIn('SHADOW_ABORT trial=1 condition=EMPTY phase=start reason=white', out)
        self.assertAlmostEqual(s.dist_acc, 0, places=3)

    def test_tap_shadow_monitors_white_while_backing(self):
        from sim import WHITE
        marker = (525, 0, 545, 900, WHITE)
        s = Sim(start=(500, 400, 0), markers=[marker],
                buttons=[(100, 300, Button.RIGHT)], max_ms=4000)
        out, s = run_script('probe_tap_shadow', s, allow_timeout=True)
        self.assertIn('SHADOW_ABORT trial=1 condition=EMPTY phase=initial_back reason=white', out)
        self.assertLess(abs(s.dist_acc), 50)

    def test_tap_shadow_aborts_stalled_tap(self):
        obj = {"x": 630, "y": 400, "r": 25, "load": 30, "immovable": True}
        s = Sim(start=(500, 400, 0), objects=[obj],
                buttons=[(100, 300, Button.RIGHT)], max_ms=5000)
        out, s = run_script('probe_tap_shadow', s, allow_timeout=True)
        self.assertIn('SHADOW_ABORT trial=1 condition=EMPTY phase=tap_forward reason=stalled', out)

    def test_tap_shadow_checks_white_during_probe_and_keeps_two_labels(self):
        R = Button.RIGHT
        s = Sim(W=700, H=900, start=(500, 400, 0), max_ms=8000, buttons=[
            (100, 300, R),       # authorize tap
            (4400, 4600, R),     # PRE label = captured
            (5000, 5200, R),     # separate probe authorization
            (5900, 6100, R),     # POST label
        ])
        out, s = run_script('probe_tap_shadow', s, allow_timeout=True)
        self.assertIn('complete=0', out)
        self.assertIn('abort_phase=PL abort_reason=white', out)
        self.assertIn('pre_label=1 post_label=1 transition=1to1', out)

    def test_tap_shadow_aborts_probe_stall(self):
        R = Button.RIGHT
        obstacle = (725, 350, 800, 450)
        s = Sim(start=(500, 400, 0), obstacles=[obstacle], max_ms=8000, buttons=[
            (100, 300, R),
            (4400, 4600, R),
            (5000, 5200, R),
            (5900, 6100, R),
        ])
        out, s = run_script('probe_tap_shadow', s, allow_timeout=True)
        self.assertIn('complete=0', out)
        self.assertRegex(out, r'abort_phase=P[LR] abort_reason=(stalled|no_progress)')

    def test_tap_shadow_completes_authorized_two_label_trial(self):
        R = Button.RIGHT
        s = Sim(start=(500, 400, 0), max_ms=9000, buttons=[
            (100, 300, R),
            (4400, 4600, R),
            (5000, 5200, R),
            (6800, 7000, R),
        ])
        out, s = run_script('probe_tap_shadow', s, allow_timeout=True)
        self.assertIn('SHADOW_LABEL stage=PRE_PROBE label=1', out)
        self.assertIn('SHADOW_LABEL stage=POST_PROBE label=1', out)
        self.assertIn('SHADOW_RESULT version=tap-shadow-v2 trial=1 condition=EMPTY complete=1', out)
        self.assertIn('transition=1to1', out)

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
