"""tools/parse_logs.py 能解析各脚本在仿真中产生的真实输出格式。"""
import os
import sys
import tempfile
import unittest

import harness  # noqa: F401
from harness import press_right, run_script
from sim import Sim, rect_arena_marker

sys.path.insert(0, os.path.join(os.path.dirname(harness.HERE), "tools"))
import parse_logs  # noqa: E402
import parse_shadow_logs  # noqa: E402


class TestParseLogs(unittest.TestCase):
    def test_parse_mission_and_demo(self):
        d = tempfile.mkdtemp()
        out, _ = run_script("mission", Sim(start=(600, 450, 0.4), seed=1, buttons=press_right()),
                            FOLLOW_MAX_MS=20000)
        open(os.path.join(d, "mission_1.txt"), "w", encoding="utf-8").write(out)
        out, _ = run_script("demo_edge", Sim(start=(600, 450, 0.0), seed=1, buttons=press_right()))
        open(os.path.join(d, "demo_edge_1.txt"), "w", encoding="utf-8").write(out)
        rows = parse_logs.parse(os.path.join(d, "mission_1.txt"))
        self.assertEqual(rows[0]["kind"], "mission")
        self.assertIn("END", rows[0]["result"])
        self.assertIn("max_white_run_mm", rows[0])
        rows = parse_logs.parse(os.path.join(d, "demo_edge_1.txt"))
        self.assertEqual(rows[0]["kind"], "demo_edge")
        self.assertIn("segments=3", rows[0]["result"])

    def test_parse_shadow_first_decisions(self):
        text = "\n".join((
            "SHADOW_RESULT version=v2 trial=1 complete=1 abort_phase=none abort_reason=none pre_label=1 post_label=1 transition=1to1 rule=1 model=1 p_post=0.97 max_infer_ms=3 max_cycle_ms=52",
            "SHADOW_RESULT version=v2 trial=2 complete=1 abort_phase=none abort_reason=none pre_label=0 post_label=1 transition=0to1 rule=1 model=1 p_post=0.92 max_infer_ms=4 max_cycle_ms=55",
            "SHADOW_RESULT version=v2 trial=3 complete=0 abort_phase=PL abort_reason=white pre_label=1 post_label=0 transition=1to0 rule=1 model=1 p_post=0.90 max_infer_ms=4 max_cycle_ms=56",
            "SHADOW_RESULT version=v2 trial=5 complete=1 abort_phase=none abort_reason=none pre_label=0 post_label=0 transition=0to0 rule=0 model=1 p_post=0.99 max_infer_ms=21 max_cycle_ms=52",
            "SHADOW_ABORT trial=4 phase=initial_back reason=white",
        ))
        report = parse_shadow_logs.summarize(text)
        self.assertEqual(report["attempted_trials"], 5)
        self.assertEqual(report["complete_trials"], 3)
        self.assertEqual(report["incomplete_trials"], 1)
        self.assertEqual(report["abort_lines"], 1)
        self.assertEqual(report["rule_vs_pre_probe"]["tp"], 1)
        self.assertEqual(report["rule_vs_pre_probe"]["fp"], 1)
        self.assertEqual(report["model_vs_post_probe"]["fp"], 0)
        self.assertEqual(report["model_vs_post_probe"]["recall"], 1.0)
        self.assertEqual(report["model_eligible_trials"], 2)
        self.assertEqual(report["model_timing_excluded_trials"], 1)
        self.assertEqual(report["transitions"]["0to1"], 1)
        self.assertEqual(report["transitions"]["1to0"], 1)
        self.assertEqual(report["complete_transitions"]["1to0"], 0)
        self.assertEqual(report["incomplete_transitions"]["1to0"], 1)
        self.assertEqual(report["drop_safety_events"], 1)
        self.assertEqual(report["incomplete_drop_safety_events"], 1)


if __name__ == "__main__":
    unittest.main()
