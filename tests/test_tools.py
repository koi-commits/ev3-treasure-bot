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
            "SHADOW_RESULT trial=1 label=1 rule=1 model=1 p=0.97 d_early=10 d_late=11 max_infer_ms=3",
            "SHADOW_RESULT trial=2 label=0 rule=1 model=0 p=0.12 d_early=9 d_late=10 max_infer_ms=4",
        ))
        report = parse_shadow_logs.summarize(text)
        self.assertEqual(report["trials"], 2)
        self.assertEqual(report["rule"]["fp"], 1)
        self.assertEqual(report["model"]["fp"], 0)
        self.assertEqual(report["model"]["recall"], 1.0)


if __name__ == "__main__":
    unittest.main()
