"""在 MicroPython 下用仿真跑真正的 src/ 脚本。

    MICROPYPATH=tests/mock:tests:src micropython tests/mpy_sim_check.py [edge|full|demo]
"""
import sys

import sim
from sim import FLOOR, RED_MARK, WHITE, Sim, rect_arena_marker
from fsm import build_signature
from pybricks.parameters import Button


def sig(surf):
    s = build_signature("x", [surf["rgb"]] * 10)
    return {"mean": s.mean, "tol": s.tol}


mode = sys.argv[1] if len(sys.argv) > 1 else "edge"
W, H = 1200, 900
objs = [{"x": 700, "y": 600, "r": 25, "load": 40}] if mode == "full" else [{"x": 250, "y": 650, "r": 25, "load": 40}] if mode == "full2" else []
s = Sim(W=W, H=H, start=(400, 300, 0.7), seed=3, buttons=[(100, 300, Button.RIGHT)],
        markers=[rect_arena_marker(W, H, "right")], objects=objs, max_ms=700000)
import config
if mode == "demo":
    import demo_edge  # noqa: F401
else:
    config.SENSE_MODE = "rgb"
    config.RGB_EDGE_BRIGHT = 14.0
    config.SIGNATURES = {"floor": sig(FLOOR), "white": sig(WHITE), "finish": sig(RED_MARK)}
    config.FOLLOW_MAX_MS = 150000
    config.MISSION_MAX_MS = 600000
    if mode.startswith("full"):
        config.MISSION_MODE = "full"
    import mission  # noqa: F401  （导入即运行 run_task(main())）
print("sim t =", s.t, "max_out =", s.max_out)
for o in s.objects:
    print("object final", int(o["x"]), int(o["y"]), "marker x>=1140, 375<=y<=525")
