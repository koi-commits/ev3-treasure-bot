"""harness.py — 在仿真里原样运行 src/ 下的脚本。"""

import contextlib
import importlib
import io
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "src")
for p in (os.path.join(HERE, "mock"), HERE, SRC):
    if p not in sys.path:
        sys.path.insert(0, p)

PROJECT_MODULES = ("config", "hardware", "common", "edge", "fsm", "sensing",
                   "mission", "demo_edge", "calibrate_marker", "learned_capture",
                   "contact_inference", "contact_weights")


def fresh_config(**overrides):
    for m in PROJECT_MODULES:
        sys.modules.pop(m, None)
    cfg = importlib.import_module("config")
    for k, v in overrides.items():
        if not hasattr(cfg, k):
            raise AttributeError("config has no " + k)
        setattr(cfg, k, v)
    return cfg


def press_right(t0=100, t1=300):
    from pybricks.parameters import Button
    return [(t0, t1, Button.RIGHT)]


def run_script(name, sim, seed=0, quiet=True, allow_timeout=False, **overrides):
    """新建 sim 后调用：重新 import config（施加覆盖）再 import 脚本（脚本末尾 run_task 会跑完）。
    返回 (stdout 文本, sim)。"""
    import sim as simmod
    simmod.SIM_HOLDER.sim = sim
    fresh_config(**overrides)
    random.seed(seed)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            importlib.import_module(name)
        except simmod.SimTimeout:
            if not allow_timeout:
                raise
            print("(sim timeout)")
    out = buf.getvalue()
    if not quiet:
        print(out)
    return out, sim
