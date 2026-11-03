"""viz.py — 跑一次仿真并画出车/物体轨迹（离线调试用）。

    python tests/viz.py full 3 out.png [KEY=VALUE ...]
"""
import math
import random
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import sim as simmod
from harness import press_right, run_script
from sim import Sim, rect_arena_marker
from test_mission_sim import H, W, rgb_cfg


def scenario(seed, full=True, sim_kw=None, base=2000):
    rng = random.Random(base + seed)
    start = (rng.uniform(150, W - 150), rng.uniform(150, H - 150), rng.uniform(0, 2 * math.pi))
    mk = rect_arena_marker(W, H, rng.choice(["right", "top", "left", "bottom"]))
    objs = [{"x": rng.uniform(150, W - 150), "y": rng.uniform(150, H - 150), "r": 25, "load": 20}] if full else []
    return Sim(W=W, H=H, start=start, seed=seed, buttons=press_right(), markers=[mk], objects=objs,
               max_ms=700000, **(sim_kw or {})), mk


def run(seed, out_png, full=True, cfg=None, sim_kw=None):
    path, opath = [], []
    orig = simmod.Sim.step

    def step(self, dt):
        orig(self, dt)
        if int(self.t) % 50 == 0:
            path.append((self.x, self.y, self.t))
            for o in self.objects:
                opath.append((o["x"], o["y"], self.t))
    simmod.Sim.step = step
    try:
        s, mk = scenario(seed, full, sim_kw)
        c = rgb_cfg(MISSION_MODE="full" if full else "edge_to_finish")
        c.update(cfg or {})
        out, s = run_script("mission", s, seed=seed, allow_timeout=True, **c)
    finally:
        simmod.Sim.step = orig
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.scatter([p[0] for p in path], [p[1] for p in path], c=[p[2] for p in path], s=1, cmap="viridis")
    if opath:
        ax.plot([p[0] for p in opath], [p[1] for p in opath], color="orange", lw=2)
        ax.add_patch(plt.Circle((opath[0][0], opath[0][1]), 25, fill=False, color="orange"))
    ax.add_patch(plt.Rectangle((0, 0), W, H, fill=False))
    ax.add_patch(plt.Rectangle((-40, -40), W + 80, H + 80, fill=False, ls="--"))
    ax.add_patch(plt.Rectangle((mk[0], mk[1]), mk[2] - mk[0], mk[3] - mk[1], color="red", alpha=0.6))
    ax.set_aspect("equal")
    ax.set_xlim(-200, W + 200)
    ax.set_ylim(-200, H + 200)
    res = [l for l in out.splitlines() if l.startswith("result")]
    ax.set_title("seed %d  %s" % (seed, res[0] if res else "?"))
    plt.savefig(out_png, dpi=70)
    return out, s


if __name__ == "__main__":
    mode, seed, png = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    cfg = {}
    for kv in sys.argv[4:]:
        k, v = kv.split("=")
        cfg[k] = eval(v)
    out, s = run(seed, png, mode == "full", cfg)
    print("\n".join(l for l in out.splitlines() if l.startswith("[") or l.startswith("result")))
    print("objects:", [(round(o["x"]), round(o["y"]), o["out"], round(o["pushed_ms"])) for o in s.objects],
          "wheel_out", round(s.wheel_max_out), "sensor_out", round(s.max_out))
