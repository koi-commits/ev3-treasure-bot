"""viz2.py — 用 evaluate.run_one 跑一次并画轨迹：python tests/viz2.py <mode> <seed> <png> [near]"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import evaluate as E
import sim as simmod


def run(mode, seed, png, **kw):
    path, opath, ev = [], [], []
    orig = simmod.Sim.step

    def step(self, dt):
        orig(self, dt)
        if int(self.t) % 40 == 0:
            path.append((self.x, self.y, self.t, int(self.v > 1), self.in_pocket(self.objects[0]) if self.objects else 0))
            for o in self.objects:
                opath.append((o["x"], o["y"]))
    simmod.Sim.step = step
    try:
        r = E.run_one(seed, mode=mode, **kw)
    finally:
        simmod.Sim.step = orig
    s = simmod.SIM_HOLDER.sim
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.scatter([p[0] for p in path], [p[1] for p in path], c=[p[2] for p in path], s=1, cmap="viridis")
    car = [p for p in path if p[4]]
    ax.scatter([p[0] for p in car], [p[1] for p in car], s=2, color="magenta")
    if opath:
        ax.plot([p[0] for p in opath], [p[1] for p in opath], color="orange", lw=2)
        ax.add_patch(plt.Circle(opath[0], 25, fill=False, color="orange"))
        ax.add_patch(plt.Circle(opath[-1], 25, fill=True, color="orange", alpha=0.6))
    ax.add_patch(plt.Rectangle((0, 0), s.W, s.H, fill=False))
    ax.add_patch(plt.Rectangle((-s.band, -s.band), s.W + 2 * s.band, s.H + 2 * s.band, fill=False, ls="--"))
    for m in s.markers:
        ax.add_patch(plt.Rectangle((m[0], m[1]), m[2] - m[0], m[3] - m[1], color="red", alpha=0.5))
    ax.set_aspect("equal")
    ax.set_xlim(-150, s.W + 150)
    ax.set_ylim(-150, s.H + 150)
    ax.set_title("{} seed {} | {} | success={}".format(mode, seed, r["result"][:40], r["success"]))
    plt.savefig(png, dpi=65)
    return r


if __name__ == "__main__":
    kw = {}
    if len(sys.argv) > 4 and sys.argv[4] == "near":
        kw["obj"] = E.object_near_edge
    r = run(sys.argv[1], int(sys.argv[2]), sys.argv[3], **kw)
    print("\n".join(l for l in r["out"].splitlines() if l.startswith("[")))
