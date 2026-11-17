"""Offline ground truth only; never supplies positions to the controller."""
import math
import sys


class Diagnostics:
    def __init__(self, sim, cell=30):
        self.cell = cell
        self.nx = math.ceil(sim.W / cell)
        self.ny = math.ceil(sim.H / cell)
        self.swept = set()
        self.wheel_max = 0
        self.wheel_phase = "startup"
        self.object_phases = {}
        self.last_sample = -100

    def step(self, sim):
        ctx = getattr(sys.modules.get("mission"), "ctx", None)
        phase = getattr(ctx, "phase", "startup")
        if sim.wheel_max_out > self.wheel_max:
            self.wheel_max = sim.wheel_max_out
            self.wheel_phase = phase
        for i, obj in enumerate(sim.objects):
            if obj["out"] and i not in self.object_phases:
                self.object_phases[i] = phase
        # At most 10 mm travel between samples at the evaluated speeds.
        if sim.t - self.last_sample < 50:
            return
        self.last_sample = sim.t
        c, s = math.cos(sim.th), math.sin(sim.th)
        front = sim.g["cb"] + sim.g["wall_far"]
        radius = math.sqrt(front ** 2 + 56 ** 2)
        for j in range(max(0, int((sim.y-radius)/self.cell)), min(self.ny, int((sim.y+radius)/self.cell)+1)):
            for i in range(max(0, int((sim.x-radius)/self.cell)), min(self.nx, int((sim.x+radius)/self.cell)+1)):
                dx = min((i+0.5)*self.cell, sim.W)-sim.x
                dy = min((j+0.5)*self.cell, sim.H)-sim.y
                if sim.g["cb"] <= dx*c+dy*s <= front and abs(-dx*s+dy*c) <= 56:
                    self.swept.add((i, j))

    def result(self):
        return dict(coverage=len(self.swept)/(self.nx*self.ny),
                    coverage_grid="\n".join("".join("." if (i,j) in self.swept else "#"
                        for i in range(self.nx)) for j in reversed(range(self.ny))),
                    wheel_max_phase=self.wheel_phase, object_out_phases=self.object_phases)
