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
        self.robot_out_phase = None
        self.carry_inplace_turns = 0
        self.swing_turns = 0
        self.swing_short = 0
        self.object_phases = {}
        self.last_sample = -100

    def step(self, sim):
        ctx = getattr(sys.modules.get("mission"), "ctx", None)
        phase = getattr(ctx, "phase", "startup")
        if sim.wheel_max_out > self.wheel_max:
            self.wheel_max = sim.wheel_max_out
            self.wheel_phase = phase
        if sim.robot_out and self.robot_out_phase is None:
            self.robot_out_phase = phase
        self.carry_inplace_turns = getattr(ctx, "carry_inplace_turns", 0)
        self.swing_turns = getattr(ctx, "swing_turns", 0)
        self.swing_short = getattr(ctx, "swing_short", 0)
        for i, obj in enumerate(sim.objects):
            if obj["out"] and i not in self.object_phases:
                self.object_phases[i] = phase
        # At most 10 mm travel between samples at the evaluated speeds.
        if sim.t - self.last_sample < 50:
            return
        self.last_sample = sim.t
        c, s = math.cos(sim.th), math.sin(sim.th)
        front = sim.g["cb"] + sim.g["wall_far"]
        half = sim.g["wall_y"]                      # = PLOW_MOUTH_MM / 2 (inner mouth)
        radius = math.sqrt(front ** 2 + half ** 2)
        for j in range(max(0, int((sim.y-radius)/self.cell)), min(self.ny, int((sim.y+radius)/self.cell)+1)):
            for i in range(max(0, int((sim.x-radius)/self.cell)), min(self.nx, int((sim.x+radius)/self.cell)+1)):
                dx = min((i+0.5)*self.cell, sim.W)-sim.x
                dy = min((j+0.5)*self.cell, sim.H)-sim.y
                if sim.g["cb"] <= dx*c+dy*s <= front and abs(-dx*s+dy*c) <= half:
                    self.swept.add((i, j))

    def result(self):
        return dict(coverage=len(self.swept)/(self.nx*self.ny),
                    coverage_grid="\n".join("".join("." if (i,j) in self.swept else "#"
                        for i in range(self.nx)) for j in reversed(range(self.ny))),
                    wheel_max_phase=self.wheel_phase, robot_out_phase=self.robot_out_phase,
                    carry_inplace_turns=self.carry_inplace_turns,
                    swing_turns=self.swing_turns, swing_short=self.swing_short,
                    object_out_phases=self.object_phases)
