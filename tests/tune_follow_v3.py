"""Straight wall at y=0 (white to the left); deterministic 50 ms drive kinematics."""
import itertools
import math
import random
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from fsm import EdgeFollower


def score(params, seeds=20):
    kept = 0
    progress = overshoot = twists = 0
    dt = .05
    for seed in range(seeds):
        rng = random.Random(seed)
        x, y, a = 0, -rng.uniform(1, 22), math.radians(rng.uniform(-10, 25))
        f = EdgeFollower(-1, *params)
        lost = False
        for t in range(0, 20000, 50):
            white = 0 <= y + 20 * math.sin(a) <= 40
            v, rate = f.step(t, white)
            omega = -math.radians(rate)
            # DriveBase twist -> 55.5 mm wheels, 104 mm track.
            wl, wr = (v-52*omega)/27.75, (v+52*omega)/27.75
            da = (wr-wl)*27.75/104*dt
            travel = (wr+wl)*27.75/2*dt
            x += travel*math.cos(a+da/2)
            y += travel*math.sin(a+da/2)
            a += da
            overshoot = max(overshoot, y+52*abs(math.cos(a))-40)
            lost |= t-f.last_white_t > 6000 or bool(f.exhausted)
        kept += not lost
        progress += x
        twists += f.twists
    return kept, progress/(20*seeds), overshoot, twists*1000/max(1,progress)


if __name__ == "__main__":
    grid = itertools.product((55,60,65),(18,22,26),(120,130,140),(300,350,400),(350,400,450))
    rows = [(p,score(p)) for p in grid]
    for row in sorted(rows,key=lambda r:(-r[1][0],r[1][2],-r[1][1],r[1][3]))[:3]:
        print(row)
