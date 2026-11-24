"""Straight wall at y=0 (white to the left); deterministic 50 ms drive kinematics.

    python tests/tune_follow_v3.py                 # original: spot 20 mm ahead, centred
    python tests/tune_follow_v3.py --spot 67 32    # spot 67 mm ahead, 32 mm LEFT (+ = left);
        searches (approach, escape, clear) at FOLLOW_SPEED / EDGE_TURN_RATE so the result
        fits a config.FOLLOW_PROFILES entry
"""
import itertools
import math
import random
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from fsm import EdgeFollower


def score(params, seeds=20, spot=None):
    """spot=None: the original model (spot 20 mm ahead, centred, robot centre
    1-22 mm short of the band). spot=(ahead, left): that spot, starting 1-22 mm
    short of the band."""
    kept = 0
    progress = overshoot = twists = 0
    dt = .05
    for seed in range(seeds):
        rng = random.Random(seed)
        x, y, a = 0, -rng.uniform(1, 22), math.radians(rng.uniform(-10, 25))
        if spot is not None:
            y -= spot[0] * math.sin(a) + spot[1] * math.cos(a)
        f = EdgeFollower(-1, *params)
        lost = False
        for t in range(0, 20000, 50):
            if spot is None:
                white = 0 <= y + 20 * math.sin(a) <= 40
            else:
                white = 0 <= y + spot[0] * math.sin(a) + spot[1] * math.cos(a) <= 40
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


def rank(rows):
    return sorted(rows, key=lambda r: (-r[1][0], r[1][2], -r[1][1], r[1][3]))


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--spot":
        sys.path.insert(0, str(Path(__file__).resolve().parent / "mock"))   # config imports pybricks
        import config
        spot = (float(sys.argv[2]), float(sys.argv[3]))
        sp, rate = config.FOLLOW_SPEED, config.EDGE_TURN_RATE
        print("spot", spot, "speed", sp, "turn", rate)
        for name, (st, esc, clr) in sorted(config.FOLLOW_PROFILES.items()):
            print("profile", name, (st, esc, clr), score((sp, st, rate, esc, clr), spot=spot))
        grid = itertools.product((10, 14, 18, 22, 26, 30, 40, 50), (200, 250, 300, 350, 400, 500, 600),
                                 (150, 200, 250, 300, 350, 400, 450))
        rows = [((st, esc, clr), score((sp, st, rate, esc, clr), spot=spot)) for st, esc, clr in grid]
        for row in rank(rows)[:5]:
            print(row)
    else:
        grid = itertools.product((55,60,65),(18,22,26),(120,130,140),(300,350,400),(350,400,450))
        rows = [(p,score(p)) for p in grid]
        for row in rank(rows)[:3]:
            print(row)
