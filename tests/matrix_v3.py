"""20-seed matrix for the v3 controller (offline only). Runs two scenarios x 20 seeds."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "mock"), os.path.join(HERE, "..", "src")]
os.chdir(os.path.join(HERE, ".."))

import evaluate as E  # noqa: E402


def row(name, runs):
    n = len(runs)
    succ = sum(1 for r in runs if r["success"])
    acc = sum(1 for r in runs if r["acceptable"])
    wheel = sum(1 for r in runs if r["wheel_beyond"])
    out = sum(1 for r in runs if r["object_out"])
    claim = sum(1 for r in runs if r["claim"] and not r["success"])
    ts = sorted(r["t"] for r in runs)
    med = ts[len(ts) // 2] if ts else 0.0
    print("%-18s %2d seeds | succ %2d/%2d | acc %2d | wheel_beyond %2d | obj_out %2d | false_claim %2d | median %5.0f s"
          % (name, n, succ, n, acc, wheel, out, claim, med))


def main():
    seeds = list(range(20))
    row("interior", [E.run_one(i, mode="full") for i in seeds])
    row("near_edge", [E.run_one(i, mode="full", obj=E.object_near_edge) for i in seeds])


if __name__ == "__main__":
    main()
