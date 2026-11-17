"""Reproducible offline matrix, JSON receipts and cell-centre coverage maps."""
import argparse
import json
import os
import sys
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "mock"), os.path.join(HERE, "..", "src")]
import evaluate as E

LABELS = ("lane", "carry", "marker_scan", "deliver", "other")
PHASE_LABEL = {"edge_walk": "marker_scan"}


def object_out_by_phase(runs):
    out = dict((label, 0) for label in LABELS)
    for r in runs:
        for phase in r.get("object_out_phases", {}).values():
            label = PHASE_LABEL.get(phase, phase if phase in LABELS else "other")
            out[label] += 1
    return out


def delivery_stats(runs):
    errs = [r["delivery_error"] for r in runs if r["success"]]
    if not errs:
        return "-", "-"
    errs.sort()
    return "%.1f" % errs[len(errs) // 2], "%.1f" % errs[-1]


def row(name, runs):
    n = len(runs)
    counts = [sum(bool(r[k]) for r in runs) for k in
              ("success", "acceptable", "object_out", "robot_out", "wheel_beyond", "false_claim")]
    times = [r["t"] for r in runs]
    err_med, err_max = delivery_stats(runs)
    print("| %s | %d | %s | %.0f / %.0f s | %.1f%% | %s / %s |" %
          (name, n, " | ".join(map(str, counts)), sum(times)/n, max(times),
           100*sum(r["coverage"] for r in runs)/n, err_med, err_max), flush=True)
    phases = object_out_by_phase(runs)
    print("  - object_out by phase: " + ", ".join(
        "%s=%d" % (k, phases[k]) for k in LABELS), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--spacing", type=float)
    parser.add_argument("--speed", type=float)
    parser.add_argument("--carry-speed", type=float)
    parser.add_argument("--marker-side", type=float, default=150.0)
    args = parser.parse_args()
    cfg = {k: v for k, v in (("LANE_SPACING_MM", args.spacing), ("SWEEP_SPEED", args.speed),
                             ("CARRY_SPEED", args.carry_speed),
                             ("MARKER_SIDE_MM", args.marker_side)) if v is not None}
    results = {}
    print("| scenario | n | success | acceptable | object_out | robot_out | wheel_beyond(info) "
          "| false_claim | mean / max time | coverage | delivery err med / max mm |", flush=True)
    for name, kw in (("interior", {}), ("near_edge", dict(obj=E.object_near_edge)),
                     ("marker_corner", dict(corner=True))):
        runs = [E.run_one(i, mode="full", cfg=cfg, marker_side=args.marker_side, **kw)
                for i in range(args.seeds)]
        row(name, runs)
        results[name] = runs
    Path(args.output).write_text(json.dumps(dict(config=cfg, runs=results), indent=2),
                                 encoding="utf-8")


if __name__ == "__main__":
    main()
