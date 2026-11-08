"""Paired, ground-truth comparison of Final and the learned recurrent policy."""

from __future__ import annotations

import argparse
import json
import os

import numpy as np
import torch

from .baseline import run_final_scenario
from .env import TreasureEnv, make_scenario
from .model import load_checkpoint

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def aggregate(runs):
    n = len(runs)
    out = {"episodes": n}
    for key in ("success", "acceptable", "wheel_on_white", "wheel_beyond",
                "object_out", "false_claim"):
        out[key] = sum(bool(r.get(key)) for r in runs) / max(1, n)
    out["median_time_s"] = float(np.median([r["time_s"] for r in runs]))
    return out


@torch.no_grad()
def run_learned(model, scenario, total_budget_ms=300000):
    env = TreasureEnv(scenario["seed"], "test", 1.0, max_steps=1500,
                      scenario=scenario, pre_map=True,
                      total_budget_ms=total_budget_ms)
    obs = env.last_obs
    hidden = torch.zeros(1, model.hidden_size)
    done, info = False, env.result()
    while not done:
        logits, _, hidden = model.step(torch.from_numpy(obs).unsqueeze(0), hidden)
        obs, _, done, info = env.step(logits.argmax(-1).item())
    return info


def compare(checkpoint, episodes=24, output=None):
    model, meta = load_checkpoint(checkpoint)
    model.eval()
    final_runs, learned_runs, pairs = [], [], []
    for seed in range(episodes):
        scenario = make_scenario(seed, "test", 1.0)
        final, _ = run_final_scenario(scenario, mission_max_ms=300000)
        learned = run_learned(model, scenario)
        final_runs.append(final)
        learned_runs.append(learned)
        pairs.append({"seed": seed, "final": final, "learned": learned})
        print("pair {:02d}/{:02d} final={} learned={} out={}/{}".format(
            seed + 1, episodes, int(final["success"]), int(learned["success"]),
            int(final["object_out"]), int(learned["object_out"])), flush=True)
    report = {
        "protocol": {
            "split": "test", "paired_scenes": True, "budget_s": 300,
            "world_truth_success": True, "difficulty": 1.0,
            "learned_architecture": "Final calib scout + greedy GRU low-level policy",
        },
        "final": aggregate(final_runs), "learned": aggregate(learned_runs),
        "pairs": pairs, "checkpoint_metadata": meta,
    }
    if output:
        with open(output, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps({"final": report["final"], "learned": report["learned"]},
                     indent=2, sort_keys=True))
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", default=os.path.join(ROOT, "models", "gru_policy.pt"))
    p.add_argument("--episodes", type=int, default=24)
    p.add_argument("--output", default=os.path.join(ROOT, "docs", "ai_full_eval.json"))
    args = p.parse_args()
    compare(args.checkpoint, args.episodes, args.output)


if __name__ == "__main__":
    main()
