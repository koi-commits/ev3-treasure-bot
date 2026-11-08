"""Re-evaluate the frozen tap-probe checkpoint on the independent test split."""

from __future__ import annotations

import argparse
import json
import math
import os

import torch

from .contact_protocol import (attempt_event_metrics, collect_probe_attempts,
                               collect_tap_attempts, fixed_probe_verdict, make_trial)
from .tap_event_train import (TapSeatGRU, decisions_from_probability,
                              probabilities_by_episode)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def wilson(successes, total, z=1.96):
    if total == 0:
        return [0.0, 0.0]
    p = successes / total
    d = 1.0 + z * z / total
    center = (p + z * z / (2 * total)) / d
    half = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / d
    return [center - half, center + half]


def evaluate(checkpoint, episodes=220, output=None, split="audit"):
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
    model = TapSeatGRU(payload["hidden_size"], len(payload["features"]))
    model.load_state_dict(payload["state_dict"])
    model.eval()
    specs = [make_trial(i, split) for i in range(episodes)]
    probe_attempts = [collect_probe_attempts(spec) for spec in specs]
    final_attempts = [collect_tap_attempts(spec) for spec in specs]
    probabilities = probabilities_by_episode(model, probe_attempts)
    learned_flags = decisions_from_probability(probabilities, payload["threshold"])
    fixed_probe_flags = [[fixed_probe_verdict(a) for a in episode]
                         for episode in probe_attempts]
    final_flags = [[a["rule_confirm"] for a in episode] for episode in final_attempts]
    learned = attempt_event_metrics(learned_flags, probe_attempts, specs)
    fixed_probe = attempt_event_metrics(fixed_probe_flags, probe_attempts, specs)
    final = attempt_event_metrics(final_flags, final_attempts, specs, truth_key="rule_truth")

    probe_flat = [a for episode in probe_attempts for a in episode]
    transition = {
        "false_to_true_during_probe": sum(not a["rule_truth"] and a["truth"] for a in probe_flat),
        "true_to_false_during_probe": sum(a["rule_truth"] and not a["truth"] for a in probe_flat),
        "true_before_and_after": sum(a["rule_truth"] and a["truth"] for a in probe_flat),
        "false_before_and_after": sum(not a["rule_truth"] and not a["truth"] for a in probe_flat),
    }
    learned["precision_wilson_95"] = wilson(learned["correct_first_triggers"],
                                             learned["triggered_episodes"])
    final["precision_wilson_95"] = wilson(final["correct_first_triggers"],
                                           final["triggered_episodes"])
    fixed_probe["precision_wilson_95"] = wilson(fixed_probe["correct_first_triggers"],
                                                 fixed_probe["triggered_episodes"])
    report = {
        "checkpoint": os.path.basename(checkpoint), "split": split,
        "episodes": episodes, "threshold_from_validation": payload["threshold"],
        "test_labels_or_seeds_used_for_tuning": False,
        "learned_tap_hold_probe": learned,
        "fixed_rule_after_hold_probe": fixed_probe,
        "final_loadmonitor_tap_seat": final,
        "probe_truth_transitions_by_attempt": transition,
    }
    if output:
        with open(output, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps({
        "learned": {k: v for k, v in learned.items() if k != "rows"},
        "final": {k: v for k, v in final.items() if k != "rows"},
        "fixed_probe": {k: v for k, v in fixed_probe.items() if k != "rows"},
        "transitions": transition,
    }, indent=2, sort_keys=True))
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", default=os.path.join(ROOT, "models", "tap_probe_gru_v1.pt"))
    p.add_argument("--episodes", type=int, default=220)
    p.add_argument("--output", default=os.path.join(ROOT, "docs", "ai_tap_probe_eval.json"))
    p.add_argument("--split", default="audit", choices=("test", "audit"))
    args = p.parse_args()
    evaluate(args.checkpoint, args.episodes, args.output, args.split)


if __name__ == "__main__":
    main()
