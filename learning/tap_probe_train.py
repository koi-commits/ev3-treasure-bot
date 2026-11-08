"""Train/evaluate a GRU after Final tap+seat plus a gentle hold probe."""

from __future__ import annotations

import argparse
import json
import os
import random
import time

import numpy as np
import torch
from torch import nn

from .contact_protocol import (TAP_PROBE_FEATURES, attempt_event_metrics,
                               collect_probe_attempts, collect_tap_attempts)
from .seeding import assert_disjoint
from .tap_event_train import (TapSeatGRU, collect_specs, decisions_from_probability,
                              flatten_attempts, probabilities_by_episode, tune)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def train(output, train_episodes=700, val_episodes=220, test_episodes=220,
          epochs=100, seed=20261110):
    assert_disjoint()
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(min(4, os.cpu_count() or 1))
    train_specs, train_eps = collect_specs(train_episodes, "train", collect_probe_attempts)
    val_specs, val_eps = collect_specs(val_episodes, "val", collect_probe_attempts)
    test_specs, test_eps = collect_specs(test_episodes, "test", collect_probe_attempts)
    # Separate original-Final runs: its state must not be altered by probe arcs.
    _, final_test_eps = collect_specs(test_episodes, "test", collect_tap_attempts)

    train_x, train_lengths, train_y, _ = flatten_attempts(train_eps)
    model = TapSeatGRU(hidden=12, input_size=len(TAP_PROBE_FEATURES))
    optimizer = torch.optim.Adam(model.parameters(), lr=2e-3, weight_decay=1e-5)
    positives = float(train_y.sum())
    pos_weight = torch.tensor(min(6.0, (len(train_y) - positives) / max(1.0, positives)))
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=pos_weight)
    x = torch.from_numpy(train_x)
    lengths = torch.from_numpy(train_lengths)
    y = torch.from_numpy(train_y)
    started = time.perf_counter()
    for epoch in range(1, epochs + 1):
        order = torch.randperm(len(train_y))
        losses = []
        for start in range(0, len(train_y), 32):
            idx = order[start:start + 32]
            logits = model(x[:, idx], lengths[idx])
            loss = loss_fn(logits, y[idx])
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            losses.append(float(loss.detach()))
        if epoch == 1 or epoch % 10 == 0:
            print("probe epoch {:03d} loss={:.4f}".format(epoch, np.mean(losses)), flush=True)
    train_seconds = time.perf_counter() - started
    model.eval()
    val_probability = probabilities_by_episode(model, val_eps)
    test_probability = probabilities_by_episode(model, test_eps)
    threshold, val_metrics = tune(val_probability, val_eps, val_specs)
    learned_flags = decisions_from_probability(test_probability, threshold)
    final_flags = [[a["rule_confirm"] for a in episode] for episode in final_test_eps]
    learned_metrics = attempt_event_metrics(learned_flags, test_eps, test_specs)
    final_metrics = attempt_event_metrics(final_flags, final_test_eps, test_specs,
                                          truth_key="rule_truth")

    before_true = sum(a["rule_truth"] for e in test_eps for a in e)
    retained_true = sum(a["truth"] for e in test_eps for a in e if a["rule_truth"])
    payload = {
        "state_dict": model.state_dict(), "features": TAP_PROBE_FEATURES,
        "hidden_size": model.hidden_size, "seed": seed, "threshold": threshold,
        "sample_ms": 50, "model_version": "tap-seat-hold-probe-v1",
        "train_episodes": train_episodes, "val_episodes": val_episodes,
        "test_episodes": test_episodes,
    }
    os.makedirs(os.path.dirname(output), exist_ok=True)
    torch.save(payload, output)
    report = {
        "protocol": {
            "primary_metric": "world truth at first confirmation handed to delivery",
            "probe": "tap+seat then 0.5s left arc + 0.5s right arc at 60mm/s, 20deg/s",
            "paired_test_specs": True, "separate_fresh_simulators": True,
            "test_split_used_for_tuning": False,
            "feature_contract": list(TAP_PROBE_FEATURES),
            "train_attempts": int(len(train_y)),
            "validation_attempts": int(sum(len(e) for e in val_eps)),
            "test_probe_attempts": int(sum(len(e) for e in test_eps)),
            "test_final_attempts": int(sum(len(e) for e in final_test_eps)),
        },
        "operating_point": {"threshold": threshold},
        "validation_learned": {k: v for k, v in val_metrics.items() if k != "rows"},
        "test_learned_probe": learned_metrics,
        "test_final_tap_seat": final_metrics,
        "probe_retention": {
            "captured_before_probe_attempts": before_true,
            "still_captured_after_probe_attempts": retained_true,
            "retention_rate": retained_true / max(1, before_true),
        },
        "train_positive_fraction": positives / max(1, len(train_y)),
        "train_seconds": train_seconds,
    }
    with open(os.path.splitext(output)[0] + "_metrics.json", "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps({
        "operating_point": report["operating_point"],
        "validation_learned": report["validation_learned"],
        "test_learned_probe": {k: v for k, v in learned_metrics.items() if k != "rows"},
        "test_final_tap_seat": {k: v for k, v in final_metrics.items() if k != "rows"},
        "probe_retention": report["probe_retention"],
    }, indent=2, sort_keys=True))
    return model, report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", default=os.path.join(ROOT, "models", "tap_probe_gru_v1.pt"))
    p.add_argument("--train-episodes", type=int, default=700)
    p.add_argument("--val-episodes", type=int, default=220)
    p.add_argument("--test-episodes", type=int, default=220)
    p.add_argument("--epochs", type=int, default=100)
    args = p.parse_args()
    train(args.output, args.train_episodes, args.val_episodes, args.test_episodes, args.epochs)


if __name__ == "__main__":
    main()
