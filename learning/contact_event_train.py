"""Train/tune ContactGRU v2 for first-action reliability, not frame accuracy."""

from __future__ import annotations

import argparse
import json
import os
import random
import time

import numpy as np
import torch
from torch import nn

from .contact_protocol import (FEATURES, aggregate_final, event_metrics, make_trial,
                               passive_trace, run_final_controller)
from .contact_train import ContactGRU
from .seeding import assert_disjoint

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def make_dataset(count, split):
    specs = [make_trial(i, split) for i in range(count)]
    traces = [passive_trace(spec) for spec in specs]
    x = np.stack([t[0] for t in traces], axis=1)
    stable = np.stack([t[1] for t in traces], axis=1)
    pocket = np.stack([t[2] for t in traces], axis=1)
    hard = np.stack([t[3] for t in traces], axis=1)
    return x, stable, pocket, hard, specs


def tune(probability, pocket, specs):
    candidates = []
    for threshold in (0.50, 0.60, 0.70, 0.80, 0.85, 0.90, 0.93, 0.95,
                      0.97, 0.98, 0.99, 0.995):
        for debounce_n in (2, 3, 5, 8, 12):
            m = event_metrics(probability, pocket, specs, threshold, debounce_n)
            # An incorrect first trigger starts delivery, so precision is the
            # primary gate. Among >=90% precision choices, maximize event recall.
            qualifies = m["first_trigger_precision"] >= 0.90 and m["triggered_episodes"] >= 5
            key = (0 if qualifies else 1,
                   -m["event_recall"] if qualifies else -m["first_trigger_precision"],
                   -m["first_trigger_precision"] if qualifies else -m["event_recall"],
                   debounce_n)
            candidates.append((key, threshold, debounce_n, m))
    _, threshold, debounce_n, metrics = min(candidates, key=lambda x: x[0])
    return threshold, debounce_n, metrics


def train(output, train_episodes=420, val_episodes=140, test_episodes=180,
          epochs=80, seed=20261108):
    assert_disjoint()
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(min(4, os.cpu_count() or 1))
    train_x, train_y, _, train_hard, _ = make_dataset(train_episodes, "train")
    val_x, _, val_pocket, _, val_specs = make_dataset(val_episodes, "val")
    test_x, _, test_pocket, _, test_specs = make_dataset(test_episodes, "test")

    model = ContactGRU(hidden=12)
    optimizer = torch.optim.Adam(model.parameters(), lr=1.5e-3)
    xt, yt = torch.from_numpy(train_x), torch.from_numpy(train_y)
    hard = torch.from_numpy(train_hard)
    positives = float(train_y.sum())
    pos_weight = torch.tensor(min(8.0, (train_y.size - positives) / max(1.0, positives)))
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=pos_weight, reduction="none")
    started = time.perf_counter()
    for epoch in range(1, epochs + 1):
        order = torch.randperm(train_episodes)
        losses = []
        for start in range(0, train_episodes, 28):
            idx = order[start:start + 28]
            logits, _ = model.sequence(xt[:, idx])
            per_step = loss_fn(logits, yt[:, idx])
            weights = 1.0 + 4.0 * hard[:, idx] * (1.0 - yt[:, idx])
            loss = (per_step * weights).sum() / weights.sum()
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            losses.append(float(loss.detach()))
        if epoch == 1 or epoch % 10 == 0:
            print("event epoch {:03d} loss={:.4f}".format(epoch, np.mean(losses)), flush=True)
    train_seconds = time.perf_counter() - started
    model.eval()
    with torch.no_grad():
        val_prob = torch.sigmoid(model.sequence(torch.from_numpy(val_x))[0]).numpy()
        test_prob = torch.sigmoid(model.sequence(torch.from_numpy(test_x))[0]).numpy()
    threshold, debounce_n, val_metrics = tune(val_prob, val_pocket, val_specs)
    test_metrics = event_metrics(test_prob, test_pocket, test_specs, threshold, debounce_n)
    final_results = []
    for i, spec in enumerate(test_specs):
        final_results.append(run_final_controller(spec))
        if (i + 1) % 30 == 0:
            print("Final tap+seat {}/{}".format(i + 1, len(test_specs)), flush=True)
    final_metrics = aggregate_final(final_results)

    payload = {
        "state_dict": model.state_dict(), "features": FEATURES,
        "hidden_size": model.hidden_size, "seed": seed,
        "threshold": threshold, "debounce_n": debounce_n,
        "sample_ms": 100, "stable_label_samples": 3,
        "train_episodes": train_episodes, "val_episodes": val_episodes,
        "test_episodes": test_episodes, "model_version": "contact-event-v2",
    }
    os.makedirs(os.path.dirname(output), exist_ok=True)
    torch.save(payload, output)
    report = {
        "protocol": {
            "primary_metric": "world truth at first controller confirmation",
            "paired_test_specs": True, "test_split_used_for_tuning": False,
            "feature_contract": list(FEATURES), "sample_ms": 100,
            "hard_negative_weight": 5.0,
        },
        "operating_point": {"threshold": threshold, "debounce_n": debounce_n},
        "validation_learned": {k: v for k, v in val_metrics.items() if k != "rows"},
        "test_learned": test_metrics,
        "test_final_tap_seat": final_metrics,
        "train_seconds": train_seconds,
        "train_positive_fraction": positives / train_y.size,
    }
    metrics_path = os.path.splitext(output)[0] + "_metrics.json"
    with open(metrics_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps({
        "operating_point": report["operating_point"],
        "validation_learned": report["validation_learned"],
        "test_learned": {k: v for k, v in test_metrics.items() if k != "rows"},
        "test_final_tap_seat": {k: v for k, v in final_metrics.items() if k != "rows"},
    }, indent=2, sort_keys=True))
    return model, report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", default=os.path.join(ROOT, "models", "contact_gru_event_v2.pt"))
    p.add_argument("--train-episodes", type=int, default=420)
    p.add_argument("--val-episodes", type=int, default=140)
    p.add_argument("--test-episodes", type=int, default=180)
    p.add_argument("--epochs", type=int, default=80)
    args = p.parse_args()
    train(args.output, args.train_episodes, args.val_episodes, args.test_episodes, args.epochs)


if __name__ == "__main__":
    main()
