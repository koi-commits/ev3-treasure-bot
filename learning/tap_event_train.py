"""Learn the final tap+seat decision from the same active evidence as Final.

The motion primitive remains production Final. The model consumes only the load,
odometry and phase sequence generated during that primitive and is evaluated at
the first confirmation that would hand control to delivery.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import time

import numpy as np
import torch
from torch import nn

from .contact_protocol import (TAP_FEATURES, attempt_event_metrics, collect_tap_attempts,
                               make_trial)
from .seeding import assert_disjoint

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TapSeatGRU(nn.Module):
    def __init__(self, hidden=12, input_size=len(TAP_FEATURES)):
        super().__init__()
        self.hidden_size = hidden
        self.input_size = input_size
        self.gru = nn.GRUCell(input_size, hidden)
        self.head = nn.Linear(hidden, 1)

    def forward(self, sequence, lengths):
        hidden = torch.zeros(sequence.shape[1], self.hidden_size, device=sequence.device)
        for t in range(sequence.shape[0]):
            proposal = self.gru(sequence[t], hidden)
            active = (t < lengths).float().unsqueeze(-1)
            hidden = proposal * active + hidden * (1.0 - active)
        return self.head(hidden).squeeze(-1)


def collect_specs(count, split, collector=collect_tap_attempts):
    specs = [make_trial(i, split) for i in range(count)]
    episodes = []
    for i, spec in enumerate(specs):
        episodes.append(collector(spec))
        if (i + 1) % 100 == 0:
            print("collect {} {}/{}".format(split, i + 1, count), flush=True)
    return specs, episodes


def flatten_attempts(episodes):
    attempts = [attempt for episode in episodes for attempt in episode]
    if not attempts:
        raise RuntimeError("no tap attempts collected")
    max_len = max(len(a["trace"]) for a in attempts)
    feature_count = attempts[0]["trace"].shape[1]
    x = np.zeros((max_len, len(attempts), feature_count), np.float32)
    lengths = np.zeros(len(attempts), np.int64)
    labels = np.zeros(len(attempts), np.float32)
    for i, attempt in enumerate(attempts):
        n = len(attempt["trace"])
        x[:n, i] = attempt["trace"]
        lengths[i] = n
        labels[i] = float(attempt["truth"])
    return x, lengths, labels, attempts


def probabilities_by_episode(model, episodes):
    nonempty = [episode for episode in episodes if episode]
    if not nonempty:
        return [[] for _ in episodes]
    x, lengths, _, flat = flatten_attempts(episodes)
    with torch.no_grad():
        probability = torch.sigmoid(model(torch.from_numpy(x),
                                          torch.from_numpy(lengths))).numpy().tolist()
    out, cursor = [], 0
    for episode in episodes:
        out.append(probability[cursor:cursor + len(episode)])
        cursor += len(episode)
    return out


def decisions_from_probability(probabilities, threshold):
    return [[p >= threshold for p in episode] for episode in probabilities]


def tune(probabilities, episodes, specs):
    choices = []
    for threshold in (0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85,
                      0.90, 0.93, 0.95, 0.97, 0.98, 0.99):
        flags = decisions_from_probability(probabilities, threshold)
        m = attempt_event_metrics(flags, episodes, specs)
        qualifies = m["first_trigger_precision"] >= 0.90 and m["triggered_episodes"] >= 5
        key = (0 if qualifies else 1,
               -m["event_recall"] if qualifies else -m["first_trigger_precision"],
               -m["first_trigger_precision"] if qualifies else -m["event_recall"])
        choices.append((key, threshold, m))
    _, threshold, metrics = min(choices, key=lambda item: item[0])
    return threshold, metrics


def train(output, train_episodes=700, val_episodes=220, test_episodes=220,
          epochs=100, seed=20261109):
    assert_disjoint()
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(min(4, os.cpu_count() or 1))

    train_specs, train_eps = collect_specs(train_episodes, "train")
    val_specs, val_eps = collect_specs(val_episodes, "val")
    test_specs, test_eps = collect_specs(test_episodes, "test")
    train_x, train_lengths, train_y, _ = flatten_attempts(train_eps)
    model = TapSeatGRU(hidden=12)
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
            print("tap epoch {:03d} loss={:.4f}".format(epoch, np.mean(losses)), flush=True)
    train_seconds = time.perf_counter() - started
    model.eval()
    val_probability = probabilities_by_episode(model, val_eps)
    test_probability = probabilities_by_episode(model, test_eps)
    threshold, val_metrics = tune(val_probability, val_eps, val_specs)
    learned_flags = decisions_from_probability(test_probability, threshold)
    rule_flags = [[a["rule_confirm"] for a in episode] for episode in test_eps]
    learned_metrics = attempt_event_metrics(learned_flags, test_eps, test_specs)
    rule_metrics = attempt_event_metrics(rule_flags, test_eps, test_specs)

    payload = {
        "state_dict": model.state_dict(), "features": TAP_FEATURES,
        "hidden_size": model.hidden_size, "seed": seed, "threshold": threshold,
        "sample_ms": 50, "model_version": "tap-seat-event-v1",
        "train_episodes": train_episodes, "val_episodes": val_episodes,
        "test_episodes": test_episodes,
    }
    os.makedirs(os.path.dirname(output), exist_ok=True)
    torch.save(payload, output)
    report = {
        "protocol": {
            "primary_metric": "world truth at first confirmation handed to delivery",
            "same_tap_attempts_for_rule_and_model": True,
            "test_split_used_for_tuning": False,
            "feature_contract": list(TAP_FEATURES),
            "train_attempts": int(len(train_y)),
            "validation_attempts": int(sum(len(e) for e in val_eps)),
            "test_attempts": int(sum(len(e) for e in test_eps)),
        },
        "operating_point": {"threshold": threshold},
        "validation_learned": {k: v for k, v in val_metrics.items() if k != "rows"},
        "test_learned_tap_sequence": learned_metrics,
        "test_final_fixed_tap_verdict": rule_metrics,
        "train_positive_fraction": positives / max(1, len(train_y)),
        "train_seconds": train_seconds,
    }
    path = os.path.splitext(output)[0] + "_metrics.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps({
        "operating_point": report["operating_point"],
        "validation_learned": report["validation_learned"],
        "test_learned_tap_sequence": {k: v for k, v in learned_metrics.items() if k != "rows"},
        "test_final_fixed_tap_verdict": {k: v for k, v in rule_metrics.items() if k != "rows"},
        "attempts": report["protocol"],
    }, indent=2, sort_keys=True))
    return model, report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", default=os.path.join(ROOT, "models", "tap_seat_gru_v1.pt"))
    p.add_argument("--train-episodes", type=int, default=700)
    p.add_argument("--val-episodes", type=int, default=220)
    p.add_argument("--test-episodes", type=int, default=220)
    p.add_argument("--epochs", type=int, default=100)
    args = p.parse_args()
    train(args.output, args.train_episodes, args.val_episodes, args.test_episodes, args.epochs)


if __name__ == "__main__":
    main()
