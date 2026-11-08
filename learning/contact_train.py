"""Train a tiny GRU to infer stable plow capture from EV3-observable history.

This scoped model is the first deployable learned component.  Simulator contact
truth is a training label only; inference sees motor loads, odometry deltas and
the commanded motion.  Test seeds and physical randomization are disjoint.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time

import numpy as np
import torch
from torch import nn

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, "tests")
if TESTS not in sys.path:
    sys.path.insert(0, TESTS)
from sim import Sim  # noqa: E402

FEATURES = ("load_l", "load_r", "load_sum_delta", "distance_delta",
            "angle_delta", "command_speed", "command_turn")


class ContactGRU(nn.Module):
    def __init__(self, hidden=12):
        super().__init__()
        self.hidden_size = hidden
        self.gru = nn.GRUCell(len(FEATURES), hidden)
        self.head = nn.Linear(hidden, 1)

    def sequence(self, x, hidden=None):
        if hidden is None:
            hidden = torch.zeros(x.shape[1], self.hidden_size, device=x.device)
        out = []
        for t in range(x.shape[0]):
            hidden = self.gru(x[t], hidden)
            out.append(self.head(hidden).squeeze(-1))
        return torch.stack(out), hidden


def generate_episode(seed, steps=320, split="train"):
    offset = {"train": 700_000, "val": 1_200_000, "test": 1_700_000}[split]
    rng = random.Random(offset + seed)
    has_object = rng.random() < 0.78
    start = (1000.0, 1000.0, 0.0)
    objects = []
    light = False
    if has_object:
        light = rng.random() < 0.30
        objects = [{
            "x": 1000 + rng.uniform(120, 380), "y": 1000 + rng.uniform(-90, 90),
            "r": rng.uniform(20, 35),
            "load": rng.uniform(6, 14) if light else rng.uniform(15, 45),
        }]
    sim = Sim(W=2000, H=2000, start=start, objects=objects, seed=seed,
              noise=rng.uniform(0.1, 0.5), load_noise=rng.uniform(1.5, 6.0),
              contact_mu=rng.uniform(0.08, 0.65), accel=rng.uniform(500, 950),
              odo_dist_scale=rng.uniform(0.96, 1.04),
              odo_turn_scale=rng.uniform(0.95, 1.05), max_ms=steps * 120)
    xs, ys = [], []
    last_d = last_a = 0.0
    baseline = None
    cmd_v, cmd_w = 170.0, 0.0
    for t in range(steps):
        if t % rng.randint(15, 35) == 0:
            cmd_v = rng.choice((100.0, 160.0, 220.0))
            cmd_w = rng.choice((-45.0, -20.0, 0.0, 20.0, 45.0))
        # Some episodes deliberately release and re-contact, forcing temporal
        # reasoning rather than a permanent episode label.
        if has_object and t > 150 and t % 90 in range(0, 8):
            cmd_v, cmd_w = -110.0, 0.0
        sim.cmd_drive(cmd_v, cmd_w)
        sim.advance(100)
        ll, lr = sim.motor_load("L"), sim.motor_load("R")
        total = ll + lr
        if baseline is None:
            baseline = total
        elif total < baseline + 8:
            baseline = 0.98 * baseline + 0.02 * total
        d, a = sim.odo_distance(), sim.odo_angle()
        xs.append((ll / 200.0, lr / 200.0, (total - baseline) / 80.0,
                   (d - last_d) / 30.0, (a - last_a) / 20.0,
                   cmd_v / 250.0, cmd_w / 120.0))
        last_d, last_a = d, a
        ys.append(float(bool(objects) and sim.in_pocket(sim.objects[0])))
    return np.asarray(xs, np.float32), np.asarray(ys, np.float32), {
        "has_object": has_object, "light": light,
    }


def make_data(count, split):
    episodes = [generate_episode(i, split=split) for i in range(count)]
    x = np.stack([e[0] for e in episodes], axis=1)
    y = np.stack([e[1] for e in episodes], axis=1)
    meta = [e[2] for e in episodes]
    return x, y, meta


def _debounce(pred, n=3):
    out = np.zeros_like(pred, dtype=bool)
    for ep in range(pred.shape[1]):
        run = 0
        for t in range(pred.shape[0]):
            run = run + 1 if pred[t, ep] else 0
            out[t, ep] = run >= n
    return out


def metrics(pred, truth, meta, debounce_n=3):
    pred = _debounce(pred, debounce_n)
    truth = truth.astype(bool)
    tp = int(np.logical_and(pred, truth).sum())
    fp = int(np.logical_and(pred, ~truth).sum())
    fn = int(np.logical_and(~pred, truth).sum())
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    false_episode = sum(bool(pred[:, i].any()) and not bool(truth[:, i].any())
                        for i in range(pred.shape[1]))
    negative_episode = sum(not bool(truth[:, i].any()) for i in range(pred.shape[1]))
    light_idx = [i for i, m in enumerate(meta) if m["light"]]
    light_recall = 0.0
    if light_idx:
        lt, lp = truth[:, light_idx], pred[:, light_idx]
        light_recall = float(np.logical_and(lt, lp).sum() / max(1, lt.sum()))
    return {
        "precision": precision, "recall": recall,
        "f1": 2 * precision * recall / max(1e-9, precision + recall),
        "false_positive_steps": fp,
        "false_confirm_episodes": false_episode,
        "negative_episodes": negative_episode,
        "light_object_recall": light_recall,
    }


def tune_operating_point(prob, truth, meta):
    """Choose threshold/debounce on validation only, favoring low false confirms."""
    choices = []
    for threshold in (0.50, 0.60, 0.70, 0.80, 0.85, 0.90, 0.95):
        for debounce_n in (3, 5, 8, 12):
            m = metrics(prob >= threshold, truth, meta, debounce_n)
            false_rate = m["false_confirm_episodes"] / max(1, m["negative_episodes"])
            # A false capture starts a destructive delivery attempt, so it is
            # costlier than a miss. Require recall >= 0.60, then minimize false
            # episode rate before maximizing F1.
            viable = m["recall"] >= 0.60
            key = (0 if viable else 1, false_rate, -m["f1"])
            choices.append((key, threshold, debounce_n, m))
    _, threshold, debounce_n, val_metrics = min(choices, key=lambda x: x[0])
    return threshold, debounce_n, val_metrics


def rule_predictions(x):
    # Comparable passive version of the existing load-margin idea: load delta
    # over 15 mNm for three consecutive samples.
    return x[:, :, 2] > (15.0 / 80.0)


def train(output, train_episodes=240, test_episodes=80, epochs=70, seed=19):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(min(4, os.cpu_count() or 1))
    train_x, train_y, _ = make_data(train_episodes, "train")
    val_x, val_y, val_meta = make_data(60, "val")
    test_x, test_y, test_meta = make_data(test_episodes, "test")
    model = ContactGRU(hidden=12)
    optimizer = torch.optim.Adam(model.parameters(), lr=2e-3)
    xt, yt = torch.from_numpy(train_x), torch.from_numpy(train_y)
    positives = float(train_y.sum())
    pos_weight = torch.tensor((train_y.size - positives) / max(1.0, positives))
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=pos_weight)
    start = time.perf_counter()
    for epoch in range(1, epochs + 1):
        order = torch.randperm(train_episodes)
        losses = []
        for i in range(0, train_episodes, 24):
            idx = order[i:i + 24]
            logits, _ = model.sequence(xt[:, idx])
            loss = loss_fn(logits, yt[:, idx])
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            losses.append(float(loss.detach()))
        if epoch == 1 or epoch % 10 == 0:
            print("contact epoch {:03d} loss={:.4f}".format(epoch, np.mean(losses)), flush=True)
    seconds = time.perf_counter() - start
    model.eval()
    with torch.no_grad():
        val_logits, _ = model.sequence(torch.from_numpy(val_x))
        logits, _ = model.sequence(torch.from_numpy(test_x))
        val_prob = torch.sigmoid(val_logits).numpy()
        test_prob = torch.sigmoid(logits).numpy()
    threshold, debounce_n, val_metrics = tune_operating_point(val_prob, val_y, val_meta)
    learned = metrics(test_prob >= threshold, test_y, test_meta, debounce_n)
    rule = metrics(rule_predictions(test_x), test_y, test_meta)
    payload = {
        "state_dict": model.state_dict(), "features": FEATURES,
        "hidden_size": model.hidden_size, "seed": seed,
        "threshold": threshold, "debounce_n": debounce_n,
        "train_episodes": train_episodes, "test_episodes": test_episodes,
    }
    os.makedirs(os.path.dirname(output), exist_ok=True)
    torch.save(payload, output)
    report = {"learned": learned, "validation": val_metrics,
              "threshold": threshold, "debounce_n": debounce_n,
              "load_margin_rule": rule, "train_seconds": seconds,
              "train_positive_fraction": positives / train_y.size,
              "test_seed_offset": 1_700_000}
    with open(os.path.splitext(output)[0] + "_metrics.json", "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))
    return model, report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", default=os.path.join(ROOT, "models", "contact_gru.pt"))
    p.add_argument("--train-episodes", type=int, default=240)
    p.add_argument("--test-episodes", type=int, default=80)
    p.add_argument("--epochs", type=int, default=70)
    args = p.parse_args()
    train(args.output, args.train_episodes, args.test_episodes, args.epochs)


if __name__ == "__main__":
    main()
