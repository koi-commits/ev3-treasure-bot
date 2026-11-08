"""Paired contact trials and production-equivalent Final tap+seat evaluation.

Each controller gets a fresh simulator built from the same immutable trial spec.
Simulator truth is returned only to the evaluator and never appears in features.
"""

from __future__ import annotations

import math
import os
import random
import sys

import numpy as np

from .seeding import derive_seed

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, "tests")
SRC = os.path.join(ROOT, "src")
MOCK = os.path.join(TESTS, "mock")
for _path in (MOCK, TESTS, SRC):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import config  # noqa: E402
from contact_features import TAP_PROBE_FEATURES, tap_probe_feature  # noqa: E402
from fsm import LoadMonitor, tap_verdict  # noqa: E402
from sim import Sim  # noqa: E402

SAMPLE_MS = 100
STABLE_N = 3
FEATURES = ("abs_load_l", "abs_load_r", "load_sum_minus_causal_baseline",
            "odometry_distance_delta", "odometry_angle_delta",
            "commanded_speed", "commanded_turn_rate")
TAP_FEATURES = ("abs_load_l", "abs_load_r", "odometry_step",
                "tap_progress", "phase_free", "phase_early", "phase_late")


def make_trial(index, split="train"):
    rng = random.Random(derive_seed(split, index, "scene"))
    p = rng.random()
    if p < 0.22:
        kind = "none"
    elif p < 0.32:
        kind = "immovable"
    else:
        kind = "movable"
    obj = None
    light = False
    if kind != "none":
        light = kind == "movable" and rng.random() < 0.30
        obj = {
            "x": 500.0 + rng.uniform(150, 620),
            "y": 800.0 + rng.uniform(-125, 125),
            "r": rng.uniform(20, 35),
            "load": rng.uniform(6, 14) if light else rng.uniform(15, 45),
        }
        if kind == "immovable":
            obj["immovable"] = True
    return {
        "index": int(index), "split": split, "kind": kind, "light": light,
        "object": obj,
        "sim_seed": derive_seed(split, index, "physics"),
        "sim_kw": {
            "load_noise": rng.uniform(1.5, 6.0),
            "contact_mu": rng.uniform(0.08, 0.65),
            "accel": rng.uniform(500, 950),
            "odo_dist_scale": rng.uniform(0.96, 1.04),
            "odo_turn_scale": rng.uniform(0.95, 1.05),
        },
    }


def make_sim(spec, max_ms=40000):
    objects = [] if spec["object"] is None else [spec["object"]]
    return Sim(W=3200, H=1600, start=(500, 800, 0.0), objects=objects,
               seed=spec["sim_seed"], max_ms=max_ms, **spec["sim_kw"])


class FeatureBuilder:
    """Same causal calculations used by the EV3 shadow-mode collector."""

    def __init__(self):
        self.last_d = 0.0
        self.last_a = 0.0
        self.baseline = None

    def sample(self, sim, cmd_v, cmd_w):
        ll, lr = abs(sim.motor_load("L")), abs(sim.motor_load("R"))
        total = ll + lr
        if self.baseline is None:
            self.baseline = total
        elif total < self.baseline + 8.0:
            self.baseline = 0.98 * self.baseline + 0.02 * total
        d, a = float(sim.odo_distance()), float(sim.odo_angle())
        feature = (ll / 200.0, lr / 200.0, (total - self.baseline) / 80.0,
                   (d - self.last_d) / 30.0, (a - self.last_a) / 20.0,
                   cmd_v / 250.0, cmd_w / 120.0)
        self.last_d, self.last_a = d, a
        return feature


def passive_trace(spec, steps=260):
    """Straight search trace at the deployment model's native 100 ms rate."""
    sim = make_sim(spec, max_ms=steps * SAMPLE_MS + 1000)
    builder = FeatureBuilder()
    xs, pocket, stable, hard = [], [], [], []
    stable_run = 0
    cmd_v, cmd_w = float(config.SWEEP_SPEED), 0.0
    for _ in range(steps):
        sim.cmd_drive(cmd_v, cmd_w)
        sim.advance(SAMPLE_MS)
        xs.append(builder.sample(sim, cmd_v, cmd_w))
        captured = bool(sim.objects and not sim.objects[0].get("immovable") and
                        sim.in_pocket(sim.objects[0]))
        stable_run = stable_run + 1 if captured else 0
        pocket.append(captured)
        stable.append(stable_run >= STABLE_N)
        hard.append(bool((sim.push_load > 0 or sim.stalled) and stable_run < STABLE_N))
    return (np.asarray(xs, np.float32), np.asarray(stable, np.float32),
            np.asarray(pocket, np.bool_), np.asarray(hard, np.float32))


def _advance_until_done(sim, timeout_ms):
    elapsed = 0
    while not sim.maneuver_done() and elapsed < timeout_ms:
        sim.advance(20)
        elapsed += 20


def tap_transaction(sim):
    """Run production tap+seat and return rule verdict, truth, and observable trace."""
    sim.cmd_stop("brake")
    sim.advance(150)
    sim.cmd_straight(-config.TAP_BACK_MM, 200)
    _advance_until_done(sim, 3500)
    sim.advance(150)
    d0 = sim.odo_distance()
    last_d = d0
    free, early, late = [], [], []
    trace = []
    sim.cmd_drive(config.TAP_SPEED, 0)
    started = sim.t
    while True:
        sim.advance(config.SAMPLE_PERIOD_MS)
        distance = sim.odo_distance()
        trav = abs(distance - d0)
        ll, lr = abs(sim.motor_load("L")), abs(sim.motor_load("R"))
        load = ll + lr
        is_free = 8 <= trav <= config.TAP_BACK_MM * 0.5
        is_early = config.TAP_BACK_MM + 15 <= trav < config.TAP_BACK_MM + config.TAP_SEAT_MM * 0.5
        is_late = trav >= config.TAP_BACK_MM + config.TAP_SEAT_MM * 0.6
        trace.append(tap_probe_feature(
            ll, lr, distance - last_d,
            trav / (config.TAP_BACK_MM + config.TAP_SEAT_MM),
            is_free, is_early, is_late)[:7])
        last_d = distance
        if is_free:
            free.append(load)
        elif is_early:
            early.append(load)
        elif is_late:
            late.append(load)
        if trav >= config.TAP_BACK_MM + config.TAP_SEAT_MM or sim.t - started >= 6000:
            break
    sim.cmd_stop("brake")
    ok1, d1 = tap_verdict(free, early, config.TAP_MARGIN)
    ok2, d2 = tap_verdict(free, late, config.TAP_MARGIN)
    completed = trav >= config.TAP_BACK_MM + config.TAP_SEAT_MM
    truth = bool(sim.objects and not sim.objects[0].get("immovable") and
                 sim.in_pocket(sim.objects[0]))
    return {"rule_confirm": completed and ok1 and ok2, "truth": truth,
            "rule_truth": truth,
            "completed": completed, "d_early": d1, "d_late": d2,
            "trace": np.asarray(trace, np.float32), "decision_ms": sim.t}


def tap_probe_transaction(sim):
    """Final tap+seat plus a gentle 1 s left/right hold probe.

    The probe supplies new real-observable evidence. The learned decision is made
    after it, so truth is evaluated after it as well; rule_truth preserves the
    original Final decision instant for auditing.
    """
    tx = tap_transaction(sim)
    trace = [tuple(row) + (0.0, 0.0) for row in tx["trace"]]
    last_d = sim.odo_distance()
    for turn, flags in ((-20.0, (1.0, 0.0)), (20.0, (0.0, 1.0))):
        sim.cmd_drive(config.TAP_SPEED, turn)
        for _ in range(10):
            sim.advance(config.SAMPLE_PERIOD_MS)
            d = sim.odo_distance()
            ll, lr = abs(sim.motor_load("L")), abs(sim.motor_load("R"))
            trace.append(tap_probe_feature(ll, lr, d - last_d, 1.0,
                                           probe_left=flags[0], probe_right=flags[1]))
            last_d = d
    sim.cmd_stop("brake")
    tx["trace"] = np.asarray(trace, np.float32)
    tx["truth"] = bool(sim.objects and not sim.objects[0].get("immovable") and
                       sim.in_pocket(sim.objects[0]))
    tx["decision_ms"] = sim.t
    tx["probe_retained"] = tx["truth"]
    return tx


def final_tap_seat(sim):
    """Backward-compatible tuple used by the Final controller evaluator."""
    tx = tap_transaction(sim)
    return tx["rule_confirm"], tx["d_early"], tx["d_late"]


def run_final_controller(spec, max_search_ms=26000):
    """First delivery-trigger event for Final LoadMonitor + full tap+seat."""
    sim = make_sim(spec, max_ms=max_search_ms + 120000)
    mon = LoadMonitor(config.LOAD_SKIP_N, config.LOAD_LEARN_N, config.LOAD_WINDOW,
                      config.LOAD_K, config.LOAD_ABS_MARGIN, config.LOAD_HOLD_N)
    mon.restart()
    stalled_ms = 0
    suspects = 0
    ever_pocket = False
    cmd_v = float(config.SWEEP_SPEED)
    while sim.t < max_search_ms:
        sim.cmd_drive(cmd_v, 0)
        sim.advance(config.SAMPLE_PERIOD_MS)
        captured = bool(sim.objects and not sim.objects[0].get("immovable") and
                        sim.in_pocket(sim.objects[0]))
        ever_pocket = ever_pocket or captured
        stalled_ms = stalled_ms + config.SAMPLE_PERIOD_MS if sim.stalled else 0
        load = abs(sim.motor_load("L")) + abs(sim.motor_load("R"))
        suspect = stalled_ms >= config.STALL_MS or mon.push(load)
        if not suspect:
            continue
        suspects += 1
        if stalled_ms >= config.STALL_MS:
            sim.cmd_straight(-20, 200)
            _advance_until_done(sim, 2500)
        ok, d1, d2 = final_tap_seat(sim)
        captured_at_action = bool(sim.objects and not sim.objects[0].get("immovable") and
                                  sim.in_pocket(sim.objects[0]))
        ever_pocket = ever_pocket or captured_at_action
        if ok:
            return {
                "triggered": True, "correct": captured_at_action,
                "trigger_ms": sim.t, "ever_pocket": ever_pocket,
                "suspects": suspects, "d_early": d1, "d_late": d2,
            }
        mon.restart()
        stalled_ms = 0
    return {"triggered": False, "correct": False, "trigger_ms": None,
            "ever_pocket": ever_pocket, "suspects": suspects}


def first_trigger(probability, pocket, threshold, debounce_n):
    run = 0
    for i, p in enumerate(probability):
        run = run + 1 if p >= threshold else 0
        if run >= debounce_n:
            return i, bool(pocket[i])
    return None, False


def event_metrics(probability, pocket, specs, threshold, debounce_n):
    triggered = correct = wrong = positives = correct_light = light_positives = 0
    delays = []
    rows = []
    for ep, spec in enumerate(specs):
        truth = pocket[:, ep]
        positives += int(truth.any())
        if spec["light"] and truth.any():
            light_positives += 1
        idx, is_correct = first_trigger(probability[:, ep], truth, threshold, debounce_n)
        if idx is not None:
            triggered += 1
            correct += int(is_correct)
            wrong += int(not is_correct)
            if is_correct:
                first_true = int(np.flatnonzero(truth)[0])
                delays.append((idx - first_true) * SAMPLE_MS)
                correct_light += int(spec["light"])
        rows.append({"index": spec["index"], "kind": spec["kind"],
                     "light": spec["light"], "ever_pocket": bool(truth.any()),
                     "trigger_index": idx, "correct_at_first_trigger": is_correct})
    return {
        "episodes": len(specs), "positive_episodes": positives,
        "triggered_episodes": triggered, "correct_first_triggers": correct,
        "wrong_or_early_first_triggers": wrong,
        "first_trigger_precision": correct / max(1, triggered),
        "event_recall": correct / max(1, positives),
        "missed_positive_episodes": positives - correct,
        "light_event_recall": correct_light / max(1, light_positives),
        "median_correct_delay_ms": float(np.median(delays)) if delays else None,
        "rows": rows,
    }


def aggregate_final(results):
    triggered = sum(r["triggered"] for r in results)
    correct = sum(r["correct"] for r in results)
    positives = sum(r["ever_pocket"] for r in results)
    return {
        "episodes": len(results), "positive_episodes": positives,
        "triggered_episodes": triggered, "correct_first_triggers": correct,
        "wrong_or_early_first_triggers": triggered - correct,
        "first_trigger_precision": correct / max(1, triggered),
        "event_recall": correct / max(1, positives),
        "missed_positive_episodes": positives - correct,
        "median_trigger_ms": float(np.median([r["trigger_ms"] for r in results
                                               if r["trigger_ms"] is not None]))
                             if triggered else None,
        "rows": results,
    }


def _collect_attempts(spec, transaction, max_search_ms=26000, max_attempts=8):
    """Collect the shared sequence of active confirmations for rule/model A/B.

    The search continues after every transaction solely to expose later attempts.
    A real controller would stop at its own first confirmation, so only the prefix
    through that event is used by first-trigger metrics.
    """
    sim = make_sim(spec, max_ms=max_search_ms + 120000)
    mon = LoadMonitor(config.LOAD_SKIP_N, config.LOAD_LEARN_N, config.LOAD_WINDOW,
                      config.LOAD_K, config.LOAD_ABS_MARGIN, config.LOAD_HOLD_N)
    stalled_ms = 0
    attempts = []
    while sim.t < max_search_ms and len(attempts) < max_attempts:
        sim.cmd_drive(config.SWEEP_SPEED, 0)
        sim.advance(config.SAMPLE_PERIOD_MS)
        stalled_ms = stalled_ms + config.SAMPLE_PERIOD_MS if sim.stalled else 0
        load = abs(sim.motor_load("L")) + abs(sim.motor_load("R"))
        if stalled_ms < config.STALL_MS and not mon.push(load):
            continue
        if stalled_ms >= config.STALL_MS:
            sim.cmd_straight(-20, 200)
            _advance_until_done(sim, 2500)
        attempts.append(transaction(sim))
        mon.restart()
        stalled_ms = 0
    return attempts


def collect_tap_attempts(spec, max_search_ms=26000, max_attempts=8):
    return _collect_attempts(spec, tap_transaction, max_search_ms, max_attempts)


def collect_probe_attempts(spec, max_search_ms=26000, max_attempts=8):
    return _collect_attempts(spec, tap_probe_transaction, max_search_ms, max_attempts)


def attempt_event_metrics(decisions, attempts_by_episode, specs, truth_key="truth"):
    """Episode metric for one boolean decision per tap transaction."""
    triggered = correct = positives = 0
    rows = []
    for spec, attempts, flags in zip(specs, attempts_by_episode, decisions):
        any_truth = any(a[truth_key] for a in attempts)
        positives += int(any_truth)
        first = next((i for i, flag in enumerate(flags) if flag), None)
        is_correct = first is not None and attempts[first][truth_key]
        triggered += int(first is not None)
        correct += int(is_correct)
        rows.append({"index": spec["index"], "kind": spec["kind"],
                     "attempts": len(attempts), "any_true_attempt": any_truth,
                     "first_confirm_attempt": first,
                     "correct_at_first_confirm": bool(is_correct)})
    return {
        "episodes": len(specs), "positive_episodes": positives,
        "triggered_episodes": triggered, "correct_first_triggers": correct,
        "wrong_or_early_first_triggers": triggered - correct,
        "first_trigger_precision": correct / max(1, triggered),
        "event_recall": correct / max(1, positives),
        "missed_positive_episodes": positives - correct,
        "rows": rows,
    }


def fixed_probe_verdict(attempt):
    """Non-learned ablation: free load versus each hold-probe half."""
    trace = attempt["trace"]
    free = [(row[0] + row[1]) * 200.0 for row in trace if row[4] > 0.5]
    left_probe = [(row[0] + row[1]) * 200.0 for row in trace if len(row) > 7 and row[7] > 0.5]
    right_probe = [(row[0] + row[1]) * 200.0 for row in trace if len(row) > 8 and row[8] > 0.5]
    ok_left, _ = tap_verdict(free, left_probe, config.TAP_MARGIN)
    ok_right, _ = tap_verdict(free, right_probe, config.TAP_MARGIN)
    return ok_left and ok_right

