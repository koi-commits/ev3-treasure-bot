"""Run the frozen Final controller and optionally record imitation sequences."""

from __future__ import annotations

import math
import os
import random
import sys

import numpy as np

from .env import CONTROL_MS, OdomObserver, action_from_command

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, "tests")
SRC = os.path.join(ROOT, "src")
for _path in (TESTS, SRC):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from evaluate import sig  # noqa: E402
from harness import press_right, run_script  # noqa: E402
from sim import FLOOR, WHITE, Sim  # noqa: E402


def _config(marker_surface, mission_max_ms):
    return {
        "SENSE_MODE": "rgb", "RGB_EDGE_BRIGHT": 14.0,
        "FOLLOW_MAX_MS": min(150000, mission_max_ms),
        "MISSION_MAX_MS": mission_max_ms,
        "MISSION_MODE": "full",
        "SIGNATURES": {
            "floor": sig(FLOOR), "white": sig(WHITE),
            "finish": sig(marker_surface),
        },
    }


def run_final_scenario(scenario, mission_max_ms=300000, collect=False):
    sim = Sim(W=1200, H=900, start=scenario["start"], seed=scenario["seed"],
              buttons=press_right(), markers=[scenario["marker"]],
              objects=[scenario["object"]], max_ms=mission_max_ms + 10000,
              **scenario["sim_kw"])
    sequence = None
    if collect:
        observer = OdomObserver()
        observations, actions = [], []
        next_sample = [0.0]

        def sample(s):
            if s.t + 1e-6 < next_sample[0]:
                return
            a = action_from_command(s)
            obs, _ = observer.observe(s, min(1.5, s.t / mission_max_ms))
            observations.append(obs)
            actions.append(a)
            observer.prev_action = a
            next_sample[0] += CONTROL_MS

        sim.step_hook = sample

    marker_surface = scenario["marker"][4]
    out, sim = run_script("mission", sim, seed=scenario["seed"], allow_timeout=True,
                          **_config(marker_surface, mission_max_ms))
    sim.step_hook = None
    o = sim.objects[0]
    x0, y0, x1, y1, _ = scenario["marker"]
    success = (x0 - 10 <= o["x"] <= x1 + 10 and y0 - 10 <= o["y"] <= y1 + 10
               and not o["out"])
    lines = [line for line in out.splitlines() if line.startswith("result:")]
    claim = bool(lines and (lines[0][8:].startswith("DELIVERED") or
                            lines[0][8:].startswith("PASS")))
    result = {
        "success": success,
        "acceptable": success and sim.wheel_max_out <= sim.band and not o["out"],
        "wheel_on_white": sim.wheel_max_out > 0,
        "wheel_beyond": sim.wheel_max_out > sim.band,
        "object_out": bool(o["out"]), "false_claim": claim and not success,
        "time_s": sim.t / 1000.0, "claim": claim,
    }
    if collect:
        if success and len(actions) >= 6:
            # Final's last reverse is the passive-plow release; expose it as the
            # learned policy's explicit release decision.
            actions[-4:] = [8] * 4
        sequence = (np.asarray(observations, np.float32), np.asarray(actions, np.int64))
    return result, sequence


def collect_expert(path, scenario_fn, episodes=48, max_attempts=120, difficulty=0.55):
    seq_obs, seq_actions, lengths = [], [], []
    attempts = 0
    while len(lengths) < episodes and attempts < max_attempts:
        scenario = scenario_fn(attempts, "train", difficulty)
        result, seq = run_final_scenario(scenario, collect=True)
        attempts += 1
        if not result["acceptable"] or seq is None:
            continue
        obs, actions = seq
        seq_obs.append(obs)
        seq_actions.append(actions)
        lengths.append(len(actions))
        print("expert {:02d}/{:02d} seed={} steps={} time={:.0f}s".format(
            len(lengths), episodes, scenario["seed"], len(actions), result["time_s"]), flush=True)
    if not lengths:
        raise RuntimeError("no successful Final trajectories collected")
    np.savez_compressed(path, obs=np.concatenate(seq_obs), actions=np.concatenate(seq_actions),
                        lengths=np.asarray(lengths, dtype=np.int32), attempts=attempts,
                        difficulty=float(difficulty))
    return {"episodes": len(lengths), "attempts": attempts,
            "samples": int(sum(lengths))}


def load_expert(path):
    data = np.load(path)
    obs, actions, lengths = data["obs"], data["actions"], data["lengths"]
    sequences, start = [], 0
    for length in lengths:
        end = start + int(length)
        sequences.append((obs[start:end], actions[start:end]))
        start = end
    return sequences
