"""Partially observable treasure task built on the existing contact simulator.

The policy never receives simulator pose, object pose, marker rectangle, contact
state, or success flags.  Those values are used only for reward shaping and the
ground-truth evaluator.  Inputs are limited to values available on the EV3:
one RGB sample, two motor-load estimates, dead-reckoned odometry, time, and the
previous action.
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
for _path in (TESTS, SRC):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from sim import RED_MARK, Sim, rect_arena_marker  # noqa: E402


W, H = 1200, 900
CONTROL_MS = 200
PHYSICS_MS = 20

# Positive turn rate is clockwise in DriveBase/Sim.
ACTION_NAMES = (
    "forward", "arc_left", "arc_right", "spin_left", "spin_right",
    "reverse", "brake", "slow_forward", "release",
)
ACTION_COMMANDS = (
    (230.0, 0.0), (180.0, -55.0), (180.0, 55.0), (25.0, -105.0),
    (25.0, 105.0), (-130.0, 0.0), (0.0, 0.0), (100.0, 0.0), None,
)
N_ACTIONS = len(ACTION_NAMES)
OBS_SIZE = 27


def _normal_start(rng):
    return (rng.uniform(150, W - 150), rng.uniform(150, H - 150),
            rng.uniform(0.0, 2.0 * math.pi))


def make_scenario(seed: int, split: str = "train", difficulty: float = 1.0):
    """Create a deterministic scene. Split offsets guarantee disjoint cases."""
    rng = random.Random(derive_seed(split, seed, "scene"))
    d = max(0.0, min(1.0, float(difficulty)))

    if d < 0.20:
        side = "right"
        start = (W - 170.0, H / 2.0, 0.0)
        ox, oy = W * 0.58, H * 0.50
    else:
        side = rng.choice(("right", "top", "left", "bottom"))
        start = _normal_start(rng)
        near_edge = d > 0.65 and rng.random() < 0.35
        if near_edge:
            edge = rng.randrange(4)
            gap = rng.uniform(55, 115)
            ox, oy = rng.uniform(150, W - 150), rng.uniform(150, H - 150)
            if edge == 0:
                oy = gap
            elif edge == 1:
                oy = H - gap
            elif edge == 2:
                ox = gap
            else:
                ox = W - gap
        else:
            ox, oy = rng.uniform(140, W - 140), rng.uniform(140, H - 140)

    marker = rect_arena_marker(W, H, side, surf=RED_MARK)
    obj = {
        "x": ox, "y": oy,
        "r": rng.uniform(23, 27) if d < 0.45 else rng.uniform(20, 35),
        "load": 22.0 if d < 0.35 else rng.uniform(10, 42),
    }
    spread = max(0.0, (d - 0.25) / 0.75)
    sim_kw = {
        "band": rng.uniform(34, 42) if spread else 40.0,
        "sensor_offset": rng.uniform(55, 78) if spread else 60.0,
        "noise": rng.uniform(0.10, 0.40) if spread else 0.15,
        "load_noise": rng.uniform(1.5, 5.0) if spread else 2.5,
        "contact_mu": rng.uniform(0.10, 0.60) if spread else 0.30,
        "odo_dist_scale": rng.uniform(0.96, 1.04) if spread else 1.0,
        "odo_turn_scale": rng.uniform(0.95, 1.04) if spread else 1.0,
        "accel": rng.uniform(550, 900) if spread else 750.0,
        "brake_decel": rng.uniform(1400, 2400) if spread else 2000.0,
    }
    return {"seed": int(seed), "split": split,
            "sim_seed": derive_seed(split, seed, "physics"),
            "policy_seed": derive_seed(split, seed, "controller"),
            "start": start, "marker": marker,
            "object": obj, "sim_kw": sim_kw, "side": side}


class OdomObserver:
    """EV3-compatible dead reckoning and feature construction."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.x = self.y = self.heading = 0.0
        self.last_dist = self.last_angle = None
        self.prev_action = 6
        self.marker_known = False
        self.target_x = self.target_y = 0.0
        self.target_n = 0

    def observe(self, sim, elapsed_fraction=0.0):
        dist, angle = float(sim.odo_distance()), float(sim.odo_angle())
        if self.last_dist is None:
            dd, da = 0.0, 0.0
        else:
            dd, da = dist - self.last_dist, angle - self.last_angle
            mid = self.heading + da * 0.5
            self.x += dd * math.cos(-math.radians(mid))
            self.y += dd * math.sin(-math.radians(mid))
        self.heading = angle
        self.last_dist, self.last_angle = dist, angle

        rgb = sim.read_sensor("rgb")
        bright = sum(rgb) / 3.0
        red_score = max(0.0, min(1.5, (rgb[0] - max(rgb[1], rgb[2])) / 15.0))
        is_marker = rgb[0] > 12 and rgb[0] > rgb[1] * 1.65 and red_score > 0.35
        is_white = bright > 14.0 and not is_marker
        loads = (sim.motor_load("L"), sim.motor_load("R"))
        h = math.radians(self.heading)
        if is_marker:
            # External memory derived only from the current RGB hit + odometry.
            # Store the observed marker point in the start-relative odom frame.
            sx = self.x + sim.sensor_offset * math.cos(-h)
            sy = self.y + sim.sensor_offset * math.sin(-h)
            self.target_n += 1
            alpha = 1.0 / min(20, self.target_n)
            self.target_x += alpha * (sx - self.target_x)
            self.target_y += alpha * (sy - self.target_y)
            self.marker_known = True
        rel_x = self.target_x - self.x if self.marker_known else 0.0
        rel_y = self.target_y - self.y if self.marker_known else 0.0
        obs = [
            rgb[0] / 30.0, rgb[1] / 30.0, rgb[2] / 30.0,
            bright / 30.0, float(is_white), red_score,
            loads[0] / 200.0, loads[1] / 200.0,
            self.x / W, self.y / H, math.sin(h), math.cos(h),
            max(-2.0, min(2.0, dd / 50.0)), max(-2.0, min(2.0, da / 45.0)),
            max(0.0, min(1.5, elapsed_fraction)),
            float(self.marker_known), rel_x / W, rel_y / H,
        ]
        obs.extend(1.0 if i == self.prev_action else 0.0 for i in range(N_ACTIONS))
        return np.asarray(obs, dtype=np.float32), is_marker


def action_from_command(sim):
    """Map the Final controller's current motor command to a policy action."""
    if sim.man is not None:
        if sim.man["kind"] == "d":
            return 0 if sim.man["target"] >= 0 else 5
        return 4 if sim.man["target"] >= 0 else 3
    v, w = sim.cmd_v, sim.cmd_w
    if sim.stop_mode is not None or (abs(v) < 5 and abs(w) < 5):
        return 6
    if v < -5:
        return 5
    if abs(w) > 75:
        return 4 if w > 0 else 3
    if w > 12:
        return 2
    if w < -12:
        return 1
    return 7 if v < 150 else 0


class TreasureEnv:
    def __init__(self, seed=0, split="train", difficulty=1.0, max_steps=1200,
                 scenario=None, pre_map=True, total_budget_ms=None):
        self.max_steps = int(max_steps)
        self.split = split
        self.difficulty = difficulty
        self.pre_map = pre_map
        self.total_budget_ms = total_budget_ms
        self.reset(seed=seed, scenario=scenario)

    def reset(self, seed=None, scenario=None):
        if seed is not None:
            self.seed = int(seed)
        self.scenario = scenario or make_scenario(self.seed, self.split, self.difficulty)
        sc = self.scenario
        self.sim = Sim(W=W, H=H, start=sc["start"], seed=sc["sim_seed"],
                       markers=[sc["marker"]], objects=[sc["object"]],
                       max_ms=self.max_steps * CONTROL_MS + 210000, **sc["sim_kw"])
        self.observer = OdomObserver()
        self.scout_out = ""
        if self.pre_map:
            self._run_scout()
            # Curriculum states cover capture and delivery before the full random
            # task. They are never used by validation/test (difficulty=1).
            if self.split == "train" and self.difficulty < 0.28:
                o = self.sim.objects[0]
                c, s = math.cos(self.sim.th), math.sin(self.sim.th)
                lead = 108.0 if self.difficulty < 0.10 else 180.0
                o["x"], o["y"] = self.sim.x + lead * c, self.sim.y + lead * s
                o["out"] = False
        self.steps = 0
        self.marker_seen = self.observer.marker_known
        self.marker_confirm = 0
        self.ever_captured = False
        self.lost_count = 0
        self.false_claim = False
        self.wheel_beyond_seen = False
        self.was_captured = False
        self.at_target_seen = False
        self.visited = {(int(self.sim.x // 120), int(self.sim.y // 120))}
        self.done = False
        obs, marker = self.observer.observe(self.sim, 0.0)
        self._update_marker(marker)
        self._potential = self._shape_potential()
        self.last_obs = obs
        return obs

    def _run_scout(self):
        """Use the existing safe Final marker-mapping phase before learned control.

        This is the deliberate hybrid boundary of the first research milestone:
        learned search/capture/transport starts only after the already validated
        edge controller has produced an odometric marker memory.
        """
        from evaluate import sig
        from harness import press_right, run_script
        from sim import FLOOR, WHITE

        self.sim.button_events = press_right()
        next_sample = [0.0]

        def sample(s):
            if s.t + 1e-6 < next_sample[0]:
                return
            a = action_from_command(s)
            self.observer.observe(s, min(1.0, s.t / 180000.0))
            self.observer.prev_action = a
            next_sample[0] += CONTROL_MS

        self.sim.step_hook = sample
        marker_surface = self.scenario["marker"][4]
        cfg = {
            "SENSE_MODE": "rgb", "RGB_EDGE_BRIGHT": 14.0,
            "FOLLOW_MAX_MS": 120000, "MISSION_MAX_MS": 180000,
            "MISSION_MODE": "calib",
            "SIGNATURES": {"floor": sig(FLOOR), "white": sig(WHITE),
                           "finish": sig(marker_surface)},
        }
        self.scout_out, _ = run_script("mission", self.sim, seed=self.scenario["policy_seed"],
                                       allow_timeout=True, **cfg)
        self.sim.step_hook = None
        self.sim.cmd_stop("brake")

    def _update_marker(self, marker):
        self.marker_confirm = self.marker_confirm + 1 if marker else 0
        newly = self.marker_confirm >= 2 and not self.marker_seen
        if self.marker_confirm >= 2:
            self.marker_seen = True
        return newly

    def _marker_center(self):
        x0, y0, x1, y1, _ = self.scenario["marker"]
        return (0.5 * (x0 + x1), 0.5 * (y0 + y1))

    @staticmethod
    def _dist(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _shape_potential(self):
        o = self.sim.objects[0]
        diag = math.sqrt(W * W + H * H)
        if not self.marker_seen:
            pos = self.sim.sensor_pos()
            goal = self._marker_center()
        elif not self.sim.in_pocket(o):
            c, s = math.cos(self.sim.th), math.sin(self.sim.th)
            pos = (self.sim.x + 120 * c, self.sim.y + 120 * s)
            goal = (o["x"], o["y"])
        else:
            pos = (o["x"], o["y"])
            goal = self._marker_center()
        return -self._dist(pos, goal) / diag

    def _advance(self, ms):
        n = max(1, int(math.ceil(ms / PHYSICS_MS)))
        for _ in range(n):
            self.sim.step(ms / n)

    def step(self, action):
        if self.done:
            raise RuntimeError("step() after terminal state")
        action = int(action)
        release = action == N_ACTIONS - 1
        if release:
            # Model-selected release primitive: brake, reverse out of the passive
            # plow, then brake. Ground truth is checked only after the release.
            self.sim.cmd_stop("brake")
            self._advance(250)
            self.sim.cmd_drive(-120, 0)
            self._advance(650)
            self.sim.cmd_stop("brake")
            self._advance(150)
        else:
            v, w = ACTION_COMMANDS[action]
            if action == 6:
                self.sim.cmd_stop("brake")
            else:
                self.sim.cmd_drive(v, w)
            self._advance(CONTROL_MS)

        self.steps += 1
        self.observer.prev_action = action
        obs, marker = self.observer.observe(self.sim, self.steps / self.max_steps)
        newly_seen = self._update_marker(marker)
        o = self.sim.objects[0]
        captured = self.sim.in_pocket(o)
        newly_captured = captured and not self.ever_captured
        if newly_captured:
            self.ever_captured = True
        if self.was_captured and not captured and not release:
            self.lost_count += 1
        self.was_captured = captured

        potential = self._shape_potential()
        reward = -0.01 + 12.0 * (potential - self._potential)
        self._potential = potential
        cell = (int(self.sim.x // 120), int(self.sim.y // 120))
        if cell not in self.visited:
            self.visited.add(cell)
            reward += 0.08
        if newly_seen:
            reward += 10.0
        if newly_captured:
            reward += 15.0
        beyond_now = self.sim.wheel_max_out > self.sim.band
        if beyond_now and not self.wheel_beyond_seen:
            reward -= 20.0
            self.wheel_beyond_seen = True

        x0, y0, x1, y1, _ = self.scenario["marker"]
        delivered = (x0 - 10 <= o["x"] <= x1 + 10 and
                     y0 - 10 <= o["y"] <= y1 + 10 and not o["out"])
        if delivered and not self.at_target_seen:
            reward += 30.0
            self.at_target_seen = True
        success = False
        if release:
            success = delivered
            self.false_claim = not success
            reward += 100.0 if success else -25.0
            self.done = True
        if o["out"]:
            reward -= 60.0
            self.done = True
        if self.steps >= self.max_steps:
            self.done = True
        if self.total_budget_ms is not None and self.sim.t >= self.total_budget_ms:
            self.done = True

        info = {
            "success": success,
            "acceptable": success and self.sim.wheel_max_out <= self.sim.band and not o["out"],
            "wheel_on_white": self.sim.wheel_max_out > 0,
            "wheel_beyond": self.sim.wheel_max_out > self.sim.band,
            "object_out": bool(o["out"]), "false_claim": self.false_claim,
            "captured": captured, "ever_captured": self.ever_captured,
            "marker_seen": self.marker_seen, "lost_count": self.lost_count,
            "time_s": self.sim.t / 1000.0,
        }
        return obs, float(reward), self.done, info

    def result(self):
        o = self.sim.objects[0]
        return {
            "success": False, "acceptable": False,
            "wheel_on_white": self.sim.wheel_max_out > 0,
            "wheel_beyond": self.sim.wheel_max_out > self.sim.band,
            "object_out": bool(o["out"]), "false_claim": self.false_claim,
            "captured": self.sim.in_pocket(o), "ever_captured": self.ever_captured,
            "marker_seen": self.marker_seen, "lost_count": self.lost_count,
            "time_s": self.sim.t / 1000.0,
        }
