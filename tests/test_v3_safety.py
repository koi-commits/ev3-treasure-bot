"""Boundary geometry and actual controller emergency-stop regressions."""
import ast
import asyncio
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import harness
from harness import press_right, run_script
from boundary import Boundary, fit_axis
from diagnostics_v3 import Diagnostics
from sim import Sim


def mission_functions(*names, **env):
    tree = ast.parse((Path(__file__).resolve().parents[1]/"src"/"mission.py").read_text(encoding="utf-8"))
    selected = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name in names]
    exec(compile(ast.Module(body=selected, type_ignores=[]), "mission.py", "exec"), env)
    return env


class BoundaryTests(unittest.TestCase):
    def test_front_clearance_tracks_pose_and_heading(self):
        b = Boundary(0, 1200, 900)
        b.bounds = [[-500, 700], [-400, 500]]
        self.assertAlmostEqual(b.clearance(0, 0, 0, 143, 0), 557)
        self.assertAlmostEqual(b.clearance(100, 0, 0, 143, 0), 457)
        self.assertAlmostEqual(b.clearance(0, 0, -90, 143, 0), 357)
        # At 45 degrees the outside corner reaches a wall before the centre.
        self.assertLess(b.clearance(0, 0, -45, 143, 56), b.clearance(0, 0, -45, 143, 0))

    def test_adjacent_hit_never_shrinks_a_known_wall(self):
        b = Boundary(0, 1200, 900)
        b.observe((700, 200), 0)
        b.observe((100, 500), -90)
        self.assertEqual(b.bounds[0][1], 700)
        self.assertEqual(b.bounds[1][1], 500)

    def test_unknown_opposite_uses_short_prior(self):
        b = Boundary(0, 1236, 723)
        b.observe((600, 0), 0)
        self.assertEqual(b.limits(0), (-123, 600))

    def test_axis_fit_survives_two_perpendicular_walls(self):
        # Near a corner the probe hits wall A (y=300) then wall B (x=200); a
        # straight line fit fails, but the pair clustering still finds the axis.
        pts = [(10, 300), (120, 300), (200, 20), (200, 120)]
        self.assertAlmostEqual(abs(fit_axis(pts, face=-90)), 0.0, delta=1)

    def test_observe_records_only_the_wall_that_was_hit(self):
        # Two hits on the y=300 wall, different positions along it. Only the
        # normal coordinate is a wall; the along-wall extents are open floor.
        b = Boundary(0, 1200, 900)
        b.observe((100, 300), -90)
        b.observe((700, 300), -90)
        self.assertEqual(b.bounds[1], [None, 300])
        self.assertEqual(b.bounds[0], [None, None])

    def test_one_wheel_out_is_allowed_but_two_simultaneous_fail(self):
        s = Sim(start=(-20, 450, math.pi/2), noise=0)
        s.step(5)
        self.assertGreater(s.wheel_max_out, s.band)
        self.assertFalse(s.robot_out)
        s.x = -100
        s.step(5)
        self.assertTrue(s.robot_out)

    def test_coverage_uses_mouth_not_robot_body(self):
        s = Sim(W=600, H=600, start=(300,300,0))
        d = Diagnostics(s, cell=10)
        d.step(s)
        self.assertNotIn((30,30), d.swept)  # axle is behind CB
        self.assertIn((40,30), d.swept)
        self.assertNotIn((40,40), d.swept)  # beyond lateral PLOW_MOUTH_MM / 2
        self.assertEqual(len(d.result()["coverage_grid"].splitlines()), 60)


class ControllerSafetyTests(unittest.TestCase):
    def lane(self, white=False, carry=False):
        events = []
        async def tick():
            return NS(white=white)
        async def straight(mm):
            events.append(("back", mm))
        async def wait(ms):
            self.fail("Must stop in the first sample")
        env = mission_functions("sweep_lane", "is_boundary", "fence_reached",
                                tick=tick, straight=straight, wait=wait,
            ctx=NS(captured=False, carrying=carry, stall_ms=0),
            config=NS(LANE_MAX_MM=1236, LANE_END_MM=150, SWEEP_SPEED=100,
                      BACKOFF_MM=80, AXLE_TRACK_MM=104, STALL_MS=600),
            PERIOD=50, geofence_mm=lambda: 4, carry_lost=lambda: False,
            FINISH="finish",
            drive=NS(distance=lambda: 0, brake=lambda: events.append("brake")),
            hold_heading=lambda *a: self.fail("Must not command forward motion"))
        result = asyncio.run(env["sweep_lane"](0, carry))
        return result, events

    def test_empty_and_loaded_lanes_stop_at_front_geofence(self):
        for carry in (False, True):
            result, events = self.lane(carry=carry)
            self.assertEqual(result[0], "geofence")
            self.assertEqual(events, ["brake"])

    def test_first_white_sample_brakes_then_backs_off_half_track_plus_30(self):
        result, events = self.lane(white=True)
        self.assertEqual(result[0], "end")
        self.assertEqual(events, ["brake", ("back", -82)])

    def test_turn_makes_up_insufficient_white_backoff(self):
        events = []
        async def straight(mm):
            events.append(("back", mm))
        async def noop(*args):
            pass
        env = mission_functions("rotate", "swing_guard",
            ctx=NS(carrying=False, use_gyro=True, white_backoff=30, boundary=None, object_pt=None),
            config=NS(AXLE_TRACK_MM=104), straight=straight, odom_update=noop,
            _settle=noop, drive=NS(turn=lambda d,wait: events.append(("turn",d))))
        asyncio.run(env["rotate"](90))
        self.assertEqual(events, [("back", -52), ("turn", 90)])

    def swing_env(self, x, heading, events):
        b = Boundary(0, 1200, 900)
        b.bounds = [[-500, 700], [None, None]]
        ctx = NS(carrying=False, use_gyro=False, white_backoff=None, boundary=b,
                 odom=NS(x=x, y=0, heading=heading), object_pt=None,
                 swing_turns=0, swing_backoff_mm=0.0, swing_short=0)
        async def straight(mm):
            events.append(("back", round(mm, 1)))
            ctx.odom.x += mm
        async def noop(*args):
            pass
        cfg = harness.fresh_config()
        env = mission_functions("rotate", "swing_guard", "ahead", ctx=ctx, config=cfg,
            straight=straight, odom_update=noop, _settle=noop, pi=math.pi, cos=math.cos, sin=math.sin,
            drive=NS(turn=lambda d, wait: events.append(("turn", d))))
        return env, ctx, cfg

    def test_turn_near_known_wall_backs_off_for_plow_swing(self):
        events = []
        env, ctx, cfg = self.swing_env(600, 0, events)   # facing the +x wall, 100 mm away
        asyncio.run(env["rotate"](90))
        need = cfg.PLOW_SWING_R_MM + cfg.SWING_MARGIN_MM - 100
        self.assertEqual(events, [("back", round(-need, 1)), ("turn", 90)])
        self.assertEqual(ctx.swing_turns, 1)
        self.assertEqual(ctx.swing_short, 0)
        self.assertGreaterEqual(700 - ctx.odom.x, cfg.PLOW_SWING_R_MM + cfg.SWING_MARGIN_MM - 1e-6)

    def test_turn_far_from_walls_needs_no_back_off(self):
        events = []
        env, ctx, cfg = self.swing_env(0, 0, events)
        asyncio.run(env["rotate"](90))
        self.assertEqual(events, [("turn", 90)])
        self.assertEqual(ctx.swing_turns, 0)

    def test_swing_back_off_never_reverses_into_a_known_wall(self):
        b = Boundary(0, 1200, 900)
        b.bounds = [[-500, 700], [None, None]]
        # Facing -x with the -x wall 100 mm ahead: reverse 129 mm; the +x wall
        # 1100 mm behind allows up to 1000 mm before the rear gets within 100 mm.
        need, allowed, blocked = b.swing_backoff(-400, 0, 180, 229, 100)
        self.assertAlmostEqual(need, 129.0, places=6)
        self.assertAlmostEqual(allowed, 1000.0, places=6)
        need, allowed, blocked = b.swing_backoff(550, 0, 0, 229, 100)
        self.assertAlmostEqual(need, 79.0, places=6)           # +x wall ahead, 150 mm away
        self.assertFalse(blocked)
        # A wall behind the robot cannot be cleared by reversing.
        need, allowed, blocked = b.swing_backoff(-400, 0, 0, 229, 100)
        self.assertEqual(need, 0.0)
        self.assertTrue(blocked)
        self.assertAlmostEqual(allowed, 0.0)                   # rear already at 100 mm
        # Mid-arena with a wall 180 mm behind: back-off is capped to keep the rear clear.
        b.bounds = [[-500, 700], [-400, 500]]
        need, allowed, blocked = b.swing_backoff(-320, 380, 0, 229, 100)
        self.assertAlmostEqual(allowed, 80.0, places=6)


    def test_gyro_alignment_waits_for_three_stable_samples(self):
        readings = iter([20, 5, 1, 0, -1])
        ctx = NS(target=None, odom=NS(heading=20), now=lambda: 0)
        seen = []
        async def tick():
            ctx.odom.heading = next(readings)
            seen.append(ctx.odom.heading)
        async def wait(ms):
            pass
        env = mission_functions("gyro_align", ctx=ctx, tick=tick, wait=wait,
            config=NS(CARRY_SPEED=70,CARRY_TURN_RATE=35), PERIOD=50, wrap180=lambda x:x, clamp=lambda x,lo,hi:max(lo,min(hi,x)),
            drive=NS(brake=lambda:None, drive=lambda v,w:self.assertGreater(v,0)))
        asyncio.run(env["gyro_align"](0))
        self.assertEqual(len(seen), 5)


class EarlyCaptureTests(unittest.TestCase):
    def test_drop_precedes_mapping_and_geofenced_recapture(self):
        import evaluate

        result = evaluate.run_one(2, mode="full", obj=evaluate.object_near_start)
        trace = result["out"]
        events = ["early capture: park at", "boundary estimates",
                  "parked tyre recaptured 0", "deliver:"]
        positions = [trace.index(event) for event in events]
        self.assertEqual(positions, sorted(positions))
        self.assertTrue(result["acceptable"], trace)
        self.assertEqual(result["carry_inplace_turns"], 0)

    def test_drop_records_capture_pose_before_reversing(self):
        ctx = NS(odom=NS(x=100, y=20, heading=0), carrying=False,
                 captured=True, pressed_n=3)
        moves = []

        async def straight(mm):
            moves.append(mm)
            ctx.odom.x += mm

        async def rotate(deg):
            self.assertFalse(ctx.captured)
            moves.append(deg)

        env = mission_functions("park_early_capture", ctx=ctx,
            config=NS(OBJ_LEAD_MM=100, TYRE_LEAVE_BACK_MM=153.0), drive=NS(brake=lambda: moves.append("brake")),
            ev=lambda *args: None, ahead=lambda x,y,h,d: (x+d,y),
            straight=straight, rotate=rotate, rand_sign=lambda: 1)
        self.assertEqual(asyncio.run(env["park_early_capture"]()), ((200, 20), 0))
        self.assertEqual(moves, ["brake", -153.0, 90])
