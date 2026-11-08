r"""Safety-gated real tap/probe collection with a shadow-only learned model.

Experimental and NOT hardware-validated. The model only writes logs; it never
controls capture, delivery, or Final. Every motion block requires a separate
RIGHT press-and-release. CENTER remains the Pybricks emergency stop.

Visual truth definition for both labels:
  RIGHT (1) = object center is securely inside the V pocket
  LEFT  (0) = absent, side contact, loose, or outside the pocket
Do not touch the robot/object between PRE and POST labels.
"""

import config
import tap_probe_weights as weights
from common import battery_check, log, wait_for_button
from contact_features import tap_probe_feature
from contact_inference import ContactInference
from fsm import tap_verdict
from hardware import color, drive, ev3, left, right
from pybricks.parameters import Button, Color
from pybricks.tools import StopWatch, run_task, wait

TRIAL_PLAN = ("EMPTY", "EMPTY", "EMPTY",
              "CENTER_HEAVY", "CENTER_HEAVY", "CENTER_HEAVY",
              "SIDE_CONTACT", "SIDE_CONTACT", "SIDE_CONTACT",
              "CENTER_LIGHT", "CENTER_LIGHT", "CENTER_LIGHT")
TRIALS = len(TRIAL_PLAN)
PERIOD_MS = config.SAMPLE_PERIOD_MS
BACK_TIMEOUT_MS = 3500
TAP_TIMEOUT_MS = 6000
PROBE_STAGE_MS = 500
PROBE_TIMEOUT_MS = 1200
NO_PROGRESS_MS = 500
MIN_PROGRESS_MM = 1
SCRIPT_VERSION = "tap-shadow-v2"


def load_pair():
    return abs(left.load()), abs(right.load())


async def is_white():
    return await color.reflection() > config.edge_threshold()


async def label_truth(stage):
    log(ev3, stage, "LEFT=0 RIGHT=1")
    while True:
        pressed = ev3.buttons.pressed()
        has_left = Button.LEFT in pressed
        has_right = Button.RIGHT in pressed
        if has_left and has_right:
            print("SHADOW_LABEL_INVALID stage={} reason=both".format(stage))
            while (Button.LEFT in ev3.buttons.pressed() or
                   Button.RIGHT in ev3.buttons.pressed()):
                await wait(20)
            log(ev3, stage, "ONE BUTTON ONLY")
        elif has_left:
            chosen = Button.LEFT
            label = 0
            break
        elif has_right:
            chosen = Button.RIGHT
            label = 1
            break
        await wait(20)
    while chosen in ev3.buttons.pressed():
        await wait(20)
    print("SHADOW_LABEL stage={} label={}".format(stage, label))
    return label


async def monitored_straight(distance, timeout_ms):
    """Run a finite move while checking white, stall, timeout, and no progress."""
    if await is_white():
        drive.brake()
        return "white"
    clock = StopWatch()
    drive.straight(distance, wait=False)
    start_d = drive.distance()
    last_progress = 0
    last_progress_ms = 0
    while not drive.done():
        await wait(PERIOD_MS)
        if await is_white():
            drive.brake()
            return "white"
        if drive.stalled():
            drive.brake()
            return "stalled"
        now = clock.time()
        progress = abs(drive.distance() - start_d)
        if progress >= last_progress + MIN_PROGRESS_MM:
            last_progress = progress
            last_progress_ms = now
        elif now - last_progress_ms >= NO_PROGRESS_MS:
            drive.brake()
            return "no_progress"
        if now >= timeout_ms:
            drive.brake()
            return "timeout"
    drive.brake()
    return None


async def tap_forward(model, rows):
    """Production tap forward segment with an independent safety envelope."""
    free, early, late = [], [], []
    clock = StopWatch()
    d0 = drive.distance()
    last_d = d0
    last_progress = 0
    last_progress_ms = 0
    next_sample = PERIOD_MS
    max_infer_ms = 0
    max_cycle_ms = 0
    previous_sample_ms = 0
    probability = 0.0
    if await is_white():
        drive.brake()
        return {"abort": "white", "phase": "tap_forward"}
    drive.drive(config.TAP_SPEED, 0)
    while True:
        delay = next_sample - clock.time()
        if delay > 0:
            await wait(delay)
        sample_ms = clock.time()
        dt_ms = sample_ms - previous_sample_ms
        previous_sample_ms = sample_ms
        next_sample += PERIOD_MS
        if dt_ms > max_cycle_ms:
            max_cycle_ms = dt_ms
        if await is_white():
            drive.brake()
            return {"abort": "white", "phase": "tap_forward"}
        if drive.stalled():
            drive.brake()
            return {"abort": "stalled", "phase": "tap_forward"}
        d = drive.distance()
        trav = abs(d - d0)
        if trav >= last_progress + MIN_PROGRESS_MM:
            last_progress = trav
            last_progress_ms = sample_ms
        elif sample_ms - last_progress_ms >= NO_PROGRESS_MS:
            drive.brake()
            return {"abort": "no_progress", "phase": "tap_forward"}
        if sample_ms >= TAP_TIMEOUT_MS:
            drive.brake()
            return {"abort": "timeout", "phase": "tap_forward"}

        ll, lr = load_pair()
        is_free = 8 <= trav <= config.TAP_BACK_MM * 0.5
        is_early = config.TAP_BACK_MM + 15 <= trav < config.TAP_BACK_MM + config.TAP_SEAT_MM * 0.5
        is_late = trav >= config.TAP_BACK_MM + config.TAP_SEAT_MM * 0.6
        feature = tap_probe_feature(
            ll, lr, d - last_d,
            trav / (config.TAP_BACK_MM + config.TAP_SEAT_MM),
            is_free, is_early, is_late)
        infer_clock = StopWatch()
        probability, _ = model.update(feature)
        infer_ms = infer_clock.time()
        if infer_ms > max_infer_ms:
            max_infer_ms = infer_ms
        phase = "F" if is_free else "E" if is_early else "L" if is_late else "-"
        rows.append((clock.time(), dt_ms, phase, ll, lr, d - last_d, probability))
        total = ll + lr
        if is_free:
            free.append(total)
        elif is_early:
            early.append(total)
        elif is_late:
            late.append(total)
        last_d = d
        if trav >= config.TAP_BACK_MM + config.TAP_SEAT_MM:
            drive.brake()
            return {"abort": None, "free": free, "early": early, "late": late,
                    "p_pre": probability, "max_infer_ms": max_infer_ms,
                    "max_cycle_ms": max_cycle_ms, "tap_ms": clock.time(),
                    "last_d": last_d}


async def probe_stage(model, rows, last_d, turn, phase, flag_left, flag_right):
    """One 500 ms probe arc with continuous safety checks."""
    clock = StopWatch()
    last_progress_d = drive.distance()
    last_progress_ms = 0
    next_sample = PERIOD_MS
    previous_sample_ms = 0
    max_infer_ms = 0
    max_cycle_ms = 0
    probability = 0.0
    if await is_white():
        drive.brake()
        return {"abort": "white", "phase": phase, "last_d": last_d,
                "p": probability, "max_infer_ms": max_infer_ms,
                "max_cycle_ms": max_cycle_ms, "elapsed_ms": 0}
    drive.drive(config.TAP_SPEED, turn)
    while clock.time() < PROBE_STAGE_MS:
        delay = next_sample - clock.time()
        if delay > 0:
            await wait(delay)
        sample_ms = clock.time()
        dt_ms = sample_ms - previous_sample_ms
        previous_sample_ms = sample_ms
        next_sample += PERIOD_MS
        if dt_ms > max_cycle_ms:
            max_cycle_ms = dt_ms
        if await is_white():
            drive.brake()
            return {"abort": "white", "phase": phase, "last_d": last_d,
                    "p": probability, "max_infer_ms": max_infer_ms,
                    "max_cycle_ms": max_cycle_ms, "elapsed_ms": sample_ms}
        if drive.stalled():
            drive.brake()
            return {"abort": "stalled", "phase": phase, "last_d": last_d,
                    "p": probability, "max_infer_ms": max_infer_ms,
                    "max_cycle_ms": max_cycle_ms, "elapsed_ms": sample_ms}
        d = drive.distance()
        if abs(d - last_progress_d) >= MIN_PROGRESS_MM:
            last_progress_d = d
            last_progress_ms = sample_ms
        elif sample_ms - last_progress_ms >= NO_PROGRESS_MS:
            drive.brake()
            return {"abort": "no_progress", "phase": phase, "last_d": last_d,
                    "p": probability, "max_infer_ms": max_infer_ms,
                    "max_cycle_ms": max_cycle_ms, "elapsed_ms": sample_ms}
        if sample_ms >= PROBE_TIMEOUT_MS:
            drive.brake()
            return {"abort": "timeout", "phase": phase, "last_d": last_d,
                    "p": probability, "max_infer_ms": max_infer_ms,
                    "max_cycle_ms": max_cycle_ms, "elapsed_ms": sample_ms}
        ll, lr = load_pair()
        feature = tap_probe_feature(ll, lr, d - last_d, 1.0,
                                    probe_left=flag_left, probe_right=flag_right)
        infer_clock = StopWatch()
        probability, _ = model.update(feature)
        infer_ms = infer_clock.time()
        if infer_ms > max_infer_ms:
            max_infer_ms = infer_ms
        rows.append((clock.time(), dt_ms, phase, ll, lr, d - last_d, probability))
        last_d = d
    drive.brake()
    return {"abort": None, "phase": phase, "last_d": last_d, "p": probability,
            "max_infer_ms": max_infer_ms, "max_cycle_ms": max_cycle_ms,
            "elapsed_ms": clock.time()}


def print_rows(index, condition, rows):
    for step, row in enumerate(rows):
        print("SHADOW_DATA trial={} condition={} step={} t_ms={} dt_ms={} phase={} ll={:.2f} lr={:.2f} dd={:.2f} p={:.5f}".format(
            index, condition, step, row[0], row[1], row[2], row[3], row[4], row[5], row[6]))


async def one_trial(index, condition):
    if await is_white():
        print("SHADOW_ABORT trial={} condition={} phase=start reason=white".format(index, condition))
        return
    model = ContactInference(weights)
    rows = []
    drive.brake()
    await wait(150)
    drive.reset()
    reason = await monitored_straight(-config.TAP_BACK_MM, BACK_TIMEOUT_MS)
    if reason is not None:
        print("SHADOW_ABORT trial={} condition={} phase=initial_back reason={}".format(
            index, condition, reason))
        return
    await wait(150)

    tap = await tap_forward(model, rows)
    if tap["abort"] is not None:
        print_rows(index, condition, rows)
        print("SHADOW_ABORT trial={} condition={} phase={} reason={}".format(
            index, condition, tap["phase"], tap["abort"]))
        return
    ok1, d1 = tap_verdict(tap["free"], tap["early"], config.TAP_MARGIN)
    ok2, d2 = tap_verdict(tap["free"], tap["late"], config.TAP_MARGIN)
    rule = int(ok1 and ok2)
    pre_label = await label_truth("PRE_PROBE")

    # Labeling may put hands near the robot: require a fresh motion authorization.
    await wait_for_button(ev3, prompt="CLEAR HANDS: RIGHT PROBE")
    left_probe = await probe_stage(model, rows, tap["last_d"], -20, "PL", True, False)
    probe = left_probe
    if left_probe["abort"] is None:
        right_probe = await probe_stage(model, rows, left_probe["last_d"], 20, "PR", False, True)
        probe = right_probe
    drive.brake()
    post_label = await label_truth("POST_PROBE")
    print_rows(index, condition, rows)
    complete = int(probe["abort"] is None)
    p_post = probe["p"]
    max_infer_ms = max(tap["max_infer_ms"], left_probe["max_infer_ms"],
                       probe["max_infer_ms"])
    max_cycle_ms = max(tap["max_cycle_ms"], left_probe["max_cycle_ms"],
                       probe["max_cycle_ms"])
    transition = "{}to{}".format(pre_label, post_label)
    print("SHADOW_RESULT version={} trial={} condition={} complete={} abort_phase={} abort_reason={} pre_label={} post_label={} transition={} rule={} model={} p_pre={:.5f} p_post={:.5f} d_early={:.2f} d_late={:.2f} max_infer_ms={} max_cycle_ms={} battery_mv={}".format(
        SCRIPT_VERSION, index, condition, complete,
        "none" if complete else probe["phase"],
        "none" if complete else probe["abort"],
        pre_label, post_label, transition, rule,
        int(complete and p_post >= weights.THRESHOLD), tap["p_pre"], p_post,
        d1, d2, max_infer_ms, max_cycle_ms, ev3.battery.voltage()))


async def main():
    ev3.screen.clear()
    log(ev3, "TAP SHADOW V2 - NO CONTROL")
    battery_mv = battery_check(ev3, config.LOW_BATTERY_MV)
    print("SHADOW_SESSION version={} trials={} period_ms={} threshold={} back_timeout_ms={} tap_timeout_ms={} probe_timeout_ms={} no_progress_ms={} battery_mv={}".format(
        SCRIPT_VERSION, TRIALS, PERIOD_MS, weights.THRESHOLD,
        BACK_TIMEOUT_MS, TAP_TIMEOUT_MS, PROBE_TIMEOUT_MS, NO_PROGRESS_MS,
        battery_mv))
    if battery_mv < config.LOW_BATTERY_MV:
        print("SHADOW_SESSION_ABORT reason=low_battery battery_mv={} minimum_mv={}".format(
            battery_mv, config.LOW_BATTERY_MV))
        drive.brake()
        return
    try:
        load_pair()
    except Exception as e:
        log(ev3, "load() failed", repr(e))
        return
    for trial, condition in enumerate(TRIAL_PLAN, 1):
        await wait_for_button(ev3, prompt="{} {} RIGHT TAP".format(trial, condition))
        ev3.light.on(Color.ORANGE)
        try:
            await one_trial(trial, condition)
        except Exception as e:
            print("SHADOW_FATAL trial={} condition={} error={}".format(
                trial, condition, repr(e)))
            return
        finally:
            drive.brake()
        ev3.light.on(Color.GREEN)
    log(ev3, "SHADOW DONE")


run_task(main())

