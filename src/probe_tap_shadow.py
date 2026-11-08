r"""Collect real tap+seat load traces and run the GRU in shadow mode.

Experimental and NOT hardware-validated. Every trial requires RIGHT. The model
never controls the decision or delivery. After the robot brakes, the operator
labels the world truth: LEFT = not captured, RIGHT = captured. CENTER remains
the Pybricks emergency stop.
"""

import config
import tap_probe_weights as weights
from common import battery_check, log, wait_for_button
from contact_inference import ContactInference
from contact_features import tap_probe_feature
from fsm import tap_verdict
from hardware import color, drive, ev3, left, right
from pybricks.parameters import Button, Color
from pybricks.tools import StopWatch, run_task, wait

TRIALS = 12


def load_pair():
    return abs(left.load()), abs(right.load())


async def label_truth():
    log(ev3, "LABEL LEFT=0 RIGHT=1")
    while True:
        pressed = ev3.buttons.pressed()
        if Button.LEFT in pressed:
            chosen = Button.LEFT
            label = 0
            break
        if Button.RIGHT in pressed:
            chosen = Button.RIGHT
            label = 1
            break
        await wait(20)
    while chosen in ev3.buttons.pressed():
        await wait(20)
    return label


async def one_trial(index):
    model = ContactInference(weights)
    infer_clock = StopWatch()
    max_infer_ms = 0
    free, early, late = [], [], []
    rows = []
    if await color.reflection() > config.edge_threshold():
        print("SHADOW_ABORT trial={} reason=start_on_white".format(index))
        return
    drive.reset()
    await drive.straight(-config.TAP_BACK_MM)
    await wait(150)
    d0 = drive.distance()
    last_d = d0
    drive.drive(config.TAP_SPEED, 0)
    aborted = False
    while True:
        if await color.reflection() > config.edge_threshold():
            aborted = True
            break
        d = drive.distance()
        trav = abs(d - d0)
        ll, lr = load_pair()
        is_free = 8 <= trav <= config.TAP_BACK_MM * 0.5
        is_early = config.TAP_BACK_MM + 15 <= trav < config.TAP_BACK_MM + config.TAP_SEAT_MM * 0.5
        is_late = trav >= config.TAP_BACK_MM + config.TAP_SEAT_MM * 0.6
        feature = tap_probe_feature(
            ll, lr, d - last_d,
            trav / (config.TAP_BACK_MM + config.TAP_SEAT_MM),
            is_free, is_early, is_late)
        t0 = infer_clock.time()
        probability, _ = model.update(feature)
        infer_ms = infer_clock.time() - t0
        if infer_ms > max_infer_ms:
            max_infer_ms = infer_ms
        phase = "F" if is_free else "E" if is_early else "L" if is_late else "-"
        rows.append((phase, ll, lr, d - last_d, probability))
        total = ll + lr
        if is_free:
            free.append(total)
        elif is_early:
            early.append(total)
        elif is_late:
            late.append(total)
        last_d = d
        if trav >= config.TAP_BACK_MM + config.TAP_SEAT_MM:
            break
        await wait(config.SAMPLE_PERIOD_MS)
    if not aborted:
        for turn, flags, phase in ((-20, (1.0, 0.0), "PL"),
                                   (20, (0.0, 1.0), "PR")):
            drive.drive(config.TAP_SPEED, turn)
            for _ in range(10):
                await wait(config.SAMPLE_PERIOD_MS)
                d = drive.distance()
                ll, lr = load_pair()
                feature = tap_probe_feature(ll, lr, d - last_d, 1.0,
                                            probe_left=flags[0], probe_right=flags[1])
                t0 = infer_clock.time()
                probability, _ = model.update(feature)
                infer_ms = infer_clock.time() - t0
                if infer_ms > max_infer_ms:
                    max_infer_ms = infer_ms
                rows.append((phase, ll, lr, d - last_d, probability))
                last_d = d
    drive.brake()
    if aborted:
        print("SHADOW_ABORT trial={} reason=white".format(index))
        return
    ok1, d1 = tap_verdict(free, early, config.TAP_MARGIN)
    ok2, d2 = tap_verdict(free, late, config.TAP_MARGIN)
    rule = int(ok1 and ok2)
    for step, row in enumerate(rows):
        print("SHADOW_DATA trial={} step={} phase={} ll={:.2f} lr={:.2f} dd={:.2f} p={:.5f}".format(
            index, step, row[0], row[1], row[2], row[3], row[4]))
    label = await label_truth()
    print("SHADOW_RESULT trial={} label={} rule={} model={} p={:.5f} d_early={:.2f} d_late={:.2f} max_infer_ms={}".format(
        index, label, rule, int(probability >= weights.THRESHOLD), probability, d1, d2, max_infer_ms))


async def main():
    ev3.screen.clear()
    log(ev3, "TAP SHADOW - UNVERIFIED")
    battery_check(ev3, config.LOW_BATTERY_MV)
    try:
        load_pair()
    except Exception as e:
        log(ev3, "load() failed", repr(e))
        return
    for trial in range(1, TRIALS + 1):
        await wait_for_button(ev3, prompt="SET TRIAL {} RIGHT".format(trial))
        ev3.light.on(Color.ORANGE)
        try:
            await one_trial(trial)
        finally:
            drive.brake()
        ev3.light.on(Color.GREEN)
    log(ev3, "SHADOW DONE")


run_task(main())

