r"""Experimental learned contact detector (NOT hardware-validated).

RIGHT press-and-release is required before motion. CENTER remains the Pybricks
emergency stop. The robot drives slowly for at most 30 s, brakes on white, and
prints CONTACT CANDIDATE; it never claims that an object was truly captured.
Run the existing probe_load.py first and keep the wheels clear for the first
real-hardware trial.
"""

import contact_weights as weights
from common import battery_check, log, wait_for_button
from contact_inference import ContactInference
from hardware import color, drive, ev3, left, right
from pybricks.parameters import Color
from pybricks.tools import StopWatch, run_task, wait


async def main():
    ev3.screen.clear()
    log(ev3, "LEARNED CAPTURE")
    log(ev3, "UNVERIFIED / CLEAR WHEELS")
    battery_check(ev3, 7000)
    await wait_for_button(ev3, prompt="RIGHT to start")
    model = ContactInference(weights)
    clock = StopWatch()
    drive.reset()
    last_d = last_a = 0.0
    baseline = None
    result = "TIMEOUT (unverified)"
    ev3.light.on(Color.ORANGE)
    try:
        while clock.time() < 30000:
            rgb = await color.rgb()
            bright = sum(rgb) / 3.0
            red = rgb[0] > 12 and rgb[0] > rgb[1] * 1.65
            if bright > 14.0 and not red:
                result = "WHITE STOP (safe)"
                break
            ll, lr = abs(left.load()), abs(right.load())
            total = ll + lr
            if baseline is None:
                baseline = total
            elif total < baseline + 8:
                baseline = 0.98 * baseline + 0.02 * total
            d, a = drive.distance(), drive.angle()
            features = (ll / 200.0, lr / 200.0, (total - baseline) / 80.0,
                        (d - last_d) / 30.0, (a - last_a) / 20.0,
                        100.0 / 250.0, 0.0)
            last_d, last_a = d, a
            probability, confirmed = model.update(features)
            print("p={:.3f} load={:.1f}".format(probability, total))
            if confirmed:
                result = "CONTACT CANDIDATE (unverified)"
                break
            drive.drive(100, 0)
            await wait(100)
    finally:
        drive.brake()
    ev3.light.on(Color.GREEN)
    print(result)
    ev3.screen.print(result)
    ev3.speaker.beep(700, 150)


run_task(main())
