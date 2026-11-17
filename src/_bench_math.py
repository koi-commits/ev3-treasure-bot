r"""Micro-benchmark: soft-float vs integer throughput on the EV3 build.

Why this exists: the shadow runbook rejects model samples with
``max_infer_ms > 20``, and the recorded trials all came back at 25-26 ms, so
every trial was excluded from accuracy scoring. Before rewriting the GRU we
need to know whether the cost is float arithmetic itself (EV3 ARM9 has no
FPU) or loop/dispatch overhead. No motors are touched here.

Press RIGHT (or wait 10 s) to start.
"""

from pybricks.tools import StopWatch, wait

_WATCH = StopWatch()

ev3 = None
try:
    from pybricks.hubs import EV3Brick
    ev3 = EV3Brick()
except Exception:
    ev3 = None


def say(msg):
    print(msg)
    if ev3 is not None:
        try:
            ev3.screen.print(msg[:22])
        except Exception:
            pass


def _clock():
    return _WATCH.time()


# ---- workloads -------------------------------------------------------------
# Each does the same number of multiply-adds: 36 rows x 12 columns.

FLOAT_W = [[0.25 + 0.01 * ((i * 12 + j) % 7) for j in range(12)] for i in range(36)]
INT_W = [[25 + ((i * 12 + j) % 7) for j in range(12)] for i in range(36)]
X = [0.1 * (j + 1) for j in range(12)]
XI = [10 + j for j in range(12)]


def float_mac(reps):
    total = 0.0
    for _ in range(reps):
        acc = 0.0
        for i in range(36):
            row = FLOAT_W[i]
            s = 0.0
            for j in range(12):
                s += row[j] * X[j]
            acc += s
        total += acc
    return total


def int_mac(reps):
    total = 0
    for _ in range(reps):
        acc = 0
        for i in range(36):
            row = INT_W[i]
            s = 0
            for j in range(12):
                s += row[j] * XI[j]
            acc += s
        total += acc
    return total


def float_sigmoid(reps):
    from math import exp
    total = 0.0
    for k in range(reps):
        for i in range(36):
            total += 1.0 / (1.0 + exp(-(0.1 * i - 1.0)))
    return total


def main():
    # No motor motion at all, so this starts on its own instead of waiting for
    # a button. A long button wait is what lets the brick fall asleep and drop
    # the USB link mid-run (observed 2026-11-10).
    say("bench running")
    wait(500)

    reps = 20
    t0 = _clock()
    float_mac(reps)
    t1 = _clock()
    dt_float = t1 - t0

    t2 = _clock()
    int_mac(reps)
    t3 = _clock()
    dt_int = t3 - t2

    t4 = _clock()
    float_sigmoid(reps)
    t5 = _clock()
    dt_sig = t5 - t4

    # 36 rows x 12 cols x 20 reps = 8640 MACs
    macs = 36 * 12 * reps
    print("BENCH float_mac_ms={} int_mac_ms={} float_sigmoid_ms={}".format(
        dt_float, dt_int, dt_sig))
    print("BENCH macs={} float_us_per_mac={:.2f} int_us_per_mac={:.2f}".format(
        macs, 1000.0 * dt_float / macs, 1000.0 * dt_int / macs))
    if dt_int > 0:
        print("BENCH speedup_x={:.1f}".format(float(dt_float) / dt_int))
    print("BENCH sigmoid_us_each={:.1f}".format(
        1000.0 * dt_sig / (36 * reps)))
    say("bench done")


main()
