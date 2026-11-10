"""model_parity.py — 用与 EV3 完全相同的输入序列，在 PC 上跑同一份导出权重。

目的：把 "EV3 实机日志" 与 "PC 参考实现" 的概率逐位对齐，证明
  (a) 上传到 hub 的权重就是 PC 上这份导出；
  (b) MicroPython 端 GRU 前向与纯 Python 参考实现一致（除了浮点舍入）。

    D:\\python\\python.exe tools\\model_parity.py
"""

import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
sys.path.insert(0, str(SRC))

import tap_probe_weights as weights  # noqa: E402
from contact_features import tap_probe_feature  # noqa: E402
from contact_inference import ContactInference  # noqa: E402

STEPS = 40
PRINT_EVERY = 5


def lcg(state):
    """与 src/_check_model.py 完全一致（小模数，EV3 只有 30 位小整数）。"""
    return (state * 1103 + 12345) % 65521


def features_for(i, state):
    state = lcg(state)
    load_l = ((state >> 8) % 400) - 200
    state = lcg(state)
    load_r = ((state >> 8) % 400) - 200
    state = lcg(state)
    dist_step = ((state >> 8) % 60) / 10.0
    state = lcg(state)
    progress = ((state >> 8) % 120) / 100.0
    phase = i % 7
    return tap_probe_feature(load_l, load_r, dist_step, progress,
                             is_free=(phase == 0),
                             is_early=(phase == 1 or phase == 2),
                             is_late=(phase == 5 or phase == 6),
                             probe_left=(phase == 3),
                             probe_right=(phase == 4)), state


def main():
    params = (len(weights.W_IH) * len(weights.W_IH[0]) + len(weights.W_HH) * len(weights.W_HH[0])
              + len(weights.B_IH) + len(weights.B_HH) + len(weights.W_OUT) + 1)
    print("MODEL_META name=tap_probe_gru_v1 input={} hidden={} threshold={} debounce_n={}".format(
        weights.INPUT_SIZE, weights.HIDDEN_SIZE, weights.THRESHOLD, weights.DEBOUNCE_N))
    print("MODEL_PARAMS total={}".format(params))
    net = ContactInference(weights)
    state = 12345
    hits = 0
    maxp = 0.0
    for i in range(STEPS):
        feat, state = features_for(i, state)
        prob, fired = net.update(feat)
        maxp = max(maxp, prob)
        hits += 1 if fired else 0
        if i % PRINT_EVERY == 0 or i == STEPS - 1:
            print("MODEL_STEP i={:02d} p={:.6f} fired={} run={}".format(i, prob, int(fired), net.run))
    print("MODEL_DONE steps={} max_p={:.6f} fired_count={}".format(STEPS, maxp, hits))


if __name__ == "__main__":
    main()
